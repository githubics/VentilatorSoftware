#if defined(BARE_STM32)

#include "hal_stm32.h"

// STM32 UART3 driver based on DMA transfers.

// Direct Memory Access mode in MCU allows to set up a memory buffer
// transfer by means of hardware with no CPU intervention.
// CPU responsibility is to set up a DMA channel configuring it's
// endpoints as peripheral and memory. Upon DMA transfer completion
// CPU is notified via interrupt.

// This driver also provides Character Match interrupt on reception.
// UART is setup to issue an interrupt upot the reception of the
// specified character.
// NOTE: interrupt is disabled after the first match, call
// interruptOnChar to set up a an interrupt again.

// An interface that gets called back by the driver on rx, tx complete
// and rx character match events.
// NOTE: all callbacks are called from interrupt context!
class UART_DMA_Listener {
public:
  // Called on DMA RX complete
  virtual void onRxComplete() = 0;
  // Called on DMA TX complete
  virtual void onTxComplete() = 0;
  // Called on specified character reception
  virtual void onCharacterMatch() = 0;
  // Called on RX errors
  virtual void onRxError() = 0;
  // Called on TX errors
  virtual void onTxError() = 0;
};

class UART3_DMA {
  UART_Regs *const uart;
  DMA_Regs *const dma;
  UART_DMA_Listener &eventListener;

public:
  bool tx_in_progress;
  bool rx_in_progress;

  // NOTE: UART3 is hardcoded here because RespiraWorks cycle controller
  // uses UART3 to communicate with Raspberry Pi.
  // Class is not generic to avoid excessive code while resolving
  // DMA channel numbers that have to be set up.
  // Specifically USART3_TX is on DMA channel 2 and USART3_RX on channel 3.
  UART3_DMA(UART_DMA_Listener &listener)
      : uart(UART3_BASE), dma(DMA1_BASE), eventListener(listener) {}

  // Performs basic UART3 initialization
  void init(int baud) {
    // Set baud rate register
    uart->baud = CPU_FREQ / baud;

    uart->ctrl3.dmar = 1; // set DMAR bit to enable DMA for receiver
    uart->ctrl3.dmat = 1; // set DMAT bit to enable DMA for transmitter
    uart->ctrl1.te = 1;   // enable transmitter
    uart->ctrl1.re = 1;   // enable receiver
    uart->ctrl1.ue = 1;   // enable uart

    // TODO Enable Hardware flow controll
    // TODO enable parity checking
  }

  // Returns true if DMA TX is in progress
  bool isTxInProgress() {
    // TODO thread safety
    return tx_in_progress;
  }

  // Returns true if DMA RX is in progress
  bool isRxInProgress() {
    // TODO thread safety
    return rx_in_progress;
  }

  // Sets up UART3 to transfer [length] characters from [buf]
  // Returns false if DMA transmission is in progress, does not
  // interrupt previous transmission.
  // Returns true if no transmission is in progress
  bool startTX(const char *buf, uint32_t length) {
    if (isTxInProgress()) {
      return false;
    }

    // UART3 transmission happens on DMA1 channel 2
    dma->chanSel.c2s = 0b0010;
    dma->channel[1].config.mem2mem = 0;
    dma->channel[1].config.priority = 0b11; // high priority
    dma->channel[1].config.msize = 0;       // 8 bits
    dma->channel[1].config.psize = 0;       // 8 bits
    dma->channel[1].config.memInc = 1;      // increment source (memory) address
    dma->channel[1].config.perInc = 0;      // don't increment dest (peripheral)
                                            // address
    dma->channel[1].config.circular = 0;    // not circular
    dma->channel[1].config.dir = 1;         // memory to peripheral
    dma->channel[1].config.teie = 1;        // interrupt on error
    dma->channel[1].config.htie = 0;        // no half-transfer interrupt
    dma->channel[1].config.tcie = 1;        // DMA complete interrupt enabled
    dma->channel[1].config.enable = 0;      // don't enable yet

    dma->channel[1].count = length;
    dma->channel[1].pAddr = reinterpret_cast<REG>(&(uart->txDat));
    dma->channel[1].mAddr = reinterpret_cast<REG>(buf);

    uart->intClear |= (1 << 6);        // Clear transmit complete flag
    dma->channel[1].config.enable = 1; // go!

    tx_in_progress = true;

    return true;
  }

  void stopTX() {
    if (isTxInProgress()) {
      dma->channel[1].config.enable = 0;
      // TODO thread safety
      tx_in_progress = 0;
    }
  }

  // Sets up reception of at least [length] chars from UART3 into [buf]
  // [timeout] is the number of baudrate bits for which RX line is
  // allowed to be idle before asserting timeout error.
  // Returns false if reception is in progress, new reception is not
  // setup. Returns true if no reception is in progress and new reception
  // was setup.

  bool startRX(const char *buf, const uint32_t length, uint32_t timeout) {
    // UART3 reception happens on DMA1 channel 3
    if (isRxInProgress()) {
      return false;
    }
    // max timeout is 24 bit
    timeout = timeout & 0x00FFFFFF;

    dma->chanSel.c2s = 0b0011;
    dma->channel[2].config.mem2mem = 0;
    dma->channel[2].config.priority = 0b11; // high priority
    dma->channel[2].config.msize = 0b00;    // 8 bits
    dma->channel[2].config.psize = 0b00;    // 8 bits
    dma->channel[2].config.memInc = 1;      // increment destination (memory)
    dma->channel[2].config.perInc = 0;      // don't increment source
                                            // (peripheral) address
    dma->channel[2].config.circular = 0;    // not circular
    dma->channel[2].config.dir = 0;         // peripheral to memory
    dma->channel[2].config.teie = 1;        // interrupt on error
    dma->channel[2].config.htie = 0;        // no half-transfer interrupt
    dma->channel[2].config.tcie = 1;        // interrupt on DMA complete
    dma->channel[2].config.enable = 0;      // don't enable yet

    dma->channel[2].count = length;
    dma->channel[2].pAddr = reinterpret_cast<REG>(buf);
    dma->channel[2].mAddr = reinterpret_cast<REG>(&(uart->txDat));

    uart->request |= (1 << 3); // Clear RXNE flag

    uart->ctrl3.ddre = 1;  // DMA is disabled following a reception error
    uart->ctrl2.rtoen = 1; // Enable receive timeout interrupt
    uart->timeout = timeout;

    dma->channel[2].config.enable = 1; // go!

    rx_in_progress = true;

    return true;
  }

  void stopRX() {
    if (isRxInProgress()) {
      uart->ctrl2.rtoen = 0; // Disable receive timeout interrupt
      dma->channel[2].config.enable = 0;
      // TODO thread safety
      rx_in_progress = 0;
    }
  }

  // Sets up an interrupt on matching char incomming form UART3
  void interruptOnChar(char c) {
    uart->ctrl1.re = 0; // Disable receiver in order to enable
                        // characted detection
    uart->ctrl2.addr = c;
    uart->ctrl1.cmie = 1; // Enable character match interrupt
    uart->ctrl1.re = 1;   // Enable receiver
  }

  // called from DMA1_CH3_ISR
  void onDmaRxComplete() {
    // Disable DMA channel
    dma->channel[2].config.enable = 0;
    rx_in_progress = false;
    eventListener.onRxComplete();
  }

  // called from DMA1_CH2_ISR
  void onDmaTxComplete() {
    // Disable DMA channel
    dma->channel[1].config.enable = 0;
    tx_in_progress = false;
    eventListener.onTxComplete();
  }

  // called from UART3_ISR
  void onRxCharacterMatch() {
    uart->ctrl1.re = 0;   // Disable receiver in order to disable
                          // characted detection
    uart->ctrl1.cmie = 0; // Enable character match interrupt
    uart->ctrl1.re = 1;   // Enable receiver
    eventListener.onCharacterMatch();
  }

  void onDmaRxError() {
    // TODO clear error flags
    stopRX();
    eventListener.onRxError();
  }

  void onDmaTxError() {
    // TODO clear error flags
    stopTX();
    eventListener.onTxError();
  }

  void onUartRxError() {
    stopRX();
    eventListener.onRxError();
  }
};

class DummyListener : public UART_DMA_Listener {
public:
  void onRxComplete() {}
  void onTxComplete() {}
  void onCharacterMatch() {}
  void onRxError(){};
  void onTxError(){};
};

DummyListener listener = DummyListener();
UART3_DMA dmaUart = UART3_DMA(listener);

void DMA1_CH2_ISR() {
  DMA_Regs *const dma = DMA1_BASE;
  if (dma->intStat.teif2) {
    dmaUart.onDmaTxError();
  } else {
    dmaUart.onDmaTxComplete();
  }
}

void DMA1_CH3_ISR() {
  DMA_Regs *const dma = DMA1_BASE;
  if (dma->intStat.teif3) {
    dmaUart.onDmaRxError();
  } else {
    dmaUart.onDmaRxComplete();
  }
}

inline bool isCharacterMatchInterrupt() {
  UART_Regs *const uart = UART3_BASE;
  return 0 != (uart->status & (1 << 17));
}

inline bool isRxError() {
  UART_Regs *const uart = UART3_BASE;

  return 0 != (uart->status & (1 << 11)) || // RTOF - Receiver timeout
         0 != (uart->status & (1 << 3)) ||  // ORE - Overrun error
         0 != (uart->status & (1 << 1));    // FE - frame error

  // 0 != uart->status & (1 << 0)  || // PE - parity error
  // 0 != uart->status & (1 << 2)  || // START bit Noise detection flag
}

// This is the interrupt handler for the UART.
void UART3_ISR() {
  UART_Regs *const uart = UART3_BASE;
  if (isRxError()) {
    uart->request |= (1 << 3); // Clear RXNE flag before clearing other flags
    uart->intClear = (1 << 11) | (1 << 3) | (1 << 1); // Clear interrupt flags
    dmaUart.onUartRxError();
  }

  if (isCharacterMatchInterrupt()) {
    dmaUart.onRxCharacterMatch();
  }
}

#endif
