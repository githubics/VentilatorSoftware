#include "hal_stm32.h"

// STM32 UART3 driver based on DMA transfers.

// Direct Memory Access mode in MCU allows to set up a memory buffer
// transfer by means of hardware with no CPU intervention.
// CPU responsibility is to set up a DMA channel configuring it's
// endpoints as peripheral and memory. Upon DMA transfer completion
// CPU is notified via interrupt.

class UART3_DMA {
public:
  UART_Regs *const uart;
  DMA_Regs *const dma;

  bool tx_in_progress;
  bool rx_in_progress;

  UART3_DMA() : uart(UART3_BASE), dma(DMA1_BASE) {}

  void init(int baud) {
    // Set baud rate register
    uart->baud = CPU_FREQ / baud;
    // Enable the UART tx and rx
    uart->ctrl[0] = 0x000D;

    // TODO Enable Hardware flow controll
  }

  bool isTxInProgress() {
    // TODO thread safety
    return tx_in_progress;
  }

  bool isRxInProgress() {
    // TODO thread safety
    return rx_in_progress;
  }

  bool transmit(const char *buf, uint32_t length) {
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
    dma->channel[1].config.perInc =
        0; // don't increment dest (peripheral) address
    dma->channel[1].config.circular = 0; // not circular
    dma->channel[1].config.dir = 1;      // memory to peripheral
    dma->channel[1].config.teie = 1;     // interrupt on error
    dma->channel[1].config.htie = 0;     // no half-transfer interrupt
    dma->channel[1].config.tcie = 1;     // DMA complete interrupt enabled
    dma->channel[1].config.enable = 0;   // don't enable yet

    dma->channel[1].count = length;
    dma->channel[1].pAddr = reinterpret_cast<REG>(&(uart->txDat));
    dma->channel[1].mAddr = reinterpret_cast<REG>(buf);

    uart->ctrl[2] |= 0x0080;  // set DMAT bit to enable DMA for transmitter
    uart->intClear |= 0x0040; // Clear transmit complete flag
    dma->channel[1].config.enable = 1; // go!

    tx_in_progress = true;

    return true;
  }

  void interruptOnChar(char c) {
    uart->ctrl[0] =
        0x0008; // TODO Disable reader in order to enable characted detection
                //    uart->ctrl[1].addr = stopChar; // TODO
    uart->ctrl[0] = 0x000D; // TODO enable receiver and Character match
                            // interrupt
  }

  bool setupDmaRx(const char *buf, const uint32_t length) {
    // UART3 reception happens on DMA1 channel 3
    if (isRxInProgress()) {
      return false;
    }

    dma->chanSel.c2s = 0b0011;
    dma->channel[2].config.mem2mem = 0;
    dma->channel[2].config.priority = 0b11; // high priority
    dma->channel[2].config.msize = 0b00;    // 8 bits
    dma->channel[2].config.psize = 0b00;    // 8 bits
    dma->channel[2].config.memInc = 1;      // increment destination (memory)
    dma->channel[2].config.perInc =
        0; // don't increment source (peripheral) address
    dma->channel[2].config.circular = 0; // not circular
    dma->channel[2].config.dir = 0;      // peripheral to memory
    dma->channel[2].config.teie = 1;     // interrupt on error
    dma->channel[2].config.htie = 0;     // no half-transfer interrupt
    dma->channel[2].config.tcie = 1;     // interrupt on DMA complete
    dma->channel[2].config.enable = 0;   // don't enable yet

    dma->channel[2].count = length;
    dma->channel[2].pAddr = reinterpret_cast<REG>(buf);
    dma->channel[2].mAddr = reinterpret_cast<REG>(&(uart->txDat));

    uart->ctrl[2] |= 0x0040; // set DMAR bit to enable DMA for receiver
    uart->request |= 0x0008; // Clear RXNE flag
    dma->channel[1].config.enable = 1; // go!

    rx_in_progress = true;

    return true;
  }

  // called from DMA1_CH3_ISR
  void onRxDmaComplete() {
    // Disable DMA channel
    dma->channel[2].config.enable = 0;
    rx_in_progress = false;
  }

  // called from DMA1_CH2_ISR
  void onTxDmaComplete() {
    // Disable DMA channel
    dma->channel[1].config.enable = 0;
    tx_in_progress = false;
  }

  // called from UART3_ISR
  void onRxCharacterMatch() {
    // Disable RX DMA channel if it was set up
    if (rx_in_progress) {
      dma->channel[2].config.enable = 0;
      rx_in_progress = false;
    }
  }
};

UART3_DMA dmaUart = UART3_DMA();

void DMA1_CH2_ISR() { dmaUart.onTxDmaComplete(); }

void DMA1_CH3_ISR() { dmaUart.onRxDmaComplete(); }

inline bool isCharacterMatchInterrupt() {
  // TODO
  // UART_Regs *reg = UART3_BASE;
  return false;
}

// This is the interrupt handler for the UART.
void UART3_ISR() {
  // Check for over run error and framing errors.
  // Clear those errors if they're set to avoid
  // further interrupts from them.

  UART_Regs *reg = UART3_BASE;
  if (reg->status & 0x000A)
    reg->intClear = 0x000A;

  if (isCharacterMatchInterrupt()) {
    dmaUart.onRxCharacterMatch();
  }
}
