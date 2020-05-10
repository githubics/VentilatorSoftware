#include "hal_stm32.h"

class UART3_DMA {
public:
  UART_Regs *const uart;
  DMA_Regs *const dma;

  bool tx_in_progress;

  UART3_DMA() : uart(UART3_BASE), dma(DMA1_BASE) {}

  void init(int baud) {
    // Set baud rate register
    uart->baud = CPU_FREQ / baud;
    // Enable the UART tx and rx
    uart->ctrl[0] = 0x000D;
  }

  bool transmit(const char *buf, uint32_t length) {
    if (tx_in_progress) {
      return false;
    }

    // UART3 transfers happen on DMA1 channel 2
    dma->chanSel.c2s = 0b0010;
    dma->channel[1].config.mem2mem = 0;
    dma->channel[1].config.priority = 0b11; // high priority
    dma->channel[1].config.msize = 0;       // 8 bits
    dma->channel[1].config.psize = 0;       // 8bits
    dma->channel[1].config.memInc = 1;      // increment memory address
    dma->channel[1].config.perInc = 0;   // don't increment peripheral address
    dma->channel[1].config.circular = 0; // not circular
    dma->channel[1].config.dir = 1;      // memory to peripheral
    dma->channel[1].config.teie = 0;     // TODO set this to process tx errors
    dma->channel[1].config.htie = 0;     // no half-transfer interrupt
    dma->channel[1].config.tcie = 1;     // TX complete interrupt enabled
    dma->channel[1].config.enable = 0;   // don't enable yet

    dma->channel[1].count = length;
    dma->channel[1].pAddr = reinterpret_cast<REG>(&(uart->txDat));
    dma->channel[1].mAddr = reinterpret_cast<REG>(buf);

    return true;
  }

  // called from DMA1_CH2_ISR
  void onTxComplete() {}

  void setCharMatchIRQ(char c) {}
};

static void DMA1_CH2_ISR() {}
