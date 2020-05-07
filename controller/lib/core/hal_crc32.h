
#define CRC32_POLYNOMIAL 0x741B8CD7

#define CRC_BASE 0x40023000
struct CRC_Regs {
  REG data;
  REG iData;
  //	union{REG r; struct{REG :23, rev_out:1, rev_in:2, polysize:2, :2, reset
  //: 2;} bits;} ctrl;
  REG ctrl;
  REG reserved;
  REG init;
  REG polynomial;
};

#define RCC_BASE 0x40021000
struct RCC_Regs {
  REG clkCtrl;   // 0x00 clock control register (RCC_CR)
  REG clkCal;    // 0x04 Internal clock sources calibration register (RCC_ICSCR)
  REG clkCfg;    // 0x08 clock configuration register (RCC_CFGR)
  REG pllCfg;    // 0x0C PLL configuration register (RCC_PLLCFGR)
  REG pllSaiCfg; // 0x10 PLLSAI1 configuration register (RCC_PLLSAI1CFGR)
  REG rsvd1;
  REG clkIntEna; // 0x18 Clock interrupt enable register ( RCC_CIER)
  REG clkIntFlg; // 0x1C Clock interrupt flag register ( RCC_CIFR)
  REG clkIntClr; // 0x20 Clock interrupt clear register ( RCC_CICR)
  REG rsvd2;
  REG periphReset[8];  // 0x28 peripheral reset registers
  REG periphClkEna[8]; // 0x48 peripheral clock registers
  REG sleepClkEna[8];  // 0x68 Clock enable in sleep
  REG indClkCfg; // 0x88 Peripherals independent clock configuration register
                 // (RCC_CCIPR)
  REG rsvd3;
  REG backup;     // 0x90 Backup domain control register (RCC_BDCR)
  REG status;     // 0x94 control & status register (RCC_CSR)
  REG recovery;   // 0x98 Clock recovery RC register (RCC_CRRCR)
  REG indClkCfg2; // 0x9C Peripherals independent clock configuration register
                  // (RCC_CCIPR2)
};
CRC_Regs *crc = (CRC_Regs *)CRC_BASE;

void crcSetup() {
  RCC_Regs *rcc = (RCC_Regs *)RCC_BASE;
  rcc->periphClkEna[0] |= (1 << 12);
  rcc->periphReset[0] &= ~(1 << 12);
  crc->init = 0xFFFFFFFF;
  crc->polynomial = CRC32_POLYNOMIAL;
  crc->ctrl = 1;
}

uint32_t halCRC32(uint32_t d) {
  crc->data = d;
  // TODO CRC32 calculation takes 4 clock cycles,
  // analyze ASM code to make sure we have enough commands to spend this time
  // before reading CRC register
  return crc->data;
}
