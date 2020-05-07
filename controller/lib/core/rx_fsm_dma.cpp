constexpr uint8_t START_MARK = 0xFF;
constexpr uint8_t END_MARK = 0xFE;
constexpr uint8_t ESC_CHAR = 0xFD;

rx_buf[GuiStatus_size * 2 + 4 + 2];

typedef enum { STATE_WAIT_START, STATE_RX } RxState_t;

RxState_t state;

void setMatchChar(uint8_t c) {}

void setupDmaRx() {}

void stopDmaRx() {}

void onCharMatchIRQ() {
  if (STATE_WAIT_START == state) {
    setupDmaRx();
    setMatchChar(END_MARK);
    state = STATE_RX;
  } else if (STATE_RX == state) {
    stopDmaRx();
    unescapeFrame();
    decodeFrame();
    setMatchChar(START_MARK);
    state = STATE_WAIT_START;
  }
}

void onDmaFinishedIRQ() {
  // we shouldn't get here as end mark must happen before
}

void init() {
  state = STATE_WAIT_START;
  setMatchChar(START_MARK);
}
