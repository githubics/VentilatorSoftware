constexpr uint8_t MARK_START = 0xFF;
constexpr uint8_t MARK_END = 0xFE;
constexpr uint8_t MARK_ESC = 0xFD;

typedef enum {
  OK,
  ERROR_DATA_BEFORE_START,
  ERROR_REPEATED_START,
  ERROR_BUFFER_OVERFLOW,
  ERROR_CRC
} RxResult_t;

typedef enum { STATE_WAIT_START, STATE_RX, STATE_ESC } RxState_t;

RxState_t state = STATE_WAIT_START;
RxResult_t rxResult = OK;

rx_buffer[GuiState_size];
rx_index = 0;

bool saveRxChar(uint8_t c) {
  if (rx_index >= GuiState_size) {
    return false;
  }
  rx_buffer[rx_index++] = c;
  return true;
}

uint8_t unescape(uint8_t c) { return c; }

RxResult_t decodeFrame() {
  if (rx_index < 1) {
    return false;
  }
  uint32_t crc_packet =
      rx_buffer[rx_index - 4] << 24 | rx_buffer[rx_index - 3] << 16 |
      rx_buffer[rx_index - 2] << 8 |
      rx_buffer[rx_index - 1] if (0 != crc32_check_hw(rx_buffer, rx_index - 4,
                                                      crc_packet)) {
    return ERROR_CRC;
  }
  else {
    if (guiStatusRingBuffer.append(decodeGuiStatus(rx_buffer, rx_index - 4))) {
      frame_pending = true;
      return OK;
    } else {
      return ERROR_RX_OVERRUN;
    }
  }
}

void onCharReceived(uint8_t c) {
  switch (state) {
  case STATE_WAIT_START:
    if (MARK_START == c) {
      rx_index = 0;
      state = STATE_RX;
    } else {
      rxResult = ERROR_DATA_BEFORE_START;
    }
    break;
  case STATE_RX:
    if (MARK_START == c) {
      rxResult = ERROR_REPEATED_START;
      state = STATE_WAIT_START;
    } else if (MARK_END == c) {
      rxResult = decodeFrame();
      state = STATE_WAIT_START;
    } else if (MARK_ESC == c) {
      state = STATE_ESC
    } else {
      if (!saveRxChar(c)) {
        rxResult = ERROR_BUFFER_OVERFLOW;
        state = STATE_WAIT_START;
      }
    }
    break;
  case STATE_ESC:
    if (MARK_START == c || MARK_END == c) {
      state = STATE_ERROR;
    } else {
      if (!saveRxChar(unescape(c))) {
        rxResult = ERROR_BUFFER_OVERFLOW;
        state = STATE_WAIT_START;
      } else {
        state = STATE_RX;
      }
    }
  }
}

void process() {}
