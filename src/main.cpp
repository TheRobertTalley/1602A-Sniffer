#include <Arduino.h>

#if defined(ARDUINO_AVR_MICRO)
// Arduino Micro wiring traced by the user:
// E = D12
// D0 = D11, D1 = D10, D2 = D9, D3 = D8, D4 = D7, D5 = D6, D6 = D5
// The Arduino-side D3/D4 wires are swapped, so D7 = D3 and RW = D4.
// RS remains on D2.
static const uint8_t PIN_RS = 2;
static const uint8_t PIN_RW = 4;
static const uint8_t PIN_E = 12;
static const uint8_t PIN_D0 = 11;
static const uint8_t PIN_D1 = 10;
static const uint8_t PIN_D2 = 9;
static const uint8_t PIN_D3 = 8;
static const uint8_t PIN_D4 = 7;
static const uint8_t PIN_D5 = 6;
static const uint8_t PIN_D6 = 5;
static const uint8_t PIN_D7 = 3;
#else
// Original Uno wiring.
static const uint8_t PIN_E = 2;
static const uint8_t PIN_RS = 4;
static const uint8_t PIN_RW = 5;
static const uint8_t PIN_D0 = A0;
static const uint8_t PIN_D1 = A1;
static const uint8_t PIN_D2 = A2;
static const uint8_t PIN_D3 = A3;
static const uint8_t PIN_D4 = 6;
static const uint8_t PIN_D5 = 7;
static const uint8_t PIN_D6 = 8;
static const uint8_t PIN_D7 = 9;
#endif

// Thresholds are in uSv per hour unless noted.
static const float THRESH_FOUND_DELTA_USVPH = 0.20f;
static const uint16_t FOUND_HOLD_MS = 3000;

static const float THRESH_FLIGHT_LOW_USVPH = 1.00f;
static const float THRESH_FLIGHT_HIGH_USVPH = 10.0f;
static const uint16_t FLIGHT_HOLD_MS = 15000;

static const float THRESH_DANGER_USVPH = 10.0f;
static const float THRESH_UNHEALTHY_USVPH = 100.0f;

static const float TAU_EMA_1M_S = 60.0f;
static const float TAU_EMA_10M_S = 600.0f;

static const uint16_t RB_SIZE = 256;
volatile uint8_t rbMeta[RB_SIZE];
volatile uint8_t rbData[RB_SIZE];
volatile uint16_t rbHead = 0;
volatile uint16_t rbTail = 0;
static bool usePolledE = false;
#if defined(ARDUINO_AVR_MICRO)
static uint8_t microLastPortD = 0;
#else
static uint8_t lastEState = 0;
#endif

static inline void pushSample(uint8_t rs, uint8_t rw, uint8_t data) {
  uint16_t nextHead = rbHead + 1;
  if (nextHead >= RB_SIZE) {
    nextHead = 0;
  }
  if (nextHead == rbTail) {
    return;
  }

  rbMeta[rbHead] = static_cast<uint8_t>((rs << 7) | (rw << 6));
  rbData[rbHead] = data;
  rbHead = nextHead;
}

#if defined(ARDUINO_AVR_MICRO)
static inline uint8_t readMicroByteFromPorts(uint8_t portB,
                                             uint8_t portD,
                                             uint8_t portC,
                                             uint8_t portE) {
  uint8_t b = 0;
  b |= ((portB >> 7) & 0x01) << 0; // D11 -> LCD D0
  b |= ((portB >> 6) & 0x01) << 1; // D10 -> LCD D1
  b |= ((portB >> 5) & 0x01) << 2; // D9  -> LCD D2
  b |= ((portB >> 4) & 0x01) << 3; // D8  -> LCD D3
  b |= ((portE >> 6) & 0x01) << 4; // D7  -> LCD D4
  b |= ((portD >> 7) & 0x01) << 5; // D6  -> LCD D5
  b |= ((portC >> 6) & 0x01) << 6; // D5  -> LCD D6
  b |= ((portD >> 0) & 0x01) << 7; // D3  -> LCD D7
  return b;
}

static inline void captureMicroFromPorts(uint8_t portD) {
  const uint8_t portB = PINB;
  const uint8_t portC = PINC;
  const uint8_t portE = PINE;
  const uint8_t rs = static_cast<uint8_t>((portD >> 1) & 0x01); // D2
  const uint8_t rw = static_cast<uint8_t>((portD >> 4) & 0x01); // D4
  pushSample(rs, rw, readMicroByteFromPorts(portB, portD, portC, portE));
}

static void pollERiseBurst(uint16_t iterations) {
  while (iterations-- > 0) {
    const uint8_t portD = PIND;
    const bool riseE = ((microLastPortD & _BV(6)) == 0) && ((portD & _BV(6)) != 0);
    if (riseE) {
      captureMicroFromPorts(portD);
    }
    microLastPortD = portD;
  }
}
#else
static inline uint8_t readByteBus() {
  uint8_t b = 0;
  b |= (digitalRead(PIN_D0) & 1) << 0;
  b |= (digitalRead(PIN_D1) & 1) << 1;
  b |= (digitalRead(PIN_D2) & 1) << 2;
  b |= (digitalRead(PIN_D3) & 1) << 3;
  b |= (digitalRead(PIN_D4) & 1) << 4;
  b |= (digitalRead(PIN_D5) & 1) << 5;
  b |= (digitalRead(PIN_D6) & 1) << 6;
  b |= (digitalRead(PIN_D7) & 1) << 7;
  return b;
}

void onERise() {
  const uint8_t rs = static_cast<uint8_t>(digitalRead(PIN_RS) & 1);
  const uint8_t rw = static_cast<uint8_t>(digitalRead(PIN_RW) & 1);
  pushSample(rs, rw, readByteBus());
}

static void pollERiseBurst(uint16_t iterations) {
  while (iterations-- > 0) {
    const uint8_t eState = static_cast<uint8_t>(digitalRead(PIN_E) & 1);
    if (lastEState == 0 && eState != 0) {
      onERise();
    }
    lastEState = eState;
  }
}
#endif

struct Capture {
  uint8_t meta;
  uint8_t data;
};

static bool popCapture(Capture &out) {
  noInterrupts();
  if (rbTail == rbHead) {
    interrupts();
    return false;
  }

  const uint16_t tail = rbTail;
  out.meta = rbMeta[tail];
  out.data = rbData[tail];

  uint16_t nextTail = tail + 1;
  if (nextTail >= RB_SIZE) {
    nextTail = 0;
  }
  rbTail = nextTail;
  interrupts();
  return true;
}

static char lcd[2][16];
static char lastLcd[2][16];
static uint8_t cursor = 0;
#if defined(ARDUINO_AVR_MICRO)
static char microPendingLcd[2][16];
static bool microPendingValid = false;
#endif

static void clearBuf(char buf[2][16]) {
  for (uint8_t row = 0; row < 2; row++) {
    for (uint8_t col = 0; col < 16; col++) {
      buf[row][col] = ' ';
    }
  }
}

static bool sameBuf(const char a[2][16], const char b[2][16]) {
  for (uint8_t row = 0; row < 2; row++) {
    for (uint8_t col = 0; col < 16; col++) {
      if (a[row][col] != b[row][col]) {
        return false;
      }
    }
  }
  return true;
}

static void copyBuf(char dst[2][16], const char src[2][16]) {
  for (uint8_t row = 0; row < 2; row++) {
    for (uint8_t col = 0; col < 16; col++) {
      dst[row][col] = src[row][col];
    }
  }
}

static inline bool ddramToRowCol(uint8_t addr, uint8_t &row, uint8_t &col) {
  if (addr < 0x10) {
    row = 0;
    col = addr;
    return true;
  }
  if (addr >= 0x40 && addr < 0x50) {
    row = 1;
    col = static_cast<uint8_t>(addr - 0x40);
    return true;
  }
  return false;
}

static bool lcdChanged() {
  for (uint8_t row = 0; row < 2; row++) {
    for (uint8_t col = 0; col < 16; col++) {
      if (lcd[row][col] != lastLcd[row][col]) {
        return true;
      }
    }
  }
  return false;
}

static void commitLcd() {
  for (uint8_t row = 0; row < 2; row++) {
    for (uint8_t col = 0; col < 16; col++) {
      lastLcd[row][col] = lcd[row][col];
    }
  }
}

static void copyLine(char *dst17, const char src[16]) {
  for (uint8_t i = 0; i < 16; i++) {
    dst17[i] = src[i];
  }
  dst17[16] = '\0';
}

static inline bool isDigitChar(char c) {
  return c >= '0' && c <= '9';
}

static bool lineHasToken(const char line[16], const char *token) {
  for (uint8_t i = 0; i < 16; i++) {
    uint8_t j = 0;
    while (token[j] != '\0' && (i + j) < 16 && line[i + j] == token[j]) {
      j++;
    }
    if (token[j] == '\0') {
      return true;
    }
  }
  return false;
}

static bool parseValueForPrefix(const char line[16], char wantPrefix, float &out) {
  int prefixPos = -1;
  for (uint8_t i = 0; i < 16; i++) {
    if (line[i] == wantPrefix && (i + 1 < 16) && line[i + 1] == ':') {
      prefixPos = i;
      break;
    }
  }
  if (prefixPos < 0) {
    return false;
  }

  int i = prefixPos + 2;
  while (i < 16 && line[i] == ' ') {
    i++;
  }
  if (i >= 16) {
    return false;
  }

  long intPart = 0;
  int digits = 0;
  while (i < 16 && isDigitChar(line[i]) && digits < 4) {
    intPart = (intPart * 10L) + (line[i] - '0');
    i++;
    digits++;
  }
  if (digits < 1) {
    return false;
  }

  if (i >= 16 || line[i] != '.') {
    return false;
  }
  i++;

  if (i + 1 >= 16) {
    return false;
  }
  if (!isDigitChar(line[i]) || !isDigitChar(line[i + 1])) {
    return false;
  }

  const int fracPart = (line[i] - '0') * 10 + (line[i + 1] - '0');
  out = static_cast<float>(intPart) + (static_cast<float>(fracPart) / 100.0f);
  return true;
}

static bool parseRateAndAvg(float &rate_usvph, bool &haveRate, float &avg_usvph, bool &haveAvg) {
  haveRate = false;
  haveAvg = false;

  float value = 0.0f;

  if (parseValueForPrefix(lcd[0], 'R', value) || parseValueForPrefix(lcd[1], 'R', value)) {
    rate_usvph = value;
    haveRate = true;
  } else if (parseValueForPrefix(lcd[0], 'S', value) || parseValueForPrefix(lcd[1], 'S', value)) {
    rate_usvph = value;
    haveRate = true;
  }

  if (parseValueForPrefix(lcd[0], 'A', value) || parseValueForPrefix(lcd[1], 'A', value)) {
    avg_usvph = value;
    haveAvg = true;
  }

  return haveRate || haveAvg;
}

static float peak_usvph = 0.0f;
static float dose_uSv = 0.0f;
static bool emaInit = false;
static float ema1m_usvph = 0.0f;
static float ema10m_usvph = 0.0f;
static uint32_t lastUpdateMs = 0;
static uint32_t foundStartMs = 0;
static uint32_t flightStartMs = 0;

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) {
    return lo;
  }
  if (x > hi) {
    return hi;
  }
  return x;
}

static void updateEma(float &ema, float x, float dt_s, float tau_s) {
  float alpha = dt_s / (tau_s + dt_s);
  alpha = clampf(alpha, 0.0f, 1.0f);
  ema += alpha * (x - ema);
}

static inline float usvph_to_radph(float usvph) {
  return usvph * 1e-6f * 100.0f;
}

static inline float usvph_to_rads(float usvph) {
  return usvph_to_radph(usvph) / 3600.0f;
}

static inline float uSv_to_rad(float uSv) {
  return uSv * 1e-6f * 100.0f;
}

static void printMetrics(float rate_usvph, bool haveRate, float avg_usvph, bool haveAvg) {
  char line1[17];
  char line2[17];
  copyLine(line1, lcd[0]);
  copyLine(line2, lcd[1]);

  const float x_usvph = haveRate ? rate_usvph : (haveAvg ? avg_usvph : 0.0f);

  const uint32_t nowMs = millis();
  float dt_s = 0.0f;
  if (lastUpdateMs != 0) {
    dt_s = static_cast<float>(nowMs - lastUpdateMs) / 1000.0f;
  }
  lastUpdateMs = nowMs;

  if (!emaInit) {
    ema1m_usvph = x_usvph;
    ema10m_usvph = x_usvph;
    emaInit = true;
  } else if (dt_s > 0.0f && dt_s < 10.0f) {
    updateEma(ema1m_usvph, x_usvph, dt_s, TAU_EMA_1M_S);
    updateEma(ema10m_usvph, x_usvph, dt_s, TAU_EMA_10M_S);
  }

  if (haveRate && rate_usvph > peak_usvph) {
    peak_usvph = rate_usvph;
  }

  if (dt_s > 0.0f && dt_s < 10.0f) {
    dose_uSv += x_usvph * (dt_s / 3600.0f);
  }

  const float alarmVal = haveRate ? rate_usvph : x_usvph;

  const bool alarmUnhealthy = (alarmVal >= THRESH_UNHEALTHY_USVPH);
  const bool alarmDanger = (alarmVal >= THRESH_DANGER_USVPH);

  const bool inFlightBand =
      (alarmVal >= THRESH_FLIGHT_LOW_USVPH && alarmVal <= THRESH_FLIGHT_HIGH_USVPH);
  if (inFlightBand) {
    if (flightStartMs == 0) {
      flightStartMs = nowMs;
    }
  } else {
    flightStartMs = 0;
  }
  const bool alarmFlight = (flightStartMs != 0 && (nowMs - flightStartMs) >= FLIGHT_HOLD_MS);

  const bool aboveBaseline = (alarmVal >= (ema10m_usvph + THRESH_FOUND_DELTA_USVPH));
  if (aboveBaseline) {
    if (foundStartMs == 0) {
      foundStartMs = nowMs;
    }
  } else {
    foundStartMs = 0;
  }
  const bool alarmFound = (foundStartMs != 0 && (nowMs - foundStartMs) >= FOUND_HOLD_MS);

  const float rate_radph = usvph_to_radph(alarmVal);
  const float rate_rads = usvph_to_rads(alarmVal);
  const float dose_rad = uSv_to_rad(dose_uSv);

  Serial.print("LCD|");
  Serial.print(line1);
  Serial.print("|");
  Serial.print(line2);
  Serial.println("|");

  Serial.print("rate_usvph=");
  if (haveRate) {
    Serial.print(rate_usvph, 2);
  } else {
    Serial.print("NA");
  }

  Serial.print(" avg_usvph=");
  if (haveAvg) {
    Serial.print(avg_usvph, 2);
  } else {
    Serial.print("NA");
  }

  Serial.print(" ema1m_usvph=");
  Serial.print(ema1m_usvph, 2);
  Serial.print(" ema10m_usvph=");
  Serial.print(ema10m_usvph, 2);

  Serial.print(" peak_usvph=");
  Serial.print(peak_usvph, 2);
  Serial.print(" dose_uSv=");
  Serial.print(dose_uSv, 3);

  Serial.print(" rate_radph=");
  Serial.print(rate_radph, 8);
  Serial.print(" rate_rads=");
  Serial.print(rate_rads, 10);
  Serial.print(" dose_rad=");
  Serial.print(dose_rad, 8);

  Serial.print(" alarm=");
  bool any = false;
  if (alarmUnhealthy) {
    Serial.print("UNHEALTHY");
    any = true;
  }
  if (alarmDanger) {
    Serial.print(any ? "," : "");
    Serial.print("DANGER");
    any = true;
  }
  if (alarmFound) {
    Serial.print(any ? "," : "");
    Serial.print("FOUND");
    any = true;
  }
  if (alarmFlight) {
    Serial.print(any ? "," : "");
    Serial.print("FLIGHT");
    any = true;
  }
  if (!any) {
    Serial.print("OK");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);

#if defined(USBCON)
  const uint32_t usbWaitStartMs = millis();
  while (!Serial && (millis() - usbWaitStartMs) < 4000) {
  }
#endif

  pinMode(PIN_E, INPUT);
  pinMode(PIN_RS, INPUT);
  pinMode(PIN_RW, INPUT);
  pinMode(PIN_D0, INPUT);
  pinMode(PIN_D1, INPUT);
  pinMode(PIN_D2, INPUT);
  pinMode(PIN_D3, INPUT);
  pinMode(PIN_D4, INPUT);
  pinMode(PIN_D5, INPUT);
  pinMode(PIN_D6, INPUT);
  pinMode(PIN_D7, INPUT);

  clearBuf(lcd);
  clearBuf(lastLcd);
#if defined(ARDUINO_AVR_MICRO)
  clearBuf(microPendingLcd);
#endif
  cursor = 0;

#if defined(ARDUINO_AVR_MICRO)
  usePolledE = true;
  microLastPortD = PIND;
#else
  const int interruptNumber = digitalPinToInterrupt(PIN_E);
  if (interruptNumber == NOT_AN_INTERRUPT) {
    usePolledE = true;
    lastEState = static_cast<uint8_t>(digitalRead(PIN_E) & 1);
  } else {
    attachInterrupt(interruptNumber, onERise, RISING);
  }
#endif
}

void loop() {
  static uint32_t lastBusMs = 0;
  static uint32_t lastCaptureMs = 0;

  if (usePolledE) {
    pollERiseBurst(4000);
  }

  Capture capture;
  while (popCapture(capture)) {
    const uint32_t nowMs = millis();
    if (lastCaptureMs != 0 && (nowMs - lastCaptureMs) > 500) {
#if defined(ARDUINO_AVR_MICRO)
      microPendingValid = false;
#endif
    }
    lastCaptureMs = nowMs;

    uint8_t rs = static_cast<uint8_t>((capture.meta >> 7) & 1);
    uint8_t rw = static_cast<uint8_t>((capture.meta >> 6) & 1);
#if defined(ARDUINO_AVR_MICRO)
    // With the traced Micro harness, the data byte is correct but RS/RW are not.
    // Infer write-vs-command from the HD44780 byte stream so we can still mirror text.
    rw = 0;
    rs = ((capture.data & 0x80) != 0 || capture.data == 0x01 || capture.data == 0x02 ||
          capture.data < 0x20)
             ? 0
             : 1;
#endif
    if (rw) {
      continue;
    }

    lastBusMs = millis();

    if (rs == 0) {
      if (capture.data == 0x01 || capture.data == 0x02) {
        clearBuf(lcd);
        cursor = 0;
      } else if (capture.data & 0x80) {
        cursor = capture.data & 0x7F;
      }
    } else {
      uint8_t row = 0;
      uint8_t col = 0;
      if (ddramToRowCol(cursor, row, col)) {
        const char ch = (capture.data >= 0x20 && capture.data <= 0x7E)
                            ? static_cast<char>(capture.data)
                            : ' ';
        lcd[row][col] = ch;
      }
      cursor++;
    }
  }

  if (lastBusMs != 0 && (millis() - lastBusMs) > 200) {
    lastBusMs = 0;
    if (lcdChanged()) {
      float rate_usvph = 0.0f;
      float avg_usvph = 0.0f;
      bool haveRate = false;
      bool haveAvg = false;
      parseRateAndAvg(rate_usvph, haveRate, avg_usvph, haveAvg);
#if defined(ARDUINO_AVR_MICRO)
      // The Micro path infers control/data from bytes, so only publish frames
      // that look complete and repeat twice in a row.
      const bool looksComplete = haveRate && haveAvg && lcd[0][0] == 'R' &&
                                 lcd[0][1] == ':' && lcd[1][0] == 'A' &&
                                 lcd[1][1] == ':' && lineHasToken(lcd[0], "usv/h") &&
                                 lineHasToken(lcd[1], "usv/h");
      if (!looksComplete) {
        microPendingValid = false;
      } else if (!microPendingValid || !sameBuf(microPendingLcd, lcd)) {
        copyBuf(microPendingLcd, lcd);
        microPendingValid = true;
      } else if (!sameBuf(lastLcd, lcd)) {
        printMetrics(rate_usvph, haveRate, avg_usvph, haveAvg);
        commitLcd();
        microPendingValid = false;
      }
#else
      printMetrics(rate_usvph, haveRate, avg_usvph, haveAvg);
      commitLcd();
#endif
    }
  }
}
