#include <Arduino.h>

// ===== Pin mapping (Uno) =====
static const uint8_t PIN_E  = 2;   // interrupt on Uno
static const uint8_t PIN_RS = 4;
static const uint8_t PIN_RW = 5;

// Data bus D0..D7 from Geiger LCD header
static const uint8_t PIN_D0 = A0;
static const uint8_t PIN_D1 = A1;
static const uint8_t PIN_D2 = A2;
static const uint8_t PIN_D3 = A3;
static const uint8_t PIN_D4 = 6;
static const uint8_t PIN_D5 = 7;
static const uint8_t PIN_D6 = 8;
static const uint8_t PIN_D7 = 9;

// ===== Alarm thresholds (edit these) =====
// Thresholds are in uSv per hour unless noted.
static const float THRESH_FOUND_DELTA_USVPH   = 0.20f;  // above baseline by this amount triggers "source found"
static const uint16_t FOUND_HOLD_MS           = 3000;   // must persist this long

static const float THRESH_FLIGHT_LOW_USVPH    = 1.00f;  // elevated band lower bound
static const float THRESH_FLIGHT_HIGH_USVPH   = 10.0f;  // elevated band upper bound
static const uint16_t FLIGHT_HOLD_MS          = 15000;  // must persist this long

static const float THRESH_DANGER_USVPH        = 10.0f;  // danger threshold
static const float THRESH_UNHEALTHY_USVPH     = 100.0f; // unhealthy threshold

// ===== EMA time constants =====
static const float TAU_EMA_1M_S  = 60.0f;
static const float TAU_EMA_10M_S = 600.0f;

// ===== Ring buffer =====
static const uint16_t RB_SIZE = 256;
volatile uint8_t  rbMeta[RB_SIZE]; // bit7=RS bit6=RW
volatile uint8_t  rbData[RB_SIZE];
volatile uint16_t rbHead = 0, rbTail = 0;
volatile uint32_t hits = 0, overflows = 0;

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
  hits++;

  uint16_t nextHead = rbHead + 1;
  if (nextHead >= RB_SIZE) nextHead = 0;
  if (nextHead == rbTail) { overflows++; return; }

  uint8_t rs = (uint8_t)digitalRead(PIN_RS) & 1;
  uint8_t rw = (uint8_t)digitalRead(PIN_RW) & 1;
  uint8_t d  = readByteBus();

  rbMeta[rbHead] = (uint8_t)((rs << 7) | (rw << 6));
  rbData[rbHead] = d;
  rbHead = nextHead;
}

// ===== Virtual 16x2 LCD =====
static char lcd[2][16];
static char lastLcd[2][16];
static uint8_t cursor = 0;

static void clearBuf(char buf[2][16]) {
  for (uint8_t r = 0; r < 2; r++) {
    for (uint8_t c = 0; c < 16; c++) buf[r][c] = ' ';
  }
}

static inline bool ddramToRowCol(uint8_t addr, uint8_t &row, uint8_t &col) {
  if (addr < 0x10) { row = 0; col = addr; return true; }
  if (addr >= 0x40 && addr < 0x50) { row = 1; col = (uint8_t)(addr - 0x40); return true; }
  return false;
}

static bool lcdChanged() {
  for (uint8_t r = 0; r < 2; r++) {
    for (uint8_t c = 0; c < 16; c++) {
      if (lcd[r][c] != lastLcd[r][c]) return true;
    }
  }
  return false;
}

static void commitLcd() {
  for (uint8_t r = 0; r < 2; r++) {
    for (uint8_t c = 0; c < 16; c++) lastLcd[r][c] = lcd[r][c];
  }
}

static void copyLine(char *dst17, const char src[16]) {
  for (uint8_t i = 0; i < 16; i++) dst17[i] = src[i];
  dst17[16] = '\0';
}

static inline bool isDigitChar(char c) { return c >= '0' && c <= '9'; }

// Parse specific prefix value like "R:00.16 ..." or "A:00.18 ..."
static bool parseValueForPrefix(const char line[16], char wantPrefix, float &out) {
  int p = -1;
  for (uint8_t i = 0; i < 16; i++) {
    if (line[i] == wantPrefix && (i + 1 < 16) && line[i + 1] == ':') { p = i; break; }
  }
  if (p < 0) return false;

  int i = p + 2; // after ':'
  while (i < 16 && line[i] == ' ') i++;
  if (i >= 16) return false;

  long intPart = 0;
  int digits = 0;
  while (i < 16 && isDigitChar(line[i]) && digits < 4) {
    intPart = (intPart * 10L) + (line[i] - '0');
    i++;
    digits++;
  }
  if (digits < 1) return false;

  if (i >= 16 || line[i] != '.') return false;
  i++;

  if (i + 1 >= 16) return false;
  if (!isDigitChar(line[i]) || !isDigitChar(line[i + 1])) return false;

  int fracPart = (line[i] - '0') * 10 + (line[i + 1] - '0');
  out = (float)intPart + ((float)fracPart / 100.0f);
  return true;
}

// Some devices use S instead of R, treat S as rate fallback.
static bool parseRateAndAvg(float &rate_usvph, bool &haveRate, float &avg_usvph, bool &haveAvg) {
  haveRate = false;
  haveAvg  = false;

  float v = 0.0f;

  // Rate: prefer R, else S
  if (parseValueForPrefix(lcd[0], 'R', v) || parseValueForPrefix(lcd[1], 'R', v)) {
    rate_usvph = v;
    haveRate = true;
  } else if (parseValueForPrefix(lcd[0], 'S', v) || parseValueForPrefix(lcd[1], 'S', v)) {
    rate_usvph = v;
    haveRate = true;
  }

  // Average: A
  if (parseValueForPrefix(lcd[0], 'A', v) || parseValueForPrefix(lcd[1], 'A', v)) {
    avg_usvph = v;
    haveAvg = true;
  }

  return haveRate || haveAvg;
}

// ===== Derived metrics state =====
static float peak_usvph = 0.0f;
static float dose_uSv = 0.0f;

static bool emaInit = false;
static float ema1m_usvph  = 0.0f;
static float ema10m_usvph = 0.0f;

static uint32_t lastUpdateMs = 0;

// Alarm timers
static uint32_t foundStartMs  = 0;
static uint32_t flightStartMs = 0;

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void updateEma(float &ema, float x, float dt_s, float tau_s) {
  float alpha = dt_s / (tau_s + dt_s);
  alpha = clampf(alpha, 0.0f, 1.0f);
  ema += alpha * (x - ema);
}

// Convert uSv per hour to rad per hour and rad per second (approx)
// Assumption for demo: gamma or x ray like, quality factor near 1
static inline float usvph_to_radph(float usvph) {
  return usvph * 1e-6f * 100.0f; // uSv/h -> Sv/h -> Gy/h -> rad/h
}
static inline float usvph_to_rads(float usvph) {
  return usvph_to_radph(usvph) / 3600.0f;
}
static inline float uSv_to_rad(float uSv) {
  return uSv * 1e-6f * 100.0f;
}

static void printMetrics(float rate_usvph, bool haveRate, float avg_usvph, bool haveAvg) {
  char l1[17], l2[17];
  copyLine(l1, lcd[0]);
  copyLine(l2, lcd[1]);

  // Choose working value for EMA and dose
  float x_usvph = haveRate ? rate_usvph : (haveAvg ? avg_usvph : 0.0f);

  uint32_t nowMs = millis();
  float dt_s = 0.0f;
  if (lastUpdateMs != 0) dt_s = (float)(nowMs - lastUpdateMs) / 1000.0f;
  lastUpdateMs = nowMs;

  if (!emaInit) {
    ema1m_usvph  = x_usvph;
    ema10m_usvph = x_usvph;
    emaInit = true;
  } else if (dt_s > 0.0f && dt_s < 10.0f) {
    updateEma(ema1m_usvph,  x_usvph, dt_s, TAU_EMA_1M_S);
    updateEma(ema10m_usvph, x_usvph, dt_s, TAU_EMA_10M_S);
  }

  if (haveRate && rate_usvph > peak_usvph) peak_usvph = rate_usvph;

  if (dt_s > 0.0f && dt_s < 10.0f) {
    dose_uSv += x_usvph * (dt_s / 3600.0f);
  }

  float alarmVal = haveRate ? rate_usvph : x_usvph;

  bool alarm_unhealthy = (alarmVal >= THRESH_UNHEALTHY_USVPH);
  bool alarm_danger    = (alarmVal >= THRESH_DANGER_USVPH);

  bool inFlightBand = (alarmVal >= THRESH_FLIGHT_LOW_USVPH && alarmVal <= THRESH_FLIGHT_HIGH_USVPH);
  if (inFlightBand) {
    if (flightStartMs == 0) flightStartMs = nowMs;
  } else {
    flightStartMs = 0;
  }
  bool alarm_flight = (flightStartMs != 0 && (nowMs - flightStartMs) >= FLIGHT_HOLD_MS);

  bool aboveBaseline = (alarmVal >= (ema10m_usvph + THRESH_FOUND_DELTA_USVPH));
  if (aboveBaseline) {
    if (foundStartMs == 0) foundStartMs = nowMs;
  } else {
    foundStartMs = 0;
  }
  bool alarm_found = (foundStartMs != 0 && (nowMs - foundStartMs) >= FOUND_HOLD_MS);

  float rate_radph = usvph_to_radph(alarmVal);
  float rate_rads  = usvph_to_rads(alarmVal);
  float dose_rad   = uSv_to_rad(dose_uSv);

  Serial.print("LCD|"); Serial.print(l1); Serial.print("|"); Serial.print(l2); Serial.println("|");

  Serial.print("rate_usvph=");
  if (haveRate) Serial.print(rate_usvph, 2); else Serial.print("NA");

  Serial.print(" avg_usvph=");
  if (haveAvg) Serial.print(avg_usvph, 2); else Serial.print("NA");

  Serial.print(" ema1m_usvph=");  Serial.print(ema1m_usvph, 2);
  Serial.print(" ema10m_usvph="); Serial.print(ema10m_usvph, 2);

  Serial.print(" peak_usvph="); Serial.print(peak_usvph, 2);
  Serial.print(" dose_uSv=");   Serial.print(dose_uSv, 3);

  Serial.print(" rate_radph="); Serial.print(rate_radph, 8);
  Serial.print(" rate_rads=");  Serial.print(rate_rads, 10);
  Serial.print(" dose_rad=");   Serial.print(dose_rad, 8);

  Serial.print(" alarm=");
  bool any = false;
  if (alarm_unhealthy) { Serial.print("UNHEALTHY"); any = true; }
  if (alarm_danger)    { Serial.print(any ? "," : ""); Serial.print("DANGER"); any = true; }
  if (alarm_found)     { Serial.print(any ? "," : ""); Serial.print("FOUND"); any = true; }
  if (alarm_flight)    { Serial.print(any ? "," : ""); Serial.print("FLIGHT"); any = true; }
  if (!any) Serial.print("OK");
  Serial.println();
}

void setup() {
  Serial.begin(115200);

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
  cursor = 0;

  attachInterrupt(digitalPinToInterrupt(PIN_E), onERise, RISING);

  Serial.println("LCD sniffer metrics running (no LCD connected)");
}

void loop() {
  static uint32_t lastBusMs = 0;
  static uint32_t lastStatusMs = 0;

  while (rbTail != rbHead) {
    uint8_t meta = rbMeta[rbTail];
    uint8_t data = rbData[rbTail];

    noInterrupts();
    uint16_t nextTail = rbTail + 1;
    if (nextTail >= RB_SIZE) nextTail = 0;
    rbTail = nextTail;
    interrupts();

    uint8_t rs = (meta >> 7) & 1;
    uint8_t rw = (meta >> 6) & 1;
    if (rw) continue;

    lastBusMs = millis();

    if (rs == 0) {
      if (data == 0x01 || data == 0x02) {
        clearBuf(lcd);
        cursor = 0;
      } else if (data & 0x80) {
        cursor = data & 0x7F;
      }
    } else {
      uint8_t row, col;
      if (ddramToRowCol(cursor, row, col)) {
        char ch = (data >= 0x20 && data <= 0x7E) ? (char)data : ' ';
        lcd[row][col] = ch;
      }
      cursor++;
    }
  }

  if (lastBusMs && (millis() - lastBusMs) > 200) {
    lastBusMs = 0;
    if (lcdChanged()) {
      float rate_usvph = 0.0f, avg_usvph = 0.0f;
      bool haveRate = false, haveAvg = false;
      parseRateAndAvg(rate_usvph, haveRate, avg_usvph, haveAvg);
      printMetrics(rate_usvph, haveRate, avg_usvph, haveAvg);
      commitLcd();
    }
  }

  if (millis() - lastStatusMs > 3000) {
    lastStatusMs = millis();
    uint32_t h, ov;
    uint16_t head, tail;
    noInterrupts();
    h = hits; ov = overflows; head = rbHead; tail = rbTail;
    interrupts();
    uint16_t used = (head >= tail) ? (head - tail) : (RB_SIZE - (tail - head));
    Serial.print("status hits="); Serial.print(h);
    Serial.print(" ov="); Serial.print(ov);
    Serial.print(" used="); Serial.println(used);
  }
}
