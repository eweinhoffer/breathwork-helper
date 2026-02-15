#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Missing secrets.h. Copy secrets.example.h to secrets.h and set credentials."
#endif

#ifndef WIFI_STA_SSID
#error "WIFI_STA_SSID must be defined in secrets.h"
#endif
#ifndef WIFI_STA_PASS
#error "WIFI_STA_PASS must be defined in secrets.h"
#endif
#ifndef OTA_HTTP_PASSWORD
#error "OTA_HTTP_PASSWORD must be defined in secrets.h"
#endif
#ifndef OTA_HTTP_USER
#define OTA_HTTP_USER "ota"
#endif

// Feather ESP32-S3 Reverse TFT variant defines these pins.
#ifndef TFT_CS
#define TFT_CS 42
#define TFT_RST 41
#define TFT_DC 40
#define TFT_BACKLITE 45
#define TFT_I2C_POWER 7
#endif

static const int BTN_START_PAUSE = 0;  // BOOT / D0, active LOW
static const int BTN_MODE = 1;         // D1, active HIGH
static const int BTN_RESET = 2;        // D2, active HIGH

static const uint16_t SCREEN_W = 240;
static const uint16_t SCREEN_H = 135;
static const uint16_t X_OFFSET = 40;  // ST7789 240x135 panel offsets
static const uint16_t Y_OFFSET = 53;

static const uint8_t MADCTL_MX = 0x40;
static const uint8_t MADCTL_MY = 0x80;
static const uint8_t MADCTL_MV = 0x20;
static const uint8_t MADCTL_RGB = 0x00;

static const uint16_t OTA_HTTP_PORT = 80;
static const uint32_t WIFI_RETRY_INTERVAL_MS = 180000;
static const uint32_t OTA_ARM_WINDOW_MS = 600000;
static const uint32_t BUTTON_LONG_PRESS_MS = 2200;

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)((r & 0xF8) << 8) | (uint16_t)((g & 0xFC) << 3) | (uint16_t)(b >> 3);
}

class ST7789Mini {
 public:
  void begin() {
    pinMode(TFT_CS, OUTPUT);
    pinMode(TFT_DC, OUTPUT);
    pinMode(TFT_RST, OUTPUT);
    pinMode(TFT_BACKLITE, OUTPUT);
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    digitalWrite(TFT_BACKLITE, HIGH);

    SPI.begin(SCK, MISO, MOSI, SS);
    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));

    digitalWrite(TFT_CS, HIGH);
    digitalWrite(TFT_DC, HIGH);

    hardReset();

    writeCommand(0x11);  // SLPOUT
    delay(120);

    writeCommandData(0x3A, 0x55);                         // COLMOD 16-bit
    writeCommandData(0x36, MADCTL_MX | MADCTL_MV | MADCTL_RGB);  // rotation 3 (landscape)
    writeCommand(0x21);                                   // INVON
    writeCommand(0x13);                                   // NORON
    delay(10);
    writeCommand(0x29);  // DISPON
    delay(100);
  }

  void fillScreen(uint16_t color) {
    fillRect(0, 0, SCREEN_W, SCREEN_H, color);
  }

  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if (x >= SCREEN_W || y >= SCREEN_H || w <= 0 || h <= 0) return;
    if (x < 0) {
      w += x;
      x = 0;
    }
    if (y < 0) {
      h += y;
      y = 0;
    }
    if (x + w > SCREEN_W) w = SCREEN_W - x;
    if (y + h > SCREEN_H) h = SCREEN_H - y;
    if (w <= 0 || h <= 0) return;

    setAddrWindow(x, y, w, h);
    uint32_t count = (uint32_t)w * (uint32_t)h;
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    digitalWrite(TFT_DC, HIGH);
    digitalWrite(TFT_CS, LOW);
    while (count--) {
      SPI.transfer(hi);
      SPI.transfer(lo);
    }
    digitalWrite(TFT_CS, HIGH);
  }

  void hLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
    fillRect(x, y, w, 1, color);
  }

 private:
  void hardReset() {
    digitalWrite(TFT_RST, HIGH);
    delay(5);
    digitalWrite(TFT_RST, LOW);
    delay(20);
    digitalWrite(TFT_RST, HIGH);
    delay(150);
  }

  void writeCommand(uint8_t cmd) {
    digitalWrite(TFT_DC, LOW);
    digitalWrite(TFT_CS, LOW);
    SPI.transfer(cmd);
    digitalWrite(TFT_CS, HIGH);
  }

  void writeData(uint8_t data) {
    digitalWrite(TFT_DC, HIGH);
    digitalWrite(TFT_CS, LOW);
    SPI.transfer(data);
    digitalWrite(TFT_CS, HIGH);
  }

  void writeCommandData(uint8_t cmd, uint8_t data) {
    writeCommand(cmd);
    writeData(data);
  }

  void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    uint16_t x0 = x + X_OFFSET;
    uint16_t x1 = x + w - 1 + X_OFFSET;
    uint16_t y0 = y + Y_OFFSET;
    uint16_t y1 = y + h - 1 + Y_OFFSET;

    writeCommand(0x2A);  // CASET
    writeData(x0 >> 8);
    writeData(x0 & 0xFF);
    writeData(x1 >> 8);
    writeData(x1 & 0xFF);

    writeCommand(0x2B);  // RASET
    writeData(y0 >> 8);
    writeData(y0 & 0xFF);
    writeData(y1 >> 8);
    writeData(y1 & 0xFF);

    writeCommand(0x2C);  // RAMWR
  }
};

ST7789Mini tft;
WebServer otaHttp(OTA_HTTP_PORT);

enum OtaState : uint8_t {
  OTA_READY = 0,
  OTA_UPDATING,
  OTA_ERROR,
};

static const uint8_t PHASE_INHALE = 0;
static const uint8_t PHASE_HOLD = 1;
static const uint8_t PHASE_EXHALE = 2;

struct Preset {
  const char* name;
  uint8_t inhaleSec;
  uint8_t holdSec;
  uint8_t exhaleSec;
};

Preset presets[] = {
    {"4-7-8", 4, 7, 8},
    {"BOX", 4, 4, 4},
    {"CALM", 5, 2, 5},
};

static uint8_t presetIndex = 0;
static uint8_t phase = PHASE_INHALE;
static bool running = true;
static uint32_t phaseStartMs = 0;
static uint32_t phaseDurationMs = 0;
static uint32_t lastDrawMs = 0;

static bool wifiConnected = false;
static IPAddress wifiIP(0, 0, 0, 0);
static uint32_t lastWiFiRetryMs = 0;

static OtaState otaState = OTA_READY;
static String otaLastError = "";
static bool otaWindowArmed = false;
static uint32_t otaWindowUntilMs = 0;

static uint8_t lastShownPhase = 255;
static uint8_t lastShownPreset = 255;
static uint8_t lastShownRemainSec = 255;
static int16_t lastBarFill = -1;
static bool lastShownRunning = false;
static String lastShownWiFi = "";
static String lastShownOTA = "";

static bool modeRawLast = false;
static bool modeStable = false;
static uint32_t modeDebounceMs = 0;
static uint32_t modePressedSinceMs = 0;
static bool modeLongFired = false;

static const int BAR_X = 112;
static const int BAR_Y = 106;
static const int BAR_W = 120;
static const int BAR_H = 16;
static const uint16_t BG_COLOR = rgb565(8, 12, 24);
static const uint16_t FG_COLOR = rgb565(245, 248, 255);
static const uint16_t HEADER_COLOR = rgb565(180, 200, 255);
static const uint16_t BAR_BG_COLOR = rgb565(35, 40, 58);
static const uint16_t BAR_BORDER_COLOR = rgb565(80, 90, 120);
static const uint16_t WIFI_COLOR = rgb565(160, 190, 255);
static const uint16_t OTA_COLOR_READY = rgb565(120, 230, 140);
static const uint16_t OTA_COLOR_UPDATING = rgb565(255, 210, 80);
static const uint16_t OTA_COLOR_ERROR = rgb565(255, 100, 100);

// 3x5 font for uppercase letters/digits and symbols used in UI.
struct Glyph3x5 {
  char c;
  uint8_t col[3];  // bit0 is top row
};

static const Glyph3x5 FONT[] = {
    {' ', {0x00, 0x00, 0x00}}, {'-', {0x04, 0x04, 0x04}}, {':', {0x00, 0x0A, 0x00}},
    {'.', {0x10, 0x00, 0x00}}, {'/', {0x18, 0x04, 0x03}},
    {'0', {0x1F, 0x11, 0x1F}}, {'1', {0x00, 0x1F, 0x00}}, {'2', {0x1D, 0x15, 0x17}},
    {'3', {0x15, 0x15, 0x1F}}, {'4', {0x07, 0x04, 0x1F}}, {'5', {0x17, 0x15, 0x1D}},
    {'6', {0x1F, 0x15, 0x1D}}, {'7', {0x01, 0x01, 0x1F}}, {'8', {0x1F, 0x15, 0x1F}},
    {'9', {0x17, 0x15, 0x1F}}, {'A', {0x1E, 0x05, 0x1E}}, {'B', {0x1F, 0x15, 0x0A}},
    {'C', {0x1F, 0x11, 0x11}}, {'D', {0x1F, 0x11, 0x0E}}, {'E', {0x1F, 0x15, 0x11}},
    {'F', {0x1F, 0x05, 0x01}}, {'G', {0x1F, 0x11, 0x1D}}, {'H', {0x1F, 0x04, 0x1F}},
    {'I', {0x11, 0x1F, 0x11}}, {'J', {0x08, 0x10, 0x0F}}, {'K', {0x1F, 0x04, 0x1B}},
    {'L', {0x1F, 0x10, 0x10}}, {'M', {0x1F, 0x06, 0x1F}}, {'N', {0x1F, 0x0E, 0x1F}},
    {'O', {0x1F, 0x11, 0x1F}}, {'P', {0x1F, 0x05, 0x07}}, {'Q', {0x1F, 0x19, 0x1F}},
    {'R', {0x1F, 0x0D, 0x16}}, {'S', {0x17, 0x15, 0x1D}}, {'T', {0x01, 0x1F, 0x01}},
    {'U', {0x1F, 0x10, 0x1F}}, {'V', {0x0F, 0x10, 0x0F}}, {'W', {0x1F, 0x0C, 0x1F}},
    {'X', {0x1B, 0x04, 0x1B}}, {'Y', {0x03, 0x1C, 0x03}}, {'Z', {0x19, 0x15, 0x13}},
};

void glyphFor(char c, uint8_t outCols[3]) {
  if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';
  for (size_t i = 0; i < sizeof(FONT) / sizeof(FONT[0]); i++) {
    if (FONT[i].c == c) {
      outCols[0] = FONT[i].col[0];
      outCols[1] = FONT[i].col[1];
      outCols[2] = FONT[i].col[2];
      return;
    }
  }
  outCols[0] = 0;
  outCols[1] = 0;
  outCols[2] = 0;
}

void drawChar3x5(int16_t x, int16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale) {
  uint8_t cols[3] = {0, 0, 0};
  glyphFor(c, cols);
  for (uint8_t col = 0; col < 3; col++) {
    for (uint8_t row = 0; row < 5; row++) {
      bool on = (cols[col] >> row) & 0x01;
      tft.fillRect(x + (int16_t)col * scale, y + (int16_t)row * scale, scale, scale, on ? fg : bg);
    }
  }
}

void drawText3x5(int16_t x, int16_t y, const String& s, uint16_t fg, uint16_t bg, uint8_t scale) {
  for (size_t i = 0; i < s.length(); i++) {
    drawChar3x5(x + (int16_t)i * (scale * 4), y, s[i], fg, bg, scale);
  }
}

uint8_t phaseSeconds(uint8_t p) {
  Preset& pr = presets[presetIndex];
  if (p == PHASE_INHALE) return pr.inhaleSec;
  if (p == PHASE_HOLD) return pr.holdSec;
  return pr.exhaleSec;
}

const char* phaseLabel(uint8_t p) {
  if (p == PHASE_INHALE) return "INHALE";
  if (p == PHASE_HOLD) return "HOLD";
  return "EXHALE";
}

uint16_t phaseColor(uint8_t p) {
  if (p == PHASE_INHALE) return rgb565(70, 180, 255);
  if (p == PHASE_HOLD) return rgb565(255, 180, 50);
  return rgb565(120, 230, 140);
}

bool otaWindowActive() {
  if (!otaWindowArmed) return false;
  return ((int32_t)(otaWindowUntilMs - millis()) > 0);
}

void armOtaWindow() {
  otaWindowArmed = true;
  otaWindowUntilMs = millis() + OTA_ARM_WINDOW_MS;
  otaState = OTA_READY;
  otaLastError = "";
  lastShownOTA = "";
}

void startPhase(uint8_t p) {
  phase = p;
  phaseStartMs = millis();
  phaseDurationMs = (uint32_t)phaseSeconds(phase) * 1000UL;
}

void nextPhase() {
  if (phase == PHASE_INHALE) startPhase(PHASE_HOLD);
  else if (phase == PHASE_HOLD) startPhase(PHASE_EXHALE);
  else startPhase(PHASE_INHALE);
}

void resetCycle() {
  startPhase(PHASE_INHALE);
}

void clearArea(int16_t x, int16_t y, int16_t w, int16_t h) {
  tft.fillRect(x, y, w, h, BG_COLOR);
}

void drawFrame() {
  tft.fillScreen(BG_COLOR);
  drawText3x5(4, 126, "D0 START D1 MODE/HOLD D2 RESET", rgb565(130, 140, 170), BG_COLOR, 1);
  tft.hLine(BAR_X, BAR_Y - 1, BAR_W, BAR_BORDER_COLOR);
  tft.hLine(BAR_X, BAR_Y + BAR_H, BAR_W, BAR_BORDER_COLOR);
}

void drawModeAndStatus(bool force) {
  if (!force && presetIndex == lastShownPreset && running == lastShownRunning) return;
  clearArea(8, 8, 230, 46);
  drawText3x5(8, 8, String("MODE ") + presets[presetIndex].name, HEADER_COLOR, BG_COLOR, 3);
  drawText3x5(8, 38, running ? "RUNNING" : "PAUSED",
              running ? rgb565(150, 255, 170) : rgb565(255, 120, 120), BG_COLOR, 2);
  lastShownPreset = presetIndex;
  lastShownRunning = running;
}

void drawPhaseLabel(bool force) {
  if (!force && phase == lastShownPhase) return;
  uint16_t accent = phaseColor(phase);
  clearArea(8, 62, 200, 28);
  drawText3x5(8, 62, String(phaseLabel(phase)), accent, BG_COLOR, 5);
  lastShownPhase = phase;
  lastBarFill = -1;  // Force bar recolor when phase color changes.
}

void drawTimeAndBar(bool force) {
  uint16_t accent = phaseColor(phase);
  uint32_t now = millis();
  uint32_t elapsed = now - phaseStartMs;
  uint32_t remainMs = (elapsed < phaseDurationMs) ? (phaseDurationMs - elapsed) : 0;
  uint8_t remainSec = (uint8_t)((remainMs + 999) / 1000);
  float pct = phaseDurationMs ? (float)elapsed / (float)phaseDurationMs : 1.0f;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 1.0f) pct = 1.0f;
  int fillW = (int)(pct * BAR_W);

  if (force || remainSec != lastShownRemainSec) {
    clearArea(8, 102, 96, 20);
    drawText3x5(8, 102, String("TIME ") + String(remainSec), FG_COLOR, BG_COLOR, 3);
    lastShownRemainSec = remainSec;
  }

  if (force || fillW != lastBarFill) {
    if (force || lastBarFill < 0 || fillW < lastBarFill) {
      tft.fillRect(BAR_X, BAR_Y, BAR_W, BAR_H, BAR_BG_COLOR);
      if (fillW > 0) tft.fillRect(BAR_X, BAR_Y, fillW, BAR_H, accent);
    } else {
      tft.fillRect(BAR_X + lastBarFill, BAR_Y, fillW - lastBarFill, BAR_H, accent);
    }
    lastBarFill = fillW;
  }
}

String wifiLabel() {
  if (!wifiConnected) return String("WIFI RETRYING ") + String(WIFI_STA_SSID);
  return String("WIFI ") + wifiIP.toString();
}

String otaLabel() {
  if (!wifiConnected) return String("OTA WAIT WIFI");
  if (otaState == OTA_ERROR) {
    if (otaLastError.length() > 0) return String("OTA ERROR ") + otaLastError;
    return String("OTA ERROR");
  }
  if (otaState == OTA_UPDATING) return String("OTA UPDATING");
  if (!otaWindowActive()) return String("OTA LOCKED HOLD D1");
  int32_t remainMs = (int32_t)(otaWindowUntilMs - millis());
  uint32_t remainSec = (uint32_t)((remainMs + 999) / 1000);
  return String("OTA READY ") + String(remainSec) + String("S");
}

uint16_t otaStatusColor() {
  if (otaState == OTA_ERROR) return OTA_COLOR_ERROR;
  if (otaState == OTA_UPDATING) return OTA_COLOR_UPDATING;
  if (otaWindowActive()) return OTA_COLOR_READY;
  return rgb565(180, 180, 180);
}

void drawNetStatus(bool force) {
  String wifiNow = wifiLabel();
  if (force || wifiNow != lastShownWiFi) {
    clearArea(8, 90, 232, 10);
    drawText3x5(8, 90, wifiNow, WIFI_COLOR, BG_COLOR, 1);
    lastShownWiFi = wifiNow;
  }

  String otaNow = otaLabel();
  if (force || otaNow != lastShownOTA) {
    clearArea(8, 116, 232, 10);
    drawText3x5(8, 116, otaNow, otaStatusColor(), BG_COLOR, 1);
    lastShownOTA = otaNow;
  }
}

void drawUI(bool force) {
  if (force) drawFrame();
  drawModeAndStatus(force);
  drawPhaseLabel(force);
  drawTimeAndBar(force);
  drawNetStatus(force);
}

bool edgePressed(uint8_t pin, bool activeHigh, bool& prev) {
  bool current = activeHigh ? (digitalRead(pin) == HIGH) : (digitalRead(pin) == LOW);
  bool edge = current && !prev;
  prev = current;
  return edge;
}

void refreshWiFiState() {
  bool oldConnected = wifiConnected;
  IPAddress oldIP = wifiIP;

  wifiConnected = (WiFi.status() == WL_CONNECTED);
  wifiIP = wifiConnected ? WiFi.localIP() : IPAddress(0, 0, 0, 0);

  if (!wifiConnected && (millis() - lastWiFiRetryMs >= WIFI_RETRY_INTERVAL_MS)) {
    WiFi.disconnect();
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    lastWiFiRetryMs = millis();
  }

  if (oldConnected != wifiConnected || oldIP != wifiIP) {
    lastShownWiFi = "";
    lastShownOTA = "";
  }
}

bool httpAuthOk() {
  if (!otaHttp.authenticate(OTA_HTTP_USER, OTA_HTTP_PASSWORD)) {
    otaHttp.requestAuthentication();
    return false;
  }
  return true;
}

void setupHttpOTA() {
  otaHttp.on("/", HTTP_GET, []() {
    if (!httpAuthOk()) return;
    String msg = String("HTTP OTA ") + (otaWindowActive() ? String("ready") : String("locked")) +
                 String("\nPOST firmware binary to /update\n");
    otaHttp.send(200, "text/plain", msg);
  });

  otaHttp.on("/update", HTTP_POST,
             []() {
               if (!httpAuthOk()) return;
               if (!otaWindowActive()) {
                 otaState = OTA_ERROR;
                 otaLastError = "LOCKED";
                 otaHttp.send(403, "text/plain", "OTA LOCKED\nHold D1 for 2.2s.\n");
                 lastShownOTA = "";
                 return;
               }

               if (Update.hasError()) {
                 otaState = OTA_ERROR;
                 if (otaLastError.length() == 0) otaLastError = "WRITE";
                 otaHttp.send(500, "text/plain", "OTA FAIL\n");
               } else {
                 otaState = OTA_READY;
                 otaLastError = "";
                 otaHttp.send(200, "text/plain", "OTA OK, rebooting\n");
                 delay(150);
                 ESP.restart();
               }
               lastShownOTA = "";
             },
             []() {
               HTTPUpload& up = otaHttp.upload();
               if (up.status == UPLOAD_FILE_START) {
                 if (!httpAuthOk()) return;
                 if (!otaWindowActive()) {
                   otaState = OTA_ERROR;
                   otaLastError = "LOCKED";
                   lastShownOTA = "";
                   return;
                 }
                 otaState = OTA_UPDATING;
                 otaLastError = "";
                 lastShownOTA = "";
                 if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                   otaState = OTA_ERROR;
                   otaLastError = "BEGIN";
                   Update.printError(Serial);
                   lastShownOTA = "";
                 }
               } else if (up.status == UPLOAD_FILE_WRITE) {
                 if (Update.write(up.buf, up.currentSize) != up.currentSize) {
                   otaState = OTA_ERROR;
                   otaLastError = "WRITE";
                   Update.printError(Serial);
                   lastShownOTA = "";
                 }
               } else if (up.status == UPLOAD_FILE_END) {
                 if (!Update.end(true)) {
                   otaState = OTA_ERROR;
                   otaLastError = "END";
                   Update.printError(Serial);
                   lastShownOTA = "";
                 }
               } else if (up.status == UPLOAD_FILE_ABORTED) {
                 Update.abort();
                 otaState = OTA_ERROR;
                 otaLastError = "ABORT";
                 lastShownOTA = "";
               }
             });

  otaHttp.begin();
}

void setupWiFiAndOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
  lastWiFiRetryMs = millis();

  refreshWiFiState();
  setupHttpOTA();
}

void handleModeButton() {
  bool raw = (digitalRead(BTN_MODE) == HIGH);

  if (raw != modeRawLast) {
    modeDebounceMs = millis();
    modeRawLast = raw;
  }

  if (millis() - modeDebounceMs <= 25) return;

  if (modeStable != raw) {
    modeStable = raw;
    if (modeStable) {
      modePressedSinceMs = millis();
      modeLongFired = false;
    } else {
      if (!modeLongFired) {
        presetIndex = (presetIndex + 1) % (sizeof(presets) / sizeof(presets[0]));
        resetCycle();
        drawUI(true);
      }
      modePressedSinceMs = 0;
      modeLongFired = false;
    }
  }

  if (modeStable && !modeLongFired && modePressedSinceMs > 0 &&
      (millis() - modePressedSinceMs >= BUTTON_LONG_PRESS_MS)) {
    modeLongFired = true;
    armOtaWindow();
    drawUI(true);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(BTN_START_PAUSE, INPUT_PULLUP);   // D0 button / BOOT
  pinMode(BTN_MODE, INPUT_PULLDOWN);        // D1 on-board button
  pinMode(BTN_RESET, INPUT_PULLDOWN);       // D2 on-board button

  tft.begin();
  setupWiFiAndOTA();
  startPhase(PHASE_INHALE);
  drawUI(true);
}

void loop() {
  static bool prevStart = false;
  static bool prevReset = false;

  otaHttp.handleClient();
  refreshWiFiState();

  if (otaWindowArmed && !otaWindowActive()) {
    otaWindowArmed = false;
    lastShownOTA = "";
  }

  handleModeButton();

  if (edgePressed(BTN_START_PAUSE, false, prevStart)) {
    running = !running;
    if (running) {
      phaseStartMs = millis();  // restart current phase timing when resuming
    }
    drawUI(true);
  }
  if (edgePressed(BTN_RESET, true, prevReset)) {
    resetCycle();
    drawUI(true);
  }

  if (running) {
    uint32_t now = millis();
    if (now - phaseStartMs >= phaseDurationMs) {
      nextPhase();
      drawUI(true);
    } else if (now - lastDrawMs >= 33) {
      lastDrawMs = now;
      drawUI(false);
    }
  } else if (millis() - lastDrawMs >= 250) {
    lastDrawMs = millis();
    drawNetStatus(false);
  }

  delay(5);
}
