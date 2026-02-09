//Repeater controller by SP3VSS
//Not for commercial use
//Copyright by VSS 2k25

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

// ==============================
// NVS / konfiguracja
// ==============================
Preferences prefs;

// Zmienne konfiguracyjne (w NVS)
String   wifiSsid;
String   wifiPass;
bool     wifiUseDhcp;

uint32_t timerDuration;
uint32_t signalStartTimeCfg;
int      toneFrequency;
uint32_t toneDelay;
uint32_t dotDuration;

bool     enableMorse;
bool     enableEdgeBeep;
bool     enablePreOffBeep;

// VOICE (DFPlayer)
bool     enableVoice;
uint32_t voiceInterval;   // NOWE: minimalny czas pomiędzy odtworzeniami jak MORSE_INTERVAL

uint32_t morseInterval;
uint32_t beepDelay;
uint32_t beepDuration;
uint32_t preOffBeepBeforeOff;
uint32_t preOffBeepDuration;
int      preOffBeepFrequency;

String   znamiennikStr;

// ==============================
// DFPlayer Mini (UART2)
// ==============================
static const int DF_TX_PIN = 17;   // ESP32 TX -> DFPlayer RX
static const int DF_RX_PIN = 16;   // ESP32 RX <- DFPlayer TX
static const uint32_t DF_BAUD = 9600; // default 9600 [page:0]
HardwareSerial DFSerial(2);

static const uint32_t DF_PLAY_DELAY_MS = 3000;

// sterowanie "po zboczu" GPIO32 + opóźnienie 3s
bool dfPlayArmed = false;
bool dfPlayDoneThisHigh = false;
uint32_t dfPlayTriggerAt = 0;

// NOWE: ograniczenie częstotliwości odtwarzania (jak MORSE_INTERVAL)
uint32_t lastVoicePlayTime = 0;

static inline uint16_t dfChecksum(uint8_t ver, uint8_t len, uint8_t cmd, uint8_t feedback, uint8_t para1, uint8_t para2) {
  uint16_t sum = ver + len + cmd + feedback + para1 + para2;
  return (uint16_t)(0 - sum);
}

static void dfSendCmd(uint8_t cmd, uint16_t param, bool feedback = false) {
  uint8_t start = 0x7E;
  uint8_t ver   = 0xFF;
  uint8_t len   = 0x06;
  uint8_t fb    = feedback ? 0x01 : 0x00;
  uint8_t p1    = (param >> 8) & 0xFF;
  uint8_t p2    = (param) & 0xFF;
  uint16_t cs   = dfChecksum(ver, len, cmd, fb, p1, p2);
  uint8_t csh   = (cs >> 8) & 0xFF;
  uint8_t csl   = (cs) & 0xFF;
  uint8_t end   = 0xEF;

  DFSerial.write(start);
  DFSerial.write(ver);
  DFSerial.write(len);
  DFSerial.write(cmd);
  DFSerial.write(fb);
  DFSerial.write(p1);
  DFSerial.write(p2);
  DFSerial.write(csh);
  DFSerial.write(csl);
  DFSerial.write(end);
}

// CMD 0x03: Specify tracking(NUM) [page:0]
static void dfPlayTrack1() {
  dfSendCmd(0x03, 0x0001, false);
}

// ==============================
// ADS1115 / I2C
// ==============================
static const uint8_t I2C_SDA = 21;
static const uint8_t I2C_SCL = 22;
static const uint8_t ADS1115_ADDR = 0x4A;

Adafruit_ADS1115 ads;
bool adsOk = false;

// ==============================
// Konwersja napięcie -> S / dBm
// ==============================
struct CalPoint { float v; float s; float dbm; };

static const CalPoint calTable[] = {
  {2.87f,  1.0f,  -121.0f},
  {3.10f,  2.0f,  -115.0f},
  {3.41f,  3.0f,  -109.0f},
  {3.67f,  4.0f,  -103.0f},
  {3.78f,  5.0f,   -97.0f},
  {3.85f,  6.0f,   -91.0f},
  {3.93f,  7.0f,   -85.0f},
  {4.06f,  8.0f,   -79.0f},
  {4.19f,  9.0f,   -73.0f},
  {4.56f, 10.0f,   -63.0f},
  {4.90f, 20.0f,   -53.0f}
};
static const int calCount = sizeof(calTable) / sizeof(calTable[0]);

bool  ch1FiltInit = false;
float ch1VoltFilt = 0.0f;
const float CH1_EMA_ALPHA = 0.15f;

float ch1Dbm = 0.0f;
float ch1S   = 0.0f;

static const float NO_SIGNAL_VOLT = 2.50f;

static inline float lerpF(float a, float b, float t) { return a + (b - a) * t; }
static inline float clampf(float x, float lo, float hi) { if (x < lo) return lo; if (x > hi) return hi; return x; }

static inline float truncTo(float x, int decimals) {
  float m = powf(10.0f, (float)decimals);
  return truncf(x * m) / m;
}
static inline String truncToStr(float x, int decimals) { return String(truncTo(x, decimals), decimals); }
static inline int truncToInt(float x) { return (int)truncf(x); }

void convertVtoDbmS(float v, float &outDbm, float &outS) {
  if (calCount < 2) { outDbm = 0; outS = 0; return; }
  if (v <= calTable[0].v) { outDbm = calTable[0].dbm; outS = calTable[0].s; return; }
  if (v >= calTable[calCount - 1].v) { outDbm = calTable[calCount - 1].dbm; outS = calTable[calCount - 1].s; return; }

  for (int i = 0; i < calCount - 1; i++) {
    float v0 = calTable[i].v;
    float v1 = calTable[i + 1].v;
    if (v >= v0 && v <= v1) {
      float t = (v - v0) / (v1 - v0);
      t = clampf(t, 0.0f, 1.0f);
      outDbm = lerpF(calTable[i].dbm, calTable[i + 1].dbm, t);
      outS   = lerpF(calTable[i].s,   calTable[i + 1].s,   t);
      return;
    }
  }
  outDbm = calTable[calCount - 1].dbm;
  outS = calTable[calCount - 1].s;
}

String formatSLevelText(float sInterpolated, float v) {
  if (v >= 4.90f) return "S9+20";
  if (v >= 4.56f) return "S9+10";
  int sInt = truncToInt(sInterpolated);
  if (sInt < 1) sInt = 1;
  if (sInt > 9) sInt = 9;
  return "S" + String(sInt);
}

// ==============================
// Zmienne robocze programu
// ==============================
#define PIN_INPUT_1 34
#define PIN_INPUT_2 35
#define PIN_OUTPUT  32
#define PIN_LED      4
#define PIN_BUZZER  23

#ifndef D2
#define D2 2
#endif
#define PIN_WIFI_LED D2

const int pwmChannel    = 0;
const int pwmResolution = 8;

unsigned long signalStartTimeMs    = 0;
unsigned long outputHighStartTime  = 0;
unsigned long ledLastToggleTime    = 0;

unsigned long morseStartTime       = 0;

unsigned long beepStartTime        = 0;
bool beepPending                   = false;
bool beepPlaying                   = false;

unsigned long preOffBeepStartTime   = 0;
unsigned long preOffBeepTriggerTime = 0;
bool preOffBeepPending              = false;
bool preOffBeepPlaying              = false;

bool inputActive                    = false;
bool outputHigh                     = false;
bool ledState                       = false;
bool timerRunning                   = false;

bool morsePlaying                   = false;
bool morsePending                   = false;
bool morseGeneratedThisCycle        = false;

unsigned long lastMorseStartTime    = 0;

bool wifiLedState = false;
unsigned long wifiLedLastToggle = 0;

unsigned long preOffFreqLastUpdate = 0;
const unsigned long PREOFF_FREQ_UPDATE_MS = 100;

// Morse
struct MorseCode { char symbol; const char* code; };
const MorseCode morseTable[] = {
  {'A', ".-"}, {'B', "-..."}, {'C', "-.-."}, {'D', "-.."}, {'E', "."},
  {'F', "..-."}, {'G', "--."}, {'H', "...."}, {'I', ".."}, {'J', ".---"},
  {'K', "-.-"}, {'L', ".-.."}, {'M', "--"}, {'N', "-."}, {'O', "---"},
  {'P', ".--."}, {'Q', "--.-"}, {'R', ".-."}, {'S', "..."}, {'T', "-"},
  {'U', "..-"}, {'V', "...-"}, {'W', ".--"}, {'X', "-..-"}, {'Y', "-.--"},
  {'Z', "--.."},
  {'0', "-----"}, {'1', ".----"}, {'2', "..---"}, {'3', "...--"}, {'4', "....-"},
  {'5', "....."}, {'6', "-...."}, {'7', "--..."}, {'8', "---.."}, {'9', "----."}
};

int morseIndex = 0;
int morseCharIndex = 0;
unsigned long morseElementStart = 0;

enum MorseState { IDLE, DOT, DASH, GAP_ELEMENT, GAP_LETTER };
MorseState morseState = IDLE;

uint32_t DOT_UNIT;
uint32_t DASH_UNIT;
uint32_t GAP_ELEMENT_UNIT;
uint32_t GAP_LETTER_UNIT;

// ==============================
// NVS: ładowanie / zapis
// ==============================
void loadConfig() {
  prefs.begin("repeater");

  wifiSsid        = prefs.getString("wifi_ssid",  "SSID");
  wifiPass        = prefs.getString("wifi_pass",  "password");
  wifiUseDhcp     = prefs.getBool  ("wifi_dhcp",  true);

  timerDuration      = prefs.getULong("tim_dur",     10000);
  signalStartTimeCfg = prefs.getULong("sig_strt",    1000);
  toneFrequency      = prefs.getInt  ("tone_frq",    1000);
  toneDelay          = prefs.getULong("tone_dly",    3000);
  dotDuration        = prefs.getULong("dot_dur",     66);

  enableMorse        = prefs.getBool("ena_morse",    true);
  enableEdgeBeep     = prefs.getBool("ena_ebp",      true);
  enablePreOffBeep   = prefs.getBool("ena_poff",     true);

  enableVoice        = prefs.getBool("ena_voice",    true);

  morseInterval      = prefs.getULong("morse_int",   60000);

  // NOWE: voiceInterval (minimalny czas)
  voiceInterval      = prefs.getULong("voice_int",   60000);

  beepDelay          = prefs.getULong("beep_dly",    1000);
  beepDuration       = prefs.getULong("beep_dur",    100);
  preOffBeepBeforeOff= prefs.getULong("poff_bfr",    1000);
  preOffBeepDuration = prefs.getULong("poff_dur",    300);
  preOffBeepFrequency= prefs.getInt  ("poff_frq",    1000);

  znamiennikStr      = prefs.getString("znamien",    "SR3X");

  prefs.end();

  DOT_UNIT         = dotDuration;
  DASH_UNIT        = DOT_UNIT * 3;
  GAP_ELEMENT_UNIT = dotDuration;
  GAP_LETTER_UNIT  = dotDuration * 3;
}

void saveConfig() {
  prefs.begin("repeater");

  prefs.putString("wifi_ssid",  wifiSsid);
  prefs.putString("wifi_pass",  wifiPass);
  prefs.putBool  ("wifi_dhcp",  wifiUseDhcp);

  prefs.putULong("tim_dur",     timerDuration);
  prefs.putULong("sig_strt",    signalStartTimeCfg);
  prefs.putInt  ("tone_frq",    toneFrequency);
  prefs.putULong("tone_dly",    toneDelay);
  prefs.putULong("dot_dur",     dotDuration);

  prefs.putBool ("ena_morse",   enableMorse);
  prefs.putBool ("ena_ebp",     enableEdgeBeep);
  prefs.putBool ("ena_poff",    enablePreOffBeep);

  prefs.putBool ("ena_voice",   enableVoice);

  prefs.putULong("morse_int",   morseInterval);

  // NOWE
  prefs.putULong("voice_int",   voiceInterval);

  prefs.putULong("beep_dly",    beepDelay);
  prefs.putULong("beep_dur",    beepDuration);
  prefs.putULong("poff_bfr",    preOffBeepBeforeOff);
  prefs.putULong("poff_dur",    preOffBeepDuration);
  prefs.putInt  ("poff_frq",    preOffBeepFrequency);

  prefs.putString("znamien",    znamiennikStr);

  prefs.end();

  DOT_UNIT         = dotDuration;
  DASH_UNIT        = DOT_UNIT * 3;
  GAP_ELEMENT_UNIT = dotDuration;
  GAP_LETTER_UNIT  = dotDuration * 3;
}

// ==============================
// BUZZER
// ==============================
void buzzerOn(int freq) {
  if (freq <= 0) return;
  ledcWriteTone(PIN_BUZZER, (uint32_t)freq);
}
void buzzerOff() { ledcWriteTone(PIN_BUZZER, 0); }

// ==============================
// ADS1115
// ==============================
static inline float adsRawToVolts(int16_t raw) { return (float)raw * 0.0001875f; }

bool readAdsVoltages(float v[4]) {
  if (!adsOk) return false;
  int16_t r0 = ads.readADC_SingleEnded(0);
  int16_t r1 = ads.readADC_SingleEnded(1);
  int16_t r2 = ads.readADC_SingleEnded(2);
  int16_t r3 = ads.readADC_SingleEnded(3);

  v[0] = adsRawToVolts(r0);
  v[1] = adsRawToVolts(r1);
  v[2] = adsRawToVolts(r2);
  v[3] = adsRawToVolts(r3) * 4.0f;
  return true;
}

// ==============================
// Wi-Fi / WWW
// ==============================
WebServer server(80);

static const char* PAGE_CSS = R"CSS(
body{font-family:Arial,Helvetica,sans-serif;background:#000;color:#fff;}
.container{display:flex;gap:20px;margin:20px;justify-content:center;}
.col{flex:1;background:#000;padding:10px;border:1px solid #666;max-width:900px;}
table{border-collapse:collapse;width:50%;margin:0 auto;}
th,td{border:1px solid #666;padding:4px;font-size:14px;text-align:left;color:#fff;}
th{background:#111;}
.high{color:#66ff66;font-weight:bold;}
.low{color:#ff6666;font-weight:bold;}
input[type=text]{width:100%;box-sizing:border-box;background:#111;color:#fff;border:1px solid #666;}
input[type=number]{width:100%;box-sizing:border-box;background:#111;color:#fff;border:1px solid #666;}
.btnrow{text-align:right;margin-top:10px;}
.btn{padding:6px 12px;margin-left:5px;background:#111;color:#fff;border:1px solid #666;}
.big{font-size:30px;font-weight:bold;text-align:center;}
.center{text-align:center;}
)CSS";

void setupWiFi() {
  WiFi.mode(WIFI_STA);

  if (!wifiUseDhcp) {
    IPAddress WIFI_LOCAL_IP(192, 168, 1, 80);
    IPAddress WIFI_GATEWAY (192, 168, 1, 1);
    IPAddress WIFI_SUBNET  (255, 255, 255, 0);
    WiFi.config(WIFI_LOCAL_IP, WIFI_GATEWAY, WIFI_SUBNET);
  }

  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  delay(100);
}

// ===============
// STRONA 1: STAN
// ===============
void handleStatePage() {
  String html;
  html.reserve(4200);

  html += F("<!DOCTYPE html><html><head><meta charset='UTF-8'>");
  html += F("<title>Stan przemiennika</title>");
  html += F("<style>");
  html += PAGE_CSS;
  html += F("</style>");

  html += F("<script>"
            "function updateStatus(){"
              "var xhttp=new XMLHttpRequest();"
              "xhttp.onreadystatechange=function(){"
                "if(this.readyState==4 && this.status==200){"
                  "document.getElementById('status').innerHTML=this.responseText;"
                "}"
              "};"
              "xhttp.open('GET','/status',true);"
              "xhttp.send();"
            "}"
            "setInterval(updateStatus,3000);"
            "window.onload=updateStatus;"
            "</script>");

  html += F("</head><body>");
  html += F("<h1 class='center'>Wizualizacja przemiennika</h1>");

  html += F("<p class='center'>Adres ESP32: ");
  if (WiFi.status() == WL_CONNECTED) html += WiFi.localIP().toString();
  else {
    html += F("Łączenie... (status=");
    html += WiFi.status();
    html += F(")");
  }
  html += F("</p>");

  html += F("<div class='container'>");
  html += F("<div class='col'><h2>Stan przemiennika</h2>");
  html += F("<div id='status'>Ładowanie...</div>");
  html += F("<div class='btnrow'>");
  html += F("<button type='button' class='btn' onclick=\"location.href='/settings'\">USTAWIENIA</button>");
  html += F("</div>");
  html += F("</div>");
  html += F("</div></body></html>");

  server.send(200, "text/html", html);
}

void handleStatus() {
  int carrierState  = digitalRead(PIN_INPUT_1);
  int tone1750State = digitalRead(PIN_INPUT_2);
  int pttState      = digitalRead(PIN_OUTPUT);

  String carrierTxt = (carrierState == HIGH) ? "BRAK SYGNAŁU" : "SYGNAŁ";
  String toneTxt    = (tone1750State == HIGH) ? "BRAK SYGNAŁU" : "SYGNAŁ";
  String pttTxt     = (pttState == HIGH) ? "ZAŁĄCZONY" : "WYŁĄCZONY";

  String carrierSpan = (carrierState == HIGH) ? "<span class='low'>" + carrierTxt + "</span>"
                                              : "<span class='high'>" + carrierTxt + "</span>";

  String toneSpan    = (tone1750State == HIGH) ? "<span class='low'>" + toneTxt + "</span>"
                                               : "<span class='high'>" + toneTxt + "</span>";

  String pttSpan     = (pttState == HIGH) ? "<span class='high'>" + pttTxt + "</span>"
                                         : "<span class='low'>" + pttTxt + "</span>";

  // NOWE: statusy enableMorse/enableVoice
  String morseTxt = enableMorse ? "WŁĄCZONY" : "WYŁĄCZONY";
  String morseSpan = enableMorse ? "<span class='high'>" + morseTxt + "</span>"
                                 : "<span class='low'>"  + morseTxt + "</span>";

  String voiceTxt = enableVoice ? "WŁĄCZONY" : "WYŁĄCZONY";
  String voiceSpan = enableVoice ? "<span class='high'>" + voiceTxt + "</span>"
                                 : "<span class='low'>"  + voiceTxt + "</span>";

  float v[4] = {0,0,0,0};
  bool okAdc = readAdsVoltages(v);

  float vinRaw = 0.0f;
  float dbmFromRaw = 0.0f;
  float sFromRaw   = 0.0f;
  float dbmFromFilt = 0.0f;
  float sFromFilt   = 0.0f;
  bool noSignal = false;

  if (okAdc) {
    vinRaw = v[0];
    if (vinRaw < NO_SIGNAL_VOLT) {
      noSignal = true;
    } else {
      convertVtoDbmS(vinRaw, dbmFromRaw, sFromRaw);

      if (!ch1FiltInit) {
        ch1VoltFilt = vinRaw;
        ch1FiltInit = true;
      } else {
        ch1VoltFilt = ch1VoltFilt + CH1_EMA_ALPHA * (vinRaw - ch1VoltFilt);
      }
      convertVtoDbmS(ch1VoltFilt, dbmFromFilt, sFromFilt);

      ch1Dbm = dbmFromFilt;
      ch1S   = sFromFilt;
    }
  }

  String sText = "---";
  String dbmText = "---";

  if (okAdc) {
    if (noSignal) {
      sText = "no signal";
      dbmText = "no signal";
    } else if (ch1FiltInit) {
      sText = formatSLevelText(sFromRaw, vinRaw);
      dbmText = String(truncToInt(dbmFromFilt));
    }
  }

  String s;
  s.reserve(4200);

  // TABELA 1: stany GPIO + nowe linie
  s += F("<table>");
  s += F("<tr><th>Sygnal</th><th>GPIO</th><th>Stan</th></tr>");

  s += F("<tr><td>Nośna</td><td>GPIO34</td><td>");
  s += carrierSpan;
  s += F("</td></tr>");

  s += F("<tr><td>Ton 1750Hz</td><td>GPIO35</td><td>");
  s += toneSpan;
  s += F("</td></tr>");

  s += F("<tr><td>PTT</td><td>GPIO32</td><td>");
  s += pttSpan;
  s += F("</td></tr>");

  // NOWE: znamiennik enable/disable
  s += F("<tr><td>Znamiennik</td><td>-</td><td>");
  s += morseSpan;
  s += F("</td></tr>");

  // NOWE: voice enable/disable
  s += F("<tr><td>VOICE (DFPlayer)</td><td>-</td><td>");
  s += voiceSpan;
  s += F("</td></tr>");

  s += F("</table>");

  // (reszta strony bez zmian)
  s += F("<h3>ADS1115 (0x4A) - napiecia</h3>");
  if (!adsOk) {
    s += F("<p><span class='low'>Blad inicjalizacji ADS1115</span></p>");
  } else {
    s += F("<table>");
    s += F("<tr><th>Wejscie</th><th>Napiecie [V]</th></tr>");

    if (!okAdc) {
      s += F("<tr><td>1</td><td><span class='low'>brak odczytu</span></td></tr>");
      s += F("<tr><td>2</td><td><span class='low'>brak odczytu</span></td></tr>");
      s += F("<tr><td>3</td><td><span class='low'>brak odczytu</span></td></tr>");
      s += F("<tr><td>4</td><td><span class='low'>brak odczytu</span></td></tr>");
    } else {
      s += "<tr><td>1</td><td>" + truncToStr(v[0], 2) + "</td></tr>";
      s += "<tr><td>2</td><td>" + truncToStr(v[1], 2) + "</td></tr>";
      s += "<tr><td>3</td><td>" + truncToStr(v[2], 2) + "</td></tr>";
      s += "<tr><td>4</td><td>" + truncToStr(v[3], 2) + "</td></tr>";
    }

    s += F("</table>");
    s += F("<p>Zakres: &plusmn;6.144 V</p>");
  }

  s += F("<h3>Poziom sygnalu (wejscie 1)</h3>");
  s += F("<table>");
  s += F("<tr><th>Poziom -dBm</th><th>S-meter</th></tr>");

  if (!okAdc) {
    s += F("<tr><td class='big'><span class='low'>---</span></td><td class='big'><span class='low'>---</span></td></tr>");
  } else if (noSignal) {
    s += "<tr><td class='big'><span class='low'>no signal</span></td><td class='big'><span class='low'>no signal</span></td></tr>";
  } else if (!ch1FiltInit) {
    s += F("<tr><td class='big'><span class='low'>---</span></td><td class='big'><span class='low'>---</span></td></tr>");
  } else {
    s += "<tr><td class='big'>" + dbmText + "</td><td class='big'>" + sText + "</td></tr>";
  }

  s += F("</table>");

  server.send(200, "text/html", s);
}

// ===================
// STRONA 2: USTAWIENIA
// ===================
void handleSettingsPage() {
  String html;
  html.reserve(7500);

  html += F("<!DOCTYPE html><html><head><meta charset='UTF-8'>");
  html += F("<title>Ustawienia</title>");
  html += F("<style>");
  html += PAGE_CSS;
  html += F("</style>");
  html += F("</head><body>");

  html += F("<h1 class='center'>Wizualizacja przemiennika</h1>");

  html += F("<p class='center'>Adres ESP32: ");
  if (WiFi.status() == WL_CONNECTED) html += WiFi.localIP().toString();
  else {
    html += F("Łączenie... (status=");
    html += WiFi.status();
    html += F(")");
  }
  html += F("</p>");

  html += F("<div class='container'>");
  html += F("<div class='col'><h2>Konfiguracja (NVS)</h2>");
  html += F("<form method='GET' action='/write'>");
  html += F("<table>");
  html += F("<tr><th>Parametr</th><th>Wartość</th></tr>");

  html += F("<tr><td>WIFI_SSID</td><td><input type='text' name='wifiSsid' value='");
  html += wifiSsid;
  html += F("'></td></tr>");

  html += F("<tr><td>WIFI_PASSWORD</td><td><input type='text' name='wifiPass' value='");
  html += wifiPass;
  html += F("'></td></tr>");

  html += F("<tr><td>WIFI_USE_DHCP (0/1)</td><td><input type='number' name='wifiDhcp' min='0' max='1' value='");
  html += wifiUseDhcp ? "1" : "0";
  html += F("'></td></tr>");

  html += F("<tr><td>ENABLE_VOICE (0/1)</td><td><input type='number' name='enableVoice' min='0' max='1' value='");
  html += enableVoice ? "1" : "0";
  html += F("'></td></tr>");

  // NOWE: VOICE_INTERVAL jak MORSE_INTERVAL
  html += F("<tr><td>VOICE_INTERVAL [ms]</td><td><input type='number' name='voiceInterval' value='");
  html += String(voiceInterval);
  html += F("'></td></tr>");

  html += F("<tr><td>TIMER_DURATION [ms]</td><td><input type='number' name='timerDuration' value='");
  html += String(timerDuration);
  html += F("'></td></tr>");

  html += F("<tr><td>SIGNAL_START_TIME [ms]</td><td><input type='number' name='signalStartTime' value='");
  html += String(signalStartTimeCfg);
  html += F("'></td></tr>");

  html += F("<tr><td>TONE_FREQUENCY [Hz]</td><td><input type='number' name='toneFrequency' value='");
  html += String(toneFrequency);
  html += F("'></td></tr>");

  html += F("<tr><td>TONE_DELAY [ms]</td><td><input type='number' name='toneDelay' value='");
  html += String(toneDelay);
  html += F("'></td></tr>");

  html += F("<tr><td>DOT_DURATION [ms]</td><td><input type='number' name='dotDuration' value='");
  html += String(dotDuration);
  html += F("'></td></tr>");

  html += F("<tr><td>ENABLE_MORSE (0/1)</td><td><input type='number' name='enableMorse' min='0' max='1' value='");
  html += enableMorse ? "1" : "0";
  html += F("'></td></tr>");

  html += F("<tr><td>ENABLE_EDGE_BEEP (0/1)</td><td><input type='number' name='enableEdgeBeep' min='0' max='1' value='");
  html += enableEdgeBeep ? "1" : "0";
  html += F("'></td></tr>");

  html += F("<tr><td>ENABLE_PRE_OFF_BEEP (0/1)</td><td><input type='number' name='enablePreOffBeep' min='0' max='1' value='");
  html += enablePreOffBeep ? "1" : "0";
  html += F("'></td></tr>");

  html += F("<tr><td>MORSE_INTERVAL [ms]</td><td><input type='number' name='morseInterval' value='");
  html += String(morseInterval);
  html += F("'></td></tr>");

  html += F("<tr><td>BEEP_DELAY [ms]</td><td><input type='number' name='beepDelay' value='");
  html += String(beepDelay);
  html += F("'></td></tr>");

  html += F("<tr><td>BEEP_DURATION [ms]</td><td><input type='number' name='beepDuration' value='");
  html += String(beepDuration);
  html += F("'></td></tr>");

  html += F("<tr><td>PRE_OFF_BEEP_BEFORE_OFF [ms]</td><td><input type='number' name='preOffBeepBeforeOff' value='");
  html += String(preOffBeepBeforeOff);
  html += F("'></td></tr>");

  html += F("<tr><td>PRE_OFF_BEEP_DURATION [ms]</td><td><input type='number' name='preOffBeepDuration' value='");
  html += String(preOffBeepDuration);
  html += F("'></td></tr>");

  html += F("<tr><td>PRE_OFF_BEEP_FREQUENCY [Hz]</td><td><input type='number' name='preOffBeepFrequency' value='");
  html += String(preOffBeepFrequency);
  html += F("'></td></tr>");

  html += F("<tr><td>Znamiennik</td><td><input type='text' name='znamiennik' value='");
  html += znamiennikStr;
  html += F("'></td></tr>");

  html += F("</table>");

  html += F("<div class='btnrow'>");
  html += F("<button type='button' class='btn' onclick=\"location.href='/read'\">READ</button>");
  html += F("<button type='submit' class='btn'>WRITE</button>");
  html += F("</div>");

  html += F("<div class='btnrow'>");
  html += F("<button type='button' class='btn' onclick=\"location.href='/'\">STAN</button>");
  html += F("</div>");

  html += F("</form></div>");
  html += F("</div></body></html>");

  server.send(200, "text/html", html);
}

void handleRead() {
  loadConfig();
  server.sendHeader("Location", "/settings", true);
  server.send(302, "text/plain", "");
}

void handleWrite() {
  if (server.hasArg("wifiSsid"))  wifiSsid  = server.arg("wifiSsid");
  if (server.hasArg("wifiPass"))  wifiPass  = server.arg("wifiPass");
  if (server.hasArg("wifiDhcp"))  wifiUseDhcp = (server.arg("wifiDhcp").toInt() != 0);

  if (server.hasArg("enableVoice")) enableVoice = (server.arg("enableVoice").toInt() != 0);

  // NOWE: voiceInterval
  if (server.hasArg("voiceInterval")) voiceInterval = server.arg("voiceInterval").toInt();

  if (server.hasArg("timerDuration"))        timerDuration        = server.arg("timerDuration").toInt();
  if (server.hasArg("signalStartTime"))      signalStartTimeCfg   = server.arg("signalStartTime").toInt();
  if (server.hasArg("toneFrequency"))        toneFrequency        = server.arg("toneFrequency").toInt();
  if (server.hasArg("toneDelay"))            toneDelay            = server.arg("toneDelay").toInt();
  if (server.hasArg("dotDuration"))          dotDuration          = server.arg("dotDuration").toInt();

  if (server.hasArg("enableMorse"))          enableMorse          = (server.arg("enableMorse").toInt() != 0);
  if (server.hasArg("enableEdgeBeep"))       enableEdgeBeep       = (server.arg("enableEdgeBeep").toInt() != 0);
  if (server.hasArg("enablePreOffBeep"))     enablePreOffBeep     = (server.arg("enablePreOffBeep").toInt() != 0);

  if (server.hasArg("morseInterval"))        morseInterval        = server.arg("morseInterval").toInt();
  if (server.hasArg("beepDelay"))            beepDelay            = server.arg("beepDelay").toInt();
  if (server.hasArg("beepDuration"))         beepDuration         = server.arg("beepDuration").toInt();
  if (server.hasArg("preOffBeepBeforeOff"))  preOffBeepBeforeOff  = server.arg("preOffBeepBeforeOff").toInt();
  if (server.hasArg("preOffBeepDuration"))   preOffBeepDuration   = server.arg("preOffBeepDuration").toInt();
  if (server.hasArg("preOffBeepFrequency"))  preOffBeepFrequency  = server.arg("preOffBeepFrequency").toInt();

  if (server.hasArg("znamiennik"))           znamiennikStr        = server.arg("znamiennik");

  saveConfig();

  server.sendHeader("Location", "/settings", true);
  server.send(302, "text/plain", "");
}

void setupWebServer() {
  server.on("/",           handleStatePage);
  server.on("/settings",   handleSettingsPage);
  server.on("/status",     handleStatus);
  server.on("/read",       handleRead);
  server.on("/write",      handleWrite);
  server.begin();
}

// ==============================
// LED WiFi na D2
// ==============================
void wifiLedProcess() {
  if (WiFi.status() == WL_CONNECTED) {
    unsigned long now = millis();
    if (now - wifiLedLastToggle >= 500) {
      wifiLedState = !wifiLedState;
      digitalWrite(PIN_WIFI_LED, wifiLedState ? HIGH : LOW);
      wifiLedLastToggle = now;
    }
  } else {
    wifiLedState = false;
    digitalWrite(PIN_WIFI_LED, LOW);
    wifiLedLastToggle = millis();
  }
}

// ==============================
// Morse
// ==============================
void morseProcess() {
  auto getMorseCode = [](char c) -> const char* {
    if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';
    for (unsigned int i = 0; i < sizeof(morseTable)/sizeof(morseTable[0]); i++) {
      if (morseTable[i].symbol == c) return morseTable[i].code;
    }
    return "";
  };

  if (morseIndex >= (int)znamiennikStr.length()) {
    morsePlaying = false;
    buzzerOff();
    return;
  }

  const char* currentCode = getMorseCode(znamiennikStr[morseIndex]);
  int codeLen = strlen(currentCode);
  unsigned long now = millis();

  switch (morseState) {
    case IDLE:
      if (morseCharIndex < codeLen) {
        char symbol = currentCode[morseCharIndex];
        if (symbol == '.') {
          buzzerOn(toneFrequency);
          morseElementStart = now;
          morseState = DOT;
        } else if (symbol == '-') {
          buzzerOn(toneFrequency);
          morseElementStart = now;
          morseState = DASH;
        } else {
          morseCharIndex++;
        }
      } else {
        buzzerOff();
        morseElementStart = now;
        morseState = GAP_LETTER;
      }
      break;

    case DOT:
      if (now - morseElementStart >= DOT_UNIT) {
        buzzerOff();
        morseElementStart = now;
        morseState = GAP_ELEMENT;
      }
      break;

    case DASH:
      if (now - morseElementStart >= DASH_UNIT) {
        buzzerOff();
        morseElementStart = now;
        morseState = GAP_ELEMENT;
      }
      break;

    case GAP_ELEMENT:
      if (now - morseElementStart >= GAP_ELEMENT_UNIT) {
        morseCharIndex++;
        morseState = IDLE;
      }
      break;

    case GAP_LETTER:
      if (now - morseElementStart >= GAP_LETTER_UNIT) {
        morseCharIndex = 0;
        morseIndex++;
        morseState = IDLE;
      }
      break;
  }
}

// ==============================
// DFPlayer: logika wyzwalania po GPIO32
// + ograniczenie minimalnego czasu VOICE_INTERVAL
// ==============================
void dfPlayerProcess() {
  if (!enableVoice) return;

  static int prevOut = LOW;
  int outNow = digitalRead(PIN_OUTPUT);
  unsigned long now = millis();

  if (outNow == HIGH && prevOut == LOW) {
    dfPlayArmed = true;
    dfPlayDoneThisHigh = false;
    dfPlayTriggerAt = now + DF_PLAY_DELAY_MS;
  }

  if (outNow == LOW && prevOut == HIGH) {
    dfPlayArmed = false;
    dfPlayDoneThisHigh = false;
  }

  if (dfPlayArmed && !dfPlayDoneThisHigh && outNow == HIGH && now >= dfPlayTriggerAt) {
    // NOWE: minimalny odstęp czasu jak morseInterval
    if (now - lastVoicePlayTime >= voiceInterval) {
      dfPlayTrack1();
      lastVoicePlayTime = now;
      dfPlayDoneThisHigh = true;
    }
    // jeśli za wcześnie, to po prostu nie odtwarzamy w tym cyklu HIGH
  }

  prevOut = outNow;
}

// ==============================
// Setup / Loop
// ==============================
void setup() {
  loadConfig();

  pinMode(PIN_INPUT_1, INPUT);
  pinMode(PIN_INPUT_2, INPUT);
  pinMode(PIN_OUTPUT,  OUTPUT);
  pinMode(PIN_LED,     OUTPUT);

  pinMode(PIN_WIFI_LED, OUTPUT);
  digitalWrite(PIN_WIFI_LED, LOW);

  digitalWrite(PIN_OUTPUT, LOW);
  digitalWrite(PIN_LED,    LOW);

  ledcAttachChannel(PIN_BUZZER, (uint32_t)toneFrequency, 8, 0);
  buzzerOff();

  Wire.begin(I2C_SDA, I2C_SCL);
  adsOk = ads.begin(ADS1115_ADDR);
  if (adsOk) ads.setGain(GAIN_TWOTHIRDS);

  if (enableVoice) {
    DFSerial.begin(DF_BAUD, SERIAL_8N1, DF_RX_PIN, DF_TX_PIN);
  } else {
    DFSerial.end();
  }

  setupWiFi();
  setupWebServer();
}

void loop() {
  server.handleClient();
  wifiLedProcess();
  dfPlayerProcess();

  int input1 = digitalRead(PIN_INPUT_1);
  int input2 = digitalRead(PIN_INPUT_2);

  if (!outputHigh) {
    ledState = false;
    digitalWrite(PIN_LED, LOW);

    if (input1 == LOW && input2 == LOW) {
      if (!inputActive) {
        signalStartTimeMs = millis();
        inputActive = true;
      } else if (millis() - signalStartTimeMs >= signalStartTimeCfg) {
        digitalWrite(PIN_OUTPUT, HIGH);
        outputHighStartTime = millis();

        ledLastToggleTime = millis();
        ledState = true;
        digitalWrite(PIN_LED, HIGH);

        outputHigh = true;
        timerRunning = false;
        inputActive = false;
        morseGeneratedThisCycle = false;

        beepPending = false;
        beepPlaying = false;
        preOffBeepPending = false;
        preOffBeepPlaying = false;
        morsePending = false;
        morsePlaying = false;
        morseIndex = 0;
        morseCharIndex = 0;

        buzzerOff();
      }
    } else {
      inputActive = false;
    }
  } else {
    unsigned long now = millis();
    if (now - ledLastToggleTime >= 500) {
      ledState = !ledState;
      digitalWrite(PIN_LED, ledState ? HIGH : LOW);
      ledLastToggleTime = now;
    }

    static int prevInput1 = HIGH;

    if (input1 == HIGH && prevInput1 == LOW) {
      outputHighStartTime = millis();
      timerRunning = true;

      if (enablePreOffBeep && timerDuration > preOffBeepBeforeOff) {
        preOffBeepTriggerTime = outputHighStartTime + (timerDuration - preOffBeepBeforeOff);
        preOffBeepPending = true;
        preOffBeepPlaying = false;
      } else {
        preOffBeepPending = false;
        preOffBeepPlaying = false;
      }

      if (enableEdgeBeep && digitalRead(PIN_OUTPUT) == HIGH) {
        beepPending = true;
        beepStartTime = millis() + beepDelay;
      }

      if (enableMorse &&
          !morseGeneratedThisCycle &&
          (millis() - lastMorseStartTime >= morseInterval)) {
        morsePending = true;
        morseStartTime = millis() + toneDelay;
      }
    } else if (input1 == LOW && prevInput1 == HIGH) {
      timerRunning = false;
      morsePlaying = false;
      morsePending = false;
      beepPending = false;
      beepPlaying = false;
      preOffBeepPending = false;
      preOffBeepPlaying = false;
      buzzerOff();
    }
    prevInput1 = input1;

    unsigned long now2 = millis();

    if (enableMorse && morsePending && now2 >= morseStartTime) {
      morsePlaying = true;
      morsePending = false;
      morseElementStart = now2;
      morseState = IDLE;
      morseIndex = 0;
      morseCharIndex = 0;
      morseGeneratedThisCycle = true;
      lastMorseStartTime = now2;
    }

    if (enableEdgeBeep && beepPending && now2 >= beepStartTime) {
      buzzerOn(toneFrequency);
      beepPending = false;
      beepPlaying = true;
      beepStartTime = now2;
    }

    if (enableEdgeBeep && beepPlaying && now2 - beepStartTime >= beepDuration) {
      buzzerOff();
      beepPlaying = false;
    }

    if (enablePreOffBeep && preOffBeepPending && now2 >= preOffBeepTriggerTime) {
      if (digitalRead(PIN_OUTPUT) == HIGH) {
        buzzerOn(preOffBeepFrequency);
        preOffBeepStartTime = now2;
        preOffBeepPlaying = true;
        preOffFreqLastUpdate = now2;
      }
      preOffBeepPending = false;
    }

    if (enablePreOffBeep && preOffBeepPlaying) {
      if (now2 - preOffFreqLastUpdate >= PREOFF_FREQ_UPDATE_MS) {
        buzzerOn(preOffBeepFrequency);
        preOffFreqLastUpdate = now2;
      }
    }

    if (enablePreOffBeep &&
        preOffBeepPlaying &&
        now2 - preOffBeepStartTime >= preOffBeepDuration) {
      buzzerOff();
      preOffBeepPlaying = false;
    }

    if (enableMorse && morsePlaying) {
      morseProcess();
    }

    if (timerRunning && now2 - outputHighStartTime >= timerDuration) {
      digitalWrite(PIN_OUTPUT, LOW);
      digitalWrite(PIN_LED, LOW);
      ledState = false;

      outputHigh = false;
      timerRunning = false;
      morsePlaying = false;
      morsePending  = false;
      beepPending   = false;
      beepPlaying   = false;
      preOffBeepPending = false;
      preOffBeepPlaying = false;
      buzzerOff();
    }
  }
}
