// TX_Q7: transmitter derived from TX_TorX (manual + autonomous, no OLED)
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

/* ===== IO ===== */
#define THROTTLE_PIN    32
#define STEER_PIN       33
#define TRIM_GAS_PIN      39
#define TRIM_STEER_PIN    34
#define TRIM_AUTO_GAS_PIN 35
#define MODE_PIN        26
#define BUTTON_PIN      25   // nút bấm còi
#define LED_STATUS      2
#define LED_AUTO        16
#define LED_MANUAL      17
#define FILTER_SIZE     5

uint8_t receiverMac[] = {0x88, 0x57, 0x21, 0x24, 0xA0, 0x40};

typedef struct {
  uint16_t throttle;
  uint16_t steer;
  uint8_t  mode;       // 1 = MANUAL (HIGH), 0 = AUTO (LOW)
  uint16_t trimGas;
  uint16_t trimSteer;
  uint16_t trimAutoGas; // mapped to 0..50 (logical autospeed)
  uint8_t  horn;       // 1 = nhấn nút còi, 0 = không
} ControlData;

ControlData txData;
bool sendOK = false;

/* ===== FILTER ===== */
uint16_t throttleBuf[FILTER_SIZE]   = {0};
uint16_t steerBuf[FILTER_SIZE]      = {0};
uint16_t trimGasBuf[FILTER_SIZE]     = {0};
uint16_t trimSteerBuf[FILTER_SIZE]   = {0};
uint16_t trimAutoGasBuf[FILTER_SIZE] = {0};
uint32_t throttleSum    = 0;
uint32_t steerSum       = 0;
uint32_t trimGasSum     = 0;
uint32_t trimSteerSum   = 0;
uint32_t trimAutoGasSum = 0;
uint8_t  filterIndex  = 0;

/* ===== CALLBACK ===== */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  sendOK = (status == ESP_NOW_SEND_SUCCESS);
}

uint16_t movingAverage(uint16_t *buffer, uint32_t &sum, uint16_t newVal) 
{
  sum -= buffer[filterIndex];
  buffer[filterIndex] = newVal;
  sum += newVal;
  return sum / FILTER_SIZE;
}

void setup()
{
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_AUTO, OUTPUT);
  pinMode(LED_MANUAL, OUTPUT);
  pinMode(MODE_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(STEER_PIN, INPUT);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE);
  esp_now_init();
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 11;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // initialize filter buffers
  for (uint8_t i = 0; i < FILTER_SIZE; ++i) {
    throttleBuf[i] = analogRead(THROTTLE_PIN);
    steerBuf[i] = analogRead(STEER_PIN);
    trimGasBuf[i] = analogRead(TRIM_GAS_PIN);
    trimSteerBuf[i] = analogRead(TRIM_STEER_PIN);
    trimAutoGasBuf[i] = analogRead(TRIM_AUTO_GAS_PIN);
    throttleSum += throttleBuf[i];
    steerSum += steerBuf[i];
    trimGasSum += trimGasBuf[i];
    trimSteerSum += trimSteerBuf[i];
    trimAutoGasSum += trimAutoGasBuf[i];
  }
}

void loop()
{
  uint16_t rawThrottle   = analogRead(THROTTLE_PIN);
  uint16_t rawSteer      = analogRead(STEER_PIN);
  uint16_t rawTrimGas    = analogRead(TRIM_GAS_PIN);
  uint16_t rawTrimSteer  = analogRead(TRIM_STEER_PIN);
  uint16_t rawTrimAutoGas = analogRead(TRIM_AUTO_GAS_PIN);

  txData.throttle   = movingAverage(throttleBuf,    throttleSum,    rawThrottle);
  txData.steer      = movingAverage(steerBuf,        steerSum,       rawSteer);
  txData.trimGas    = movingAverage(trimGasBuf,      trimGasSum,     rawTrimGas);
  txData.trimSteer  = movingAverage(trimSteerBuf,    trimSteerSum,   rawTrimSteer);
  txData.trimAutoGas = movingAverage(trimAutoGasBuf, trimAutoGasSum, rawTrimAutoGas);

  filterIndex++;
  if (filterIndex >= FILTER_SIZE) filterIndex = 0;

  // map raw ADC -> logical autospeed (0..50) to match TX_TorX behavior
  txData.trimAutoGas = map(txData.trimAutoGas, 0, 4095, 0, 50);

  /* ===== NÚT CÒI ===== */
  txData.horn = (digitalRead(BUTTON_PIN) == LOW) ? 1 : 0; // nhấn nút = LOW (pull-up)

  /* ===== MODE SWITCH ===== */
  bool autoMode = !digitalRead(MODE_PIN); // LOW (GND) = AUTO, HIGH = MANUAL
  txData.mode = autoMode ? 0 : 1;        // 0 = AUTO, 1 = MANUAL

  if (autoMode) {
    digitalWrite(LED_AUTO, HIGH);
    digitalWrite(LED_MANUAL, LOW);
  } else {
    digitalWrite(LED_AUTO, LOW);
    digitalWrite(LED_MANUAL, HIGH);
  }

  esp_now_send(receiverMac, (uint8_t *)&txData, sizeof(txData));

  // status LED blink when transmission OK
  static unsigned long ledTimer = 0;
  static bool ledState = false;
  if (sendOK && millis() - ledTimer >= 200) 
  {
    ledTimer = millis();
    ledState = !ledState;
    digitalWrite(LED_STATUS, ledState);
  }
  delay (20);
}