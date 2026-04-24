#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* ===== OLED ===== */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_SDA      21
#define OLED_SCL      22
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* ===== JOYSTICK & IO ===== */
#define THROTTLE_PIN    33
#define STEER_PIN       32
#define TRIM_GAS_PIN      39   // Biến trở giới hạn tốc độ tối đa (manual)
#define TRIM_STEER_PIN    34   // Biến trở giữ góc lái khi thả joystick
#define TRIM_AUTO_GAS_PIN 35   // Biến trở tốc độ tự hành (15-20)
#define MODE_PIN        26
#define LED_STATUS      2
#define LED_AUTO        16
#define LED_MANUAL      17
#define FILTER_SIZE     5

uint8_t receiverMac[] = {0x88, 0x57, 0x21, 0x24, 0xA0, 0x40};

typedef struct {
  uint16_t throttle;
  uint16_t steer;
  uint8_t  mode;       // 1 = MANUAL (HIGH), 0 = AUTO (LOW)
  uint16_t trimGas;     // 0-4095 → giới hạn tốc độ tối đa (manual)
  uint16_t trimSteer;   // 0-4095 → góc lái giữ khi thả joystick
  uint16_t trimAutoGas; // 0-4095 → tốc độ tự hành (15-20)
} ControlData;

typedef struct {
  uint16_t dist[5];    // S1..S5 (cm), 999 = không phát hiện
  uint8_t  driveState; // 0=STOP, 1=FORWARD, 2=BACKWARD
  uint8_t  steerState; // 0=CENTER, 1=LEFT, 2=RIGHT
} SensorData;

ControlData txData;
SensorData  rxSensor;
bool        sensorDataReceived = false;

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

/* ===== STATUS ===== */
bool sendOK = false;
unsigned long ledTimer = 0;
bool ledState = false;
unsigned long oledTimer = 0;

/* ===== CALLBACK ===== */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  sendOK = (status == ESP_NOW_SEND_SUCCESS);
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (len == sizeof(SensorData)) {
    memcpy(&rxSensor, incomingData, sizeof(rxSensor));
    sensorDataReceived = true;
  }
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
  Serial.begin(115200);

  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_AUTO, OUTPUT);
  pinMode(LED_MANUAL, OUTPUT);
  pinMode(MODE_PIN, INPUT_PULLUP);

  /* OLED */
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  /* ESP-NOW */
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  esp_now_init();
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 11;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  Serial.println("END SETUP");
}

void loop() 
{
  /* ===== READ ADC ===== */
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

  esp_now_send(receiverMac, (uint8_t *)&txData, sizeof(txData));

  /* ===== MODE SWITCH ===== */
  bool autoMode = !digitalRead(MODE_PIN); // LOW (GND) = AUTO, HIGH = MANUAL
  txData.mode = autoMode ? 0 : 1;        // 0 = AUTO, 1 = MANUAL

  if (autoMode) 
  {
    digitalWrite(LED_AUTO, HIGH);
    digitalWrite(LED_MANUAL, LOW);
  } else {
    digitalWrite(LED_AUTO, LOW);
    digitalWrite(LED_MANUAL, HIGH);
  }

  /* ===== OLED UPDATE (10 Hz) ===== */
  if (millis() - oledTimer >= 100)
  {
    oledTimer = millis();
    display.clearDisplay();

    if (autoMode)
    {
      // Header vùng vàng (y=0-15): 2 dòng × 8px, chữ đen nền vàng
      display.fillRect(0, 0, 128, 16, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(19, 0);
      display.print("** AUTO MODE **");
      display.setCursor(0, 8);
      display.print("ESPNow: ");
      display.print(sendOK ? "OK" : "FAIL");
      display.setTextColor(SSD1306_WHITE);

      // Nội dung vùng xanh: 5 khoảng cách siêu âm
      if (sensorDataReceived) {
        // Dòng 1: Middle (S3) căn giữa màn hình
        String mStr = "M:" + String(rxSensor.dist[2]);
        display.setCursor((128 - (int)mStr.length() * 6) / 2, 18);
        display.print(mStr);
        // Dòng 2: L  FL  FR  R theo thứ tự
        display.setCursor(0, 28);
        display.print("L:");   display.print(rxSensor.dist[0]);
        display.print(" FL:"); display.print(rxSensor.dist[1]);
        display.print(" FR:"); display.print(rxSensor.dist[3]);
        display.print(" R:");  display.print(rxSensor.dist[4]);
      } else {
        display.setCursor(0, 18);
        display.print("Waiting sensors...");
      }
      // Dòng 3: tốc độ tự hành
      display.setCursor(0, 38);
      display.print("AutoSpd: ");
      display.println((int)map(txData.trimAutoGas, 0, 4095, 10, 20));

      // Dòng 4: trạng thái tiến/lùi (trái) + lái (phải)
      if (sensorDataReceived) {
        String driveStr = (rxSensor.driveState == 1) ? "FWD" :
                          (rxSensor.driveState == 2) ? "BWD" : "STP";
        String steerStr = (rxSensor.steerState == 1) ? "< LEFT" :
                          (rxSensor.steerState == 2) ? "RIGHT >" : "CENTER";
        display.setCursor(0, 48);
        display.print(driveStr);
        display.setCursor(128 - (int)steerStr.length() * 6, 48);
        display.print(steerStr);
      }
    }
    else
    {
      // Header vùng vàng (y=0-15): 2 dòng × 8px, chữ đen nền vàng
      display.fillRect(0, 0, 128, 16, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(4, 0);
      display.print("ESP32 RC TRANSMITTER");
      display.setCursor(0, 8);
      display.print("ESPNow: ");
      display.print(sendOK ? "OK" : "FAIL");
      display.setTextColor(SSD1306_WHITE);

      // Nội dung vùng xanh (y=18+)
      display.setCursor(0, 18);
      display.print("Throttle: ");
      display.println(txData.throttle);

      display.print("Steering: ");
      display.println(txData.steer);

      display.print("TrimGas: ");
      display.println(map(txData.trimGas, 0, 4095, 50, 255));

      display.print("TrimSteer: ");
      display.println(map(txData.trimSteer, 0, 4095, 65, 125));
    }

    display.display();
  }

  /* ===== STATUS LED BLINK ===== */
  if (sendOK && millis() - ledTimer >= 200) 
  {
    ledTimer = millis();
    ledState = !ledState;
    digitalWrite(LED_STATUS, ledState);
  }
}
