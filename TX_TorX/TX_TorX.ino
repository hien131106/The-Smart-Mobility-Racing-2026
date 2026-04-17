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
#define THROTTLE_PIN 33
#define STEER_PIN    32
#define MODE_PIN     26
#define LED_STATUS   2
#define LED_AUTO     16
#define LED_MANUAL   17
#define FILTER_SIZE  5

uint8_t receiverMac[] = {0x88, 0x57, 0x21, 0x24, 0xA0, 0x40};

typedef struct {
  uint16_t throttle;
  uint16_t steer;
  uint8_t  mode;    // 1 = MANUAL (HIGH), 0 = AUTO (LOW)
} ControlData;

ControlData txData;

/* ===== FILTER ===== */
uint16_t throttleBuf[FILTER_SIZE] = {0};
uint16_t steerBuf[FILTER_SIZE]    = {0};
uint32_t throttleSum = 0;
uint32_t steerSum    = 0;
uint8_t  filterIndex = 0;

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
  uint16_t rawThrottle = analogRead(THROTTLE_PIN);
  uint16_t rawSteer    = analogRead(STEER_PIN);

  txData.throttle = movingAverage(throttleBuf, throttleSum, rawThrottle);
  txData.steer    = movingAverage(steerBuf,    steerSum,    rawSteer);

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
    display.setCursor(0, 0);
    display.println("ESP32 RC TRANSMITTER");

    display.print("ESPNow: ");
    display.println(sendOK ? "OK" : "NOT OK");

    display.print("Mode: ");
    display.println(autoMode ? "AUTO" : "MANUAL");

    display.print("Throttle: ");
    display.println(txData.throttle);

    display.print("Steering: ");
    display.println(txData.steer);

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
