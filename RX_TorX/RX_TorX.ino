#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <esp_wifi.h>
#include <math.h>

// ===== CHÂN PHẦN CỨNG =====
#define LED_PIN         2
#define RPWM_PIN        25
#define LPWM_PIN        26
#define SERVO_PIN       27
#define LEFT_BLINKER    16
#define RIGHT_BLINKER   17

// ===== CẢM BIẾN SIÊU ÂM (5 CON - trái sang phải) =====
// S1=trái ngoài, S2=trái trong, S3=giữa, S4=phải trong, S5=phải ngoài
#define NUM_SENSORS 5
const int trigPins[NUM_SENSORS] = {23, 22, 21, 19, 18};
const int echoPins[NUM_SENSORS] = {33, 32, 35, 34, 39};
int distances[NUM_SENSORS];

// ===== NGƯỠNG KHOẢNG CÁCH AUTO (cm) =====
#define FIRST_WARNING   10
#define SECOND_WARNING  30
#define THIRD_WARNING   35

// ===== TỐC ĐỘ AUTO (được điều chỉnh qua trim chân 35, range 10-30) =====
uint8_t autoSpd = 13;
// Map auto trim to real PWM range (adjust MIN_AUTO_PWM to the minimum PWM
// that reliably moves the motor on your hardware)
#define MIN_AUTO_PWM  60
#define MAX_AUTO_PWM 120

// ===== GÓC SERVO AUTO (constrain 65-125) =====
#define STEER_C         90
#define STEER_L_MEDIUM  65   // 90 - 40 → capped
#define STEER_L_HARD    75   // 90 - 50 → capped
#define STEER_R_MEDIUM 115   // 90 + 40 → capped
#define STEER_R_HARD   125   // 90 + 50 → capped

// ===== STATE MACHINE LÙI =====
enum AutoState { NAVIGATE, BACKING };
AutoState autoState      = NAVIGATE;
uint8_t   autoDriveDir   = 0; // 0=STOP, 1=FORWARD, 2=BACKWARD
uint8_t   autoSteerDir   = 0; // 0=CENTER, 1=LEFT, 2=RIGHT
unsigned long backStart  = 0;
unsigned long backDur    = 0;   // thời gian lùi, set riêng mỗi maneuver

// ===== BLINK AUTO =====
unsigned long lastBlinkAuto = 0;
bool ledStateAuto           = false;

// ===== CẤU TRÚC GÓI ESP-NOW =====
typedef struct {
  uint16_t throttle;
  uint16_t steer;
  uint8_t  mode;       // 1 = MANUAL, 0 = AUTO
  uint16_t trimGas;     // 0-4095 → giới hạn tốc độ tối đa (manual)
  uint16_t trimSteer;   // 0-4095 → góc lái giữ khi thả joystick
  uint16_t trimAutoGas; // logical autospeed (10-30) — TX sends mapped 10-30
} ControlData;

typedef struct {
  uint16_t dist[5];    // S1..S5 (cm), 999 = không phát hiện
  uint8_t  driveState; // 0=STOP, 1=FORWARD, 2=BACKWARD
  uint8_t  steerState; // 0=CENTER, 1=LEFT, 2=RIGHT
  uint16_t energy100;  // battery voltage in hundredths of volts (e.g., 1265 -> 12.65V)
} SensorData;

ControlData rxData;
unsigned long lastRecv = 0;
bool linkEstablished   = false;

// MAC của TX (tay cầm) — lưu từ gói đầu tiên nhận được
uint8_t txMacAddr[6];
bool    txMacSaved  = false; // MAC đã lưu, chờ add peer từ main loop
bool    txPeerAdded = false; // peer đã add xong, có thể gửi

// ===== THAM SỐ MANUAL =====
#define THROTTLE_CENTER     1850
#define THROTTLE_RANGE_FWD  1550
#define THROTTLE_RANGE_REV  890
#define STEER_MIN_ADC       280
#define STEER_MAX_ADC       3730
#define STEER_CENTER_ADC    1996
#define DEADZONE            60
#define LINK_TIMEOUT        500
#define PWM_MAX_FWD         255
#define PWM_MAX_REV         159
#define EXPO_GAIN           2.5
#define RAMP_UP             4
#define RAMP_DOWN           8
#define BLINK_INTERVAL      100

int currentPWM = 0, targetPWM = 0;
bool forward = true;
unsigned long prevBlink = 0;
bool blinkStateManual   = false;
unsigned long ledTimer  = 0;
bool ledState           = false;

Servo steeringServo;

// PWM (use LEDC instead of analogWrite for deterministic behaviour)
#define PWM_FREQ_HZ      21000
#define PWM_RESOLUTION   8
#define RPWM_CHANNEL     4
#define LPWM_CHANNEL     5

/* If 1, invert throttle sign (useful if motor wiring is reversed) */
#define THROTTLE_INVERT 0

int8_t prevMode = -1; // track previous mode to reset state on mode switch

// ===== BATTERY MONITOR =====
#define BAT_PIN           36    // ADC pin used to measure divider output (VP)
#define ADC_REF_VOLTAGE   3.30f
#define R_DIV_TOP         51000.0f
#define R_DIV_BOTTOM      10000.0f
#define ENERGY_FILTER_SIZE 10
// Calibration factor to correct ADC + divider inaccuracies.
// Measured: OLED showed 11.23V but actual is 12.26V -> factor = 12.26/11.23 ~= 1.09189
#define CALIB_FACTOR 1.09189f

uint16_t energyBuf[ENERGY_FILTER_SIZE] = {0};
uint32_t energySum = 0;
uint8_t  energyIndex = 0;
uint16_t energyAvgADC = 0; // averaged ADC reading
uint16_t energy100 = 0;    // measured battery voltage * 100 (hundredths of volt)

// periodic sensor send
unsigned long lastSensorSend = 0;
const unsigned long SENSOR_SEND_INTERVAL = 100; // ms

// send sensor data helper
void sendSensorDataIfDue(bool includeSensors) {
  if (!txPeerAdded) return;
  unsigned long now = millis();
  if (now - lastSensorSend < SENSOR_SEND_INTERVAL) return;
  lastSensorSend = now;

  SensorData sData;
  if (includeSensors) {
    for (int i = 0; i < 5; i++) sData.dist[i] = (uint16_t)distances[i];
    sData.driveState = autoDriveDir;
    sData.steerState = autoSteerDir;
  } else {
    // indicate sensors not provided
    for (int i = 0; i < 5; i++) sData.dist[i] = 999;
    sData.driveState = 0;
    sData.steerState = 0;
  }
  sData.energy100 = energy100;
  esp_now_send(txMacAddr, (uint8_t *)&sData, sizeof(sData));
}

// ===== ĐỌC CẢM BIẾN =====
void readUltrasonics() {
  for (int i = 0; i < NUM_SENSORS; i++) 
  {
    digitalWrite(trigPins[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[i], LOW);
    unsigned long dur = pulseIn(echoPins[i], HIGH, 30000);
    distances[i] = (dur == 0) ? 999 : int(dur / 2 / 29.412);
    delay(10);
  }
}

// ===== MOTOR / SERVO HELPERS =====
void stopMotor()
{
  autoDriveDir = 0;
  ledcWrite(RPWM_CHANNEL, 0);
  ledcWrite(LPWM_CHANNEL, 0);
  currentPWM = 0; targetPWM = 0;
}
void driveForward(uint8_t spd)  {
  autoDriveDir = 1;
  ledcWrite(RPWM_CHANNEL, 0);
  ledcWrite(LPWM_CHANNEL, spd);
  currentPWM = spd;
  forward = true;
}
void driveBackward(uint8_t spd) {
  autoDriveDir = 2;
  ledcWrite(LPWM_CHANNEL, 0);
  ledcWrite(RPWM_CHANNEL, spd);
  currentPWM = spd;
  forward = false;
}
void autoSteer(int angle) {
  if      (angle < 85) autoSteerDir = 1; // LEFT
  else if (angle > 95) autoSteerDir = 2; // RIGHT
  else                 autoSteerDir = 0; // CENTER
  steeringServo.write(angle);
}

// ===== BLINK HELPERS (AUTO) =====
void blinkLeft() {
  unsigned long now = millis();
  if (now - lastBlinkAuto >= 250) { lastBlinkAuto = now; ledStateAuto = !ledStateAuto; }
  digitalWrite(LEFT_BLINKER,  ledStateAuto ? HIGH : LOW);
  digitalWrite(RIGHT_BLINKER, LOW);
}
void blinkRight() {
  unsigned long now = millis();
  if (now - lastBlinkAuto >= 250) { lastBlinkAuto = now; ledStateAuto = !ledStateAuto; }
  digitalWrite(RIGHT_BLINKER, ledStateAuto ? HIGH : LOW);
  digitalWrite(LEFT_BLINKER,  LOW);
}
void notBlink() {
  digitalWrite(LEFT_BLINKER,  LOW);
  digitalWrite(RIGHT_BLINKER, LOW);
}

// ===== AUTO NAVIGATE =====
// Sensor mapping (S1..S5 trái→phải):
//   fl = S2 = distances[1]   (front-left)
//   fm = S3 = distances[2]   (front-middle)
//   fr = S4 = distances[3]   (front-right)
//   l  = S1 = distances[0]   (side-left)
//   r  = S5 = distances[4]   (side-right)
void autoNavigate()
{
  int fl = distances[1];
  int fm = distances[2];
  int fr = distances[3];
  int l  = distances[0];
  int r  = distances[4];

  // --- Đang trong trạng thái lùi: giữ đến hết thời gian ---
  if (autoState == BACKING)
  {
    if (millis() - backStart < backDur) return;
    stopMotor();
    autoState = NAVIGATE;
  }

  // --- Quyết định hướng đi (thuật toán gốc) ---

  // Giữa quá gần → lùi thẳng
  if (fm <= FIRST_WARNING)
  {
    autoSteer(STEER_C);
    driveBackward(autoSpd);
    notBlink();
    autoState = BACKING;
    backStart = millis();
    backDur = 1000;
  }
  // Góc trái rất gần → lùi nghiêng trái
  else if (fl <= FIRST_WARNING)
  {
    autoSteer(STEER_L_MEDIUM);
    driveBackward(autoSpd);
    blinkLeft();
    autoState = BACKING;
    backStart = millis();
    backDur = 1000;
  }
  // Góc phải rất gần → lùi nghiêng phải
  else if (fr <= FIRST_WARNING)
  {
    autoSteer(STEER_R_MEDIUM);
    driveBackward(autoSpd);
    blinkRight();
    autoState = BACKING;
    backStart = millis();
    backDur = 1000;
  }
  // Trái + giữa gần → quẹo phải mạnh
  else if (fl <= SECOND_WARNING && fm <= SECOND_WARNING)
  {
    autoSteer(STEER_R_HARD);
    driveForward(autoSpd);
    blinkRight();
  }
  // Phải + giữa gần → quẹo trái mạnh
  else if (fr <= SECOND_WARNING && fm <= SECOND_WARNING)
  {
    autoSteer(STEER_L_HARD);
    driveForward(autoSpd);
    blinkLeft();
  }
  // Trái cảnh báo → nghiêng phải vừa
  else if (fl > FIRST_WARNING && fl < THIRD_WARNING)
  {
    autoSteer(STEER_R_MEDIUM);
    driveForward(autoSpd);
    blinkRight();
  }
  // Phải cảnh báo → nghiêng trái vừa
  else if (fr > FIRST_WARNING && fr < THIRD_WARNING)
  {
    autoSteer(STEER_L_MEDIUM);
    driveForward(autoSpd);
    blinkLeft();
  }
  // Cạnh trái quá gần → lùi nghiêng trái tránh tường
  else if (l <= FIRST_WARNING)
  {
    autoSteer(STEER_L_MEDIUM);
    driveBackward(autoSpd);
    blinkLeft();
    autoState = BACKING;
    backStart = millis();
    backDur = 800;
  }
  // Cạnh phải quá gần → lùi nghiêng phải tránh tường
  else if (r <= FIRST_WARNING)
  {
    autoSteer(STEER_R_MEDIUM);
    driveBackward(autoSpd);
    blinkRight();
    autoState = BACKING;
    backStart = millis();
    backDur = 800;
  }
  // Đường thông → tiến thẳng
  else
  {
    autoSteer(STEER_C);
    driveForward(autoSpd);
    notBlink();
  }
}

// ===== CALLBACK =====
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (len == sizeof(ControlData)) {
    memcpy(&rxData, incomingData, sizeof(rxData));
    lastRecv = millis();
    linkEstablished = true;

    // Lưu MAC của TX — KHÔNG gọi esp_now_add_peer từ trong callback
    if (!txMacSaved && !txPeerAdded) {
      memcpy(txMacAddr, mac, 6);
      txMacSaved = true;
    }
  }
}

// ===== SETUP =====
void setup() 
{
  /* Serial removed for performance */

  pinMode(LED_PIN,       OUTPUT);
  pinMode(RPWM_PIN,      OUTPUT);
  pinMode(LPWM_PIN,      OUTPUT);
  pinMode(LEFT_BLINKER,  OUTPUT);
  pinMode(RIGHT_BLINKER, OUTPUT);

  for (int i = 0; i < NUM_SENSORS; i++) 
  {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(90);

  /* LEDC PWM setup for motor outputs */
  ledcSetup(RPWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcSetup(LPWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttachPin(RPWM_PIN, RPWM_CHANNEL);
  ledcAttachPin(LPWM_PIN, LPWM_CHANNEL);
  /* ensure outputs start at 0 */
  ledcWrite(RPWM_CHANNEL, 0);
  ledcWrite(LPWM_CHANNEL, 0);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);

  /* Serial debug removed */
}

// ===== LOOP =====
void loop() 
{
  bool linkOK = linkEstablished && (millis() - lastRecv < LINK_TIMEOUT);

  /* ===== FAILSAFE ===== */
  if (!linkOK) 
  {
    stopMotor();
    steeringServo.write(90);
    digitalWrite(LEFT_BLINKER,  LOW);
    digitalWrite(RIGHT_BLINKER, LOW);
    digitalWrite(LED_PIN, LOW);
    autoState = NAVIGATE;
    return;
  }

  // Thêm TX làm peer ngoài callback (an toàn với WiFi task)
  if (txMacSaved && !txPeerAdded) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, txMacAddr, 6);
    peer.channel = 11;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) txPeerAdded = true;
  }

  // ===== BATTERY READ & AVERAGE =====
  uint16_t rawAdc = analogRead(BAT_PIN);
  // update circular buffer
  energySum -= energyBuf[energyIndex];
  energyBuf[energyIndex] = rawAdc;
  energySum += rawAdc;
  energyIndex++;
  if (energyIndex >= ENERGY_FILTER_SIZE) energyIndex = 0;
  energyAvgADC = energySum / ENERGY_FILTER_SIZE;
  // convert to voltage
  float vout = ((float)energyAvgADC / 4095.0f) * ADC_REF_VOLTAGE;
  float vin = vout * ((R_DIV_TOP + R_DIV_BOTTOM) / R_DIV_BOTTOM);
  // apply calibration factor to correct for ADC/reference/divider error
  energy100 = (uint16_t)roundf(vin * CALIB_FACTOR * 100.0f);

  /* detect mode change and sync state: on any mode transition reset motor outputs safely */
  if (prevMode != -1 && rxData.mode != prevMode) {
    currentPWM = 0;
    targetPWM = 0;
    stopMotor();
  }
  prevMode = rxData.mode;

  /* ===== AUTO MODE ===== */
  if (rxData.mode == 0)
  {
     autoSpd = rxData.trimAutoGas;
    readUltrasonics();

    /* Debug prints removed for performance */

    autoNavigate();

    // prepare and send sensor + energy periodically (include sensors in AUTO)
    sendSensorDataIfDue(true);

    digitalWrite(LED_PIN, HIGH);
    return;
  }

  /* ===== MANUAL MODE ===== */

  // Trim gas → giới hạn tốc độ tối đa (50-255)
  int maxFwd = map(rxData.trimGas, 0, 4095, 50, 255);

  // Throttle expo + ramp
  int16_t err = (int32_t)rxData.throttle - THROTTLE_CENTER;
  if (abs(err) < DEADZONE)
  {
    targetPWM = 0;
  } else {
    /* apply optional inversion for wiring differences */
    forward = ((err > 0) != (THROTTLE_INVERT != 0));
    int range  = forward ? THROTTLE_RANGE_FWD : THROTTLE_RANGE_REV;
    int pwmMax = forward ? maxFwd : PWM_MAX_REV;
    float norm = constrain((float)(abs(err) - DEADZONE) / (float)(range - DEADZONE), 0.0f, 1.0f);
    targetPWM  = (int)(pow(norm, EXPO_GAIN) * pwmMax);
  }
  if      (currentPWM < targetPWM) currentPWM += RAMP_UP;
  else if (currentPWM > targetPWM) currentPWM -= RAMP_DOWN;
  currentPWM = constrain(currentPWM, 0, forward ? maxFwd : PWM_MAX_REV);

  if (currentPWM == 0) 
  {
    ledcWrite(RPWM_CHANNEL, 0); ledcWrite(LPWM_CHANNEL, 0);
  } else if (forward) {
    ledcWrite(RPWM_CHANNEL, 0); ledcWrite(LPWM_CHANNEL, currentPWM);
  } else {
    ledcWrite(RPWM_CHANNEL, currentPWM); ledcWrite(LPWM_CHANNEL, 0);
  }

  // Servo lái
  // Trim steer → góc giữ khi thả joystick (65-125)
  int trimSteerAngle = map(rxData.trimSteer, 0, 4095, 65, 125);

  int servoAngle;
  if (abs((int32_t)rxData.steer - STEER_CENTER_ADC) < DEADZONE)
  {
    // Thả joystick → giữ góc theo biến trở
    servoAngle = trimSteerAngle;
  } else {
    // Đang lái → servo theo joystick
    servoAngle = constrain(map(rxData.steer, STEER_MIN_ADC, STEER_MAX_ADC, 125, 65), 65, 125);
  }
  steeringServo.write(servoAngle);

  // Xi nhan
  unsigned long now = millis();
  if (servoAngle < 75 || servoAngle > 115) 
  {
    if (now - prevBlink >= BLINK_INTERVAL) 
    {
      prevBlink = now; 
      blinkStateManual = !blinkStateManual; 
    }
    if (servoAngle < 75) 
    {
      digitalWrite(LEFT_BLINKER, blinkStateManual); 
      digitalWrite(RIGHT_BLINKER, LOW);
    } else {
      digitalWrite(RIGHT_BLINKER, blinkStateManual); 
      digitalWrite(LEFT_BLINKER, LOW);
    }
  } else {
    digitalWrite(LEFT_BLINKER, LOW); 
    digitalWrite(RIGHT_BLINKER, LOW);
    blinkStateManual = false;
  }

  // LED nhấp nháy
  if (now - ledTimer >= 200) 
  {
    ledTimer = now; 
    ledState = !ledState; 
    digitalWrite(LED_PIN, ledState); 
  }

  /* Runtime debug prints removed for performance */
  // send energy periodically in MANUAL (no sensors)
  sendSensorDataIfDue(false);
}