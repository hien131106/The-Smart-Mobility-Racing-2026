// RX_Q7: receiver derived from RX_TorX (manual + autonomous, no OLED/battery telemetry)
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <esp_wifi.h>
#include <math.h>

// ===== PINS / PWM =====
#define LED_PIN         2
#define RPWM_PIN        25
#define LPWM_PIN        26
#define SERVO_PIN       27
#define BUZZER_PIN      4    // còi
#define PWM_FREQ_HZ     2000
#define PWM_RESOLUTION  8
#define RPWM_CHANNEL    4
#define LPWM_CHANNEL    5
#define LEFT_BLINKER    16
#define RIGHT_BLINKER   17

// ===== CẢM BIẾN SIÊU ÂM (5 CON - trái sang phải) =====
// S1=trái ngoài, S2=trái trong, S3=giữa, S4=phải trong, S5=phải ngoài
#define NUM_SENSORS 5
const int trigPins[NUM_SENSORS] = {23, 22, 21, 19, 18};
const int echoPins[NUM_SENSORS] = {33, 32, 35, 34, 39};
int distances[NUM_SENSORS];

// ===== NGƯỠNG KHOẢNG CÁCH AUTO (cm) =====
#define DIST_BACK   5   // < này: lùi
#define DIST_STEER  29  // < này (và > DIST_BACK): tiến + bẻ lái
#define DIST_STEER_2 18

// ===== TỐC ĐỘ AUTO (điều chỉnh qua trim TX) =====
uint8_t autoSpd = 13;

// ===== GÓC SERVO AUTO (constrain 65-125) =====
#define STEER_C         90
#define STEER_L_MEDIUM  65
#define STEER_R_MEDIUM 125

// ===== STATE MACHINE LÙI =====
enum AutoState { NAVIGATE, BACKING };
AutoState autoState     = NAVIGATE;
uint8_t   autoDriveDir  = 0; // 0=STOP, 1=FORWARD, 2=BACKWARD
uint8_t   autoSteerDir  = 0; // 0=CENTER, 1=LEFT, 2=RIGHT
unsigned long backStart = 0;
unsigned long backDur   = 0;

// ===== COMM / TIMEOUT =====
const unsigned long LINK_TIMEOUT = 500; // ms

// ===== CONTROL DATA (matches TX_Q7) =====
typedef struct {
  uint16_t throttle;
  uint16_t steer;
  uint8_t  mode;       // 1 = MANUAL, 0 = AUTO
  uint16_t trimGas;     // 0-4095 → giới hạn tốc độ tối đa (manual)
  uint16_t trimSteer;   // 0-4095 → góc lái giữ khi thả joystick
  uint16_t trimAutoGas; // 0-50 → tốc độ tự hành
  uint8_t  horn;       // 1 = nhấn nút còi, 0 = không
} ControlData;

ControlData rxData;
unsigned long lastRecv = 0;
bool linkEstablished = false;

uint8_t txMacAddr[6];
bool txMacSaved = false;
bool txPeerAdded = false;

Servo steeringServo;

// ===== MANUAL PARAMETERS (from RX_TorX) =====
#define THROTTLE_CENTER     1850
#define THROTTLE_RANGE_FWD  1550
#define THROTTLE_RANGE_REV  890
#define STEER_MIN_ADC       280
#define STEER_MAX_ADC       3730
#define STEER_CENTER_ADC    1996
#define DEADZONE            60
#define PWM_MAX_FWD         255
#define PWM_MAX_REV         159
#define EXPO_GAIN           1.8
#define RAMP_UP             1
#define RAMP_DOWN           2
#define BLINK_INTERVAL      100

int currentPWM = 0, targetPWM = 0;
bool forward = true;
int8_t prevMode = -1; // track previous mode to reset state on mode switch
unsigned long prevBlink = 0;
bool blinkStateManual   = false;

// ===== CALLBACK =====
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (len == sizeof(ControlData)) {
    memcpy(&rxData, incomingData, sizeof(rxData));
    lastRecv = millis();
    linkEstablished = true;
    if (!txMacSaved && !txPeerAdded) {
      memcpy(txMacAddr, mac, 6);
      txMacSaved = true;
    }
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
void driveForward(uint8_t spd)
{
  autoDriveDir = 1;
  ledcWrite(RPWM_CHANNEL, 0);
  ledcWrite(LPWM_CHANNEL, spd);
  currentPWM = spd;
  forward = true;
}
void driveBackward(uint8_t spd)
{
  autoDriveDir = 2;
  ledcWrite(LPWM_CHANNEL, 0);
  ledcWrite(RPWM_CHANNEL, spd);
  currentPWM = spd;
  forward = false;
}
void autoSteer(int angle)
{
  if      (angle < 85) autoSteerDir = 1; // LEFT
  else if (angle > 95) autoSteerDir = 2; // RIGHT
  else                 autoSteerDir = 0; // CENTER
  steeringServo.write(angle);
}

// ===== ĐỌC CẢM BIẾN =====
void readUltrasonics()
{
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

// ===== AUTO NAVIGATE =====
// Sensor layout (S1..S5):
//   l_out = distances[0]  (S1)
//   l_mid = distances[1]  (S2)
//   center= distances[2]  (S3)
//   r_mid = distances[3]  (S4)
//   r_out = distances[4]  (S5)
void autoNavigate()
{
  int l_out  = distances[0];
  int l_mid  = distances[1];
  int center = distances[2];
  int r_mid  = distances[3];
  int r_out  = distances[4];

  // Đang lùi → chờ hết thời gian rồi mới xét lại
  if (autoState == BACKING)
  {
    if (millis() - backStart < backDur) return;
    stopMotor();
    autoState = NAVIGATE;
  }

  // Case 2: trái < DIST_BACK → lùi + bẻ trái 1s
  if (l_out < DIST_BACK || l_mid < DIST_BACK)
  {
    autoSteer(STEER_L_MEDIUM);
    driveBackward(autoSpd);
    autoState = BACKING; backStart = millis(); backDur = 1000;
    return;
  }

  // Case 3: phải < DIST_BACK → lùi + bẻ phải 1s
  if (r_out < DIST_BACK || r_mid < DIST_BACK)
  {
    autoSteer(STEER_R_MEDIUM);
    driveBackward(autoSpd);
    autoState = BACKING; backStart = millis(); backDur = 1000;
    return;
  }

  // Case 1: giữa < DIST_BACK → lùi thẳng 1s
  if (center < DIST_BACK)
  {
    autoSteer(STEER_C);
    driveBackward(autoSpd);
    autoState = BACKING; backStart = millis(); backDur = 1000;
    return;
  }

  // Case 4: trái trong vùng cảnh báo → tiến + bẻ phải
  if ((l_out > DIST_BACK && l_out < DIST_STEER_2) ||
      (l_mid > DIST_BACK && l_mid < DIST_STEER))
  {
    autoSteer(STEER_R_MEDIUM);
    driveForward(autoSpd);
    return;
  }

  // Case 5: phải trong vùng cảnh báo → tiến + bẻ trái
  if ((r_out > DIST_BACK && r_out < DIST_STEER_2) ||
      (r_mid > DIST_BACK && r_mid < DIST_STEER))
  {
    autoSteer(STEER_L_MEDIUM);
    driveForward(autoSpd);
    return;
  }

  // Mặc định: đường thông → tiến thẳng
  autoSteer(STEER_C);
  driveForward(autoSpd);
}

// ===== SETUP =====
void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LEFT_BLINKER, OUTPUT);
  pinMode(RIGHT_BLINKER, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(90);

  ledcSetup(RPWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcSetup(LPWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttachPin(RPWM_PIN, RPWM_CHANNEL);
  ledcAttachPin(LPWM_PIN, LPWM_CHANNEL);
  ledcWrite(RPWM_CHANNEL, 0);
  ledcWrite(LPWM_CHANNEL, 0);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
}

// ===== LOOP =====
void loop()
{
  unsigned long now = millis();

  // add peer once outside the callback
  if (txMacSaved && !txPeerAdded) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, txMacAddr, 6);
    peer.channel = 11;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) txPeerAdded = true;
  }

  bool linkOK = linkEstablished && (now - lastRecv < LINK_TIMEOUT);
  if (!linkOK) {
    stopMotor();
    steeringServo.write(90);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    autoState = NAVIGATE;
    return;
  }

  // Còi: kêu khi TX nhấn nút
  digitalWrite(BUZZER_PIN, rxData.horn ? HIGH : LOW);

  // detect mode change and reset motor state safely on transition
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
    autoNavigate();
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
    forward = (err > 0);
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

  // Servo lái — trimSteer used when joystick centered
  int trimSteerAngle = map(rxData.trimSteer, 0, 4095, 65, 125);

  int servoAngle;
  if (abs((int32_t)rxData.steer - STEER_CENTER_ADC) < DEADZONE)
  {
    servoAngle = trimSteerAngle;
  } else {
    servoAngle = constrain(map(rxData.steer, STEER_MIN_ADC, STEER_MAX_ADC, 125, 65), 65, 125);
  }
  steeringServo.write(servoAngle);

  // Xi nhan
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

  // simple status LED when link present
  digitalWrite(LED_PIN, HIGH);

  delay(1);
}
