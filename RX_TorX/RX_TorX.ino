#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <math.h>

// Khai báo chân
#define LED_PIN    2
#define RPWM_PIN   25   // BTS7960 tiến
#define LPWM_PIN   26   // BTS7960 lùi
#define SERVO_PIN  27   // Servo lái

#define LEFT_BLINKER    16
#define RIGHT_BLINKER   17
#define BLINK_INTERVAL 100

// ===== CẢM BIẾN SIÊU ÂM =====
#define TRIG1 23
#define TRIG2 22
#define TRIG3 21
#define TRIG4 19
#define TRIG5 18

#define ECHO1 33
#define ECHO2 32
#define ECHO3 35
#define ECHO4 34
#define ECHO5 39

#define SONIC_TIMEOUT_US  25000 // Timeout pulseIn (~4m)

// Giá trị joystick
#define THROTTLE_MIN        930
#define THROTTLE_MAX        3370
#define THROTTLE_CENTER     1820
#define THROTTLE_RANGE_FWD  (THROTTLE_MAX - THROTTLE_CENTER)
#define THROTTLE_RANGE_REV  (THROTTLE_CENTER - THROTTLE_MIN)

#define STEER_MIN       280
#define STEER_MAX       3730
#define STEER_CENTER    1986

#define DEADZONE        50
#define LINK_TIMEOUT    500

// Tham số điều khiển
#define PWM_MAX_FWD 255
#define PWM_MAX_REV 159
#define EXPO_GAIN 2.5
#define RAMP_UP   4
#define RAMP_DOWN 8

// Cấu trúc gói dữ liệu
typedef struct {
  uint16_t throttle;
  uint16_t steer;
  uint8_t  mode;    // 1 = MANUAL (HIGH), 0 = AUTO (LOW)
} ControlData;

ControlData rxData;

// Giám sát kết nối
unsigned long lastRxTime = 0;
bool linkEstablished = false;

// Xi nhan
unsigned long prevMillis = 0;
bool blinkState = false;

// LED
unsigned long ledTimer = 0;
bool ledState = false;

// Biến điều khiển động cơ
int currentPWM = 0;
int targetPWM  = 0;
bool forward = true;

Servo steeringServo;

// ===== HÀM ĐO CẢM BIẾN SIÊU ÂM =====
float readDistanceCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, SONIC_TIMEOUT_US);
  if (duration == 0) return -1;   // -1 = không có phản hồi
  return duration * 0.0343 / 2.0;
}

// ===== DỪNG ĐỘNG CƠ =====
void stopMotor() {
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  currentPWM = 0;
  targetPWM  = 0;
}

// Callback khi nhận gói ESP-NOW
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(ControlData)) {
    memcpy(&rxData, incomingData, sizeof(rxData));
    lastRxTime = millis();
    linkEstablished = true;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);

  pinMode(LEFT_BLINKER, OUTPUT);
  pinMode(RIGHT_BLINKER, OUTPUT);

  digitalWrite(LEFT_BLINKER, 0);
  digitalWrite(RIGHT_BLINKER, 0);

  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);

  // Cảm biến siêu âm
  int trigPins[] = {TRIG1, TRIG2, TRIG3, TRIG4, TRIG5};
  int echoPins[] = {ECHO1, ECHO2, ECHO3, ECHO4, ECHO5};
  for (int i = 0; i < 5; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(90);

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("RX TorX Ready");
}

void loop() {

  bool linkOK = linkEstablished && ((millis() - lastRxTime) < LINK_TIMEOUT);

  /* ===== FAILSAFE ===== */
  if (!linkOK) {
    stopMotor();
    steeringServo.write(90);
    digitalWrite(LED_PIN, LOW);
  }
  /* ===== AUTO MODE (modePin LOW = 0) ===== */
  else if (rxData.mode == 0) {

    stopMotor();
    steeringServo.write(90);

    // Đọc 5 cảm biến siêu âm
    float d1 = readDistanceCm(TRIG1, ECHO1);
    float d2 = readDistanceCm(TRIG2, ECHO2);
    float d3 = readDistanceCm(TRIG3, ECHO3);
    float d4 = readDistanceCm(TRIG4, ECHO4);
    float d5 = readDistanceCm(TRIG5, ECHO5);

    // In ra Serial Monitor để test
    Serial.print("AUTO | ");
    Serial.print("D1:"); Serial.print(d1 < 0 ? "ERR" : String(d1, 1) + "cm"); Serial.print("  ");
    Serial.print("D2:"); Serial.print(d2 < 0 ? "ERR" : String(d2, 1) + "cm"); Serial.print("  ");
    Serial.print("D3:"); Serial.print(d3 < 0 ? "ERR" : String(d3, 1) + "cm"); Serial.print("  ");
    Serial.print("D4:"); Serial.print(d4 < 0 ? "ERR" : String(d4, 1) + "cm"); Serial.print("  ");
    Serial.print("D5:"); Serial.println(d5 < 0 ? "ERR" : String(d5, 1) + "cm");

    // LED sáng liên tục báo đang ở AUTO
    // digitalWrite(LED_PIN, HIGH);
  }
  /* ===== MANUAL MODE (modePin HIGH = 1) ===== */
  else {

    /* ===== THROTTLE: EXPO + RAMP ===== */
    int16_t err = (int32_t)rxData.throttle - THROTTLE_CENTER;

    if (abs(err) < DEADZONE) {
      targetPWM = 0;
    } else {
      forward = (err > 0);
      int range  = forward ? THROTTLE_RANGE_FWD : THROTTLE_RANGE_REV;
      int pwmMax = forward ? PWM_MAX_FWD : PWM_MAX_REV;

      float norm = (float)(abs(err) - DEADZONE) / (float)(range - DEADZONE);
      norm = constrain(norm, 0.0, 1.0);

      float expo = pow(norm, EXPO_GAIN);
      targetPWM = (int)(expo * pwmMax);
    }

    if (currentPWM < targetPWM)
      currentPWM += RAMP_UP;
    else if (currentPWM > targetPWM)
      currentPWM -= RAMP_DOWN;

    currentPWM = constrain(currentPWM, 0, forward ? PWM_MAX_FWD : PWM_MAX_REV);

    if (currentPWM == 0) {
      analogWrite(RPWM_PIN, 0);
      analogWrite(LPWM_PIN, 0);
    } else if (forward) {
      analogWrite(RPWM_PIN, 0);
      analogWrite(LPWM_PIN, currentPWM);
    } else {
      analogWrite(RPWM_PIN, currentPWM);
      analogWrite(LPWM_PIN, 0);
    }

    /* ===== SERVO LÁI ===== */
    int16_t steerError = (int32_t)rxData.steer - STEER_CENTER;
    int servoAngle;

    if (abs(steerError) < DEADZONE) {
      servoAngle = 90;
    } else {
      servoAngle = map(rxData.steer, STEER_MIN, STEER_MAX, 55, 125);
      servoAngle = constrain(servoAngle, 55, 125);
    }

    steeringServo.write(servoAngle);

    /* ===== BLINKER ===== */
    unsigned long currentMillis = millis();

    if (servoAngle < 65 || servoAngle > 115) {
      if (currentMillis - prevMillis >= BLINK_INTERVAL) {
        prevMillis = currentMillis;
        blinkState = !blinkState;
      }
      if (servoAngle < 65) {
        digitalWrite(RIGHT_BLINKER, LOW);
        digitalWrite(LEFT_BLINKER, blinkState);
      } else {
        digitalWrite(RIGHT_BLINKER, blinkState);
        digitalWrite(LEFT_BLINKER, LOW);
      }
    } else {
      digitalWrite(LEFT_BLINKER, LOW);
      digitalWrite(RIGHT_BLINKER, LOW);
      blinkState = false;
    }

    /* ===== LED ===== */
    if (millis() - ledTimer >= 200) {
      ledTimer = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }

    Serial.print("MANUAL | ");
    Serial.print("Throttle:"); Serial.print(rxData.throttle);
    Serial.print(" PWM:"); Serial.print(currentPWM);
    Serial.print(" Dir:"); Serial.print(forward ? "FWD" : "REV");
    Serial.print(" Servo:"); Serial.println(servoAngle);
  }
}
