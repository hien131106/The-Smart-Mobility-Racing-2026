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

// ===== CẢM BIẾN SIÊU ÂM (3 CON) =====
#define NUM_SENSORS 3
const int trigPins[NUM_SENSORS] = {23, 22, 21};  // FL, FM, FR
const int echoPins[NUM_SENSORS] = {33, 32, 35};  // FL, FM, FR
int distances[NUM_SENSORS];

// ===== CẤU TRÚC GÓI ESP-NOW =====
typedef struct {
  uint16_t throttle;
  uint16_t steer;
  uint8_t  mode;    // 1 = MANUAL, 0 = AUTO
} ControlData;

ControlData rxData;
unsigned long lastRecv = 0;
bool linkEstablished   = false;

// ===== THAM SỐ MANUAL =====
#define THROTTLE_CENTER     1820
#define THROTTLE_RANGE_FWD  1550
#define THROTTLE_RANGE_REV  890
#define STEER_MIN_ADC       280
#define STEER_MAX_ADC       3730
#define STEER_CENTER_ADC    1986
#define DEADZONE            50
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

// ===== ĐỌC CẢM BIẾN =====
void readUltrasonics() {
  for (int i = 0; i < NUM_SENSORS; i++) {
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

// ===== MOTOR =====
void stopMotor() {
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  currentPWM = 0; targetPWM = 0;
}

// ===== CALLBACK =====
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(ControlData)) {
    memcpy(&rxData, incomingData, sizeof(rxData));
    lastRecv = millis();
    linkEstablished = true;
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN,       OUTPUT);
  pinMode(RPWM_PIN,      OUTPUT);
  pinMode(LPWM_PIN,      OUTPUT);
  pinMode(LEFT_BLINKER,  OUTPUT);
  pinMode(RIGHT_BLINKER, OUTPUT);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(90);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("RX TorX Ready");
}

// ===== LOOP =====
void loop() {
  bool linkOK = linkEstablished && (millis() - lastRecv < LINK_TIMEOUT);

  /* ===== FAILSAFE ===== */
  if (!linkOK) {
    stopMotor();
    steeringServo.write(90);
    digitalWrite(LEFT_BLINKER,  LOW);
    digitalWrite(RIGHT_BLINKER, LOW);
    digitalWrite(LED_PIN, LOW);
    return;
  }

  /* ===== AUTO MODE ===== */
  if (rxData.mode == 0) {
    readUltrasonics();

    Serial.print("FL:"); Serial.print(distances[0]);
    Serial.print("cm  FM:"); Serial.print(distances[1]);
    Serial.print("cm  FR:"); Serial.print(distances[2]);
    Serial.println("cm");

    digitalWrite(LED_PIN, HIGH);
    return;
  }

  /* ===== MANUAL MODE ===== */

  // Throttle expo + ramp
  int16_t err = (int32_t)rxData.throttle - THROTTLE_CENTER;
  if (abs(err) < DEADZONE) {
    targetPWM = 0;
  } else {
    forward    = (err > 0);
    int range  = forward ? THROTTLE_RANGE_FWD : THROTTLE_RANGE_REV;
    int pwmMax = forward ? PWM_MAX_FWD : PWM_MAX_REV;
    float norm = constrain((float)(abs(err) - DEADZONE) / (float)(range - DEADZONE), 0.0f, 1.0f);
    targetPWM  = (int)(pow(norm, EXPO_GAIN) * pwmMax);
  }
  if      (currentPWM < targetPWM) currentPWM += RAMP_UP;
  else if (currentPWM > targetPWM) currentPWM -= RAMP_DOWN;
  currentPWM = constrain(currentPWM, 0, forward ? PWM_MAX_FWD : PWM_MAX_REV);

  if (currentPWM == 0) {
    analogWrite(RPWM_PIN, 0); analogWrite(LPWM_PIN, 0);
  } else if (forward) {
    analogWrite(RPWM_PIN, 0); analogWrite(LPWM_PIN, currentPWM);
  } else {
    analogWrite(RPWM_PIN, currentPWM); analogWrite(LPWM_PIN, 0);
  }

  // Servo lái
  int servoAngle;
  if (abs((int32_t)rxData.steer - STEER_CENTER_ADC) < DEADZONE) {
    servoAngle = 90;
  } else {
    servoAngle = constrain(map(rxData.steer, STEER_MIN_ADC, STEER_MAX_ADC, 55, 125), 55, 125);
  }
  steeringServo.write(servoAngle);

  // Xi nhan
  unsigned long now = millis();
  if (servoAngle < 65 || servoAngle > 115) {
    if (now - prevBlink >= BLINK_INTERVAL) { prevBlink = now; blinkStateManual = !blinkStateManual; }
    if (servoAngle < 65) {
      digitalWrite(LEFT_BLINKER, blinkStateManual); digitalWrite(RIGHT_BLINKER, LOW);
    } else {
      digitalWrite(RIGHT_BLINKER, blinkStateManual); digitalWrite(LEFT_BLINKER, LOW);
    }
  } else {
    digitalWrite(LEFT_BLINKER, LOW); digitalWrite(RIGHT_BLINKER, LOW);
    blinkStateManual = false;
  }

  // LED nhấp nháy
  if (now - ledTimer >= 200) { ledTimer = now; ledState = !ledState; digitalWrite(LED_PIN, ledState); }

  Serial.print("MANUAL | T:"); Serial.print(rxData.throttle);
  Serial.print(" PWM:"); Serial.print(currentPWM);
  Serial.print(" Dir:"); Serial.print(forward ? "FWD" : "REV");
  Serial.print(" Servo:"); Serial.println(servoAngle);
}
