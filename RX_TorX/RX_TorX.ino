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
#define FIRST_WARNING   25
#define SECOND_WARNING  25
#define THIRD_WARNING   35

// ===== TỐC ĐỘ AUTO =====
#define AUTO_SPD       15   // Tiến thẳng
#define AUTO_SPD_SLOW  15   // Tiến khi tránh
#define AUTO_SPD_BACK  15   // Lùi

// ===== GÓC SERVO AUTO (constrain 65-125) =====
#define STEER_C         90
#define STEER_L_MEDIUM  65   // 90 - 40 → capped
#define STEER_L_HARD    65   // 90 - 50 → capped
#define STEER_R_MEDIUM 125   // 90 + 40 → capped
#define STEER_R_HARD   125   // 90 + 50 → capped

// ===== STATE MACHINE LÙI =====
enum AutoState { NAVIGATE, BACKING };
AutoState autoState      = NAVIGATE;
unsigned long backStart  = 0;
unsigned long backDur    = 0;   // thời gian lùi, set riêng mỗi maneuver

// ===== BLINK AUTO =====
unsigned long lastBlinkAuto = 0;
bool ledStateAuto           = false;

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
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  currentPWM = 0; targetPWM = 0;
}
void driveForward(uint8_t spd)  { analogWrite(RPWM_PIN, 0);   analogWrite(LPWM_PIN, spd); }
void driveBackward(uint8_t spd) { analogWrite(RPWM_PIN, spd); analogWrite(LPWM_PIN, 0);   }

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

  // Giữa quá gần → lùi thẳng (750ms)
  if (fm <= FIRST_WARNING)
  {
    steeringServo.write(STEER_C);
    driveBackward(AUTO_SPD_BACK);
    notBlink();
    autoState = BACKING; 
    backStart = millis(); 
    backDur = 1000;
  }
  // Góc trái rất gần → lùi nghiêng trái (750ms)
  else if (fl <= FIRST_WARNING)
  {
    steeringServo.write(STEER_L_MEDIUM);
    driveBackward(AUTO_SPD_BACK);
    blinkLeft();
    autoState = BACKING; 
    backStart = millis(); 
    backDur = 1000;
  }
  // Góc phải rất gần → lùi nghiêng phải (500ms)
  else if (fr <= FIRST_WARNING)
  {
    steeringServo.write(STEER_R_MEDIUM);
    driveBackward(AUTO_SPD_BACK);
    blinkRight();
    autoState = BACKING; 
    backStart = millis(); 
    backDur = 1000;
  }
  // Trái + giữa gần → quẹo phải mạnh
  else if (fl <= SECOND_WARNING && fm <= SECOND_WARNING)
  {
    steeringServo.write(STEER_R_HARD);
    driveForward(AUTO_SPD_SLOW);
    blinkRight();
  }
  // Phải + giữa gần → quẹo trái mạnh
  else if (fr <= SECOND_WARNING && fm <= SECOND_WARNING)
  {
    steeringServo.write(STEER_L_HARD);
    driveForward(AUTO_SPD_SLOW);
    blinkLeft();
  }
  // Trái cảnh báo → nghiêng phải vừa
  else if (fl > FIRST_WARNING && fl < THIRD_WARNING)
  {
    steeringServo.write(STEER_R_MEDIUM);
    driveForward(AUTO_SPD_SLOW);
    blinkRight();
  }
  // Phải cảnh báo → nghiêng trái vừa
  else if (fr > FIRST_WARNING && fr < THIRD_WARNING)
  {
    steeringServo.write(STEER_L_MEDIUM);
    driveForward(AUTO_SPD_SLOW);
    blinkLeft();
  }
  // Cạnh trái gần hơn phải → nghiêng phải
  else if (l <= FIRST_WARNING && l < r)
  {
    steeringServo.write(STEER_R_MEDIUM);
    driveForward(AUTO_SPD_SLOW);
    blinkRight();
  }
  // Cạnh phải gần hơn trái → nghiêng trái
  else if (r <= FIRST_WARNING && r < l)
  {
    steeringServo.write(STEER_L_MEDIUM);
    driveForward(AUTO_SPD_SLOW);
    blinkLeft();
  }
  // Đường thông → tiến thẳng
  else
  {
    steeringServo.write(STEER_C);
    driveForward(AUTO_SPD);
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
  }
}

// ===== SETUP =====
void setup() 
{
  Serial.begin(115200);

  pinMode(LED_PIN,       OUTPUT);
  pinMode(RPWM_PIN,      OUTPUT);
  pinMode(LPWM_PIN,      OUTPUT);
  pinMode(LEFT_BLINKER,  OUTPUT);
  pinMode(RIGHT_BLINKER, OUTPUT);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);

  for (int i = 0; i < NUM_SENSORS; i++) 
  {
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

  /* ===== AUTO MODE ===== */
  if (rxData.mode == 0) 
  {
    readUltrasonics();

    Serial.print("S1:"); Serial.print(distances[0]);
    Serial.print("cm  S2:"); Serial.print(distances[1]);
    Serial.print("cm  S3:"); Serial.print(distances[2]);
    Serial.print("cm  S4:"); Serial.print(distances[3]);
    Serial.print("cm  S5:"); Serial.print(distances[4]);
    Serial.println("cm");

    autoNavigate();
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  /* ===== MANUAL MODE ===== */

  // Throttle expo + ramp
  int16_t err = (int32_t)rxData.throttle - THROTTLE_CENTER;
  if (abs(err) < DEADZONE) 
  {
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

  if (currentPWM == 0) 
  {
    analogWrite(RPWM_PIN, 0); analogWrite(LPWM_PIN, 0);
  } else if (forward) {
    analogWrite(RPWM_PIN, 0); analogWrite(LPWM_PIN, currentPWM);
  } else {
    analogWrite(RPWM_PIN, currentPWM); analogWrite(LPWM_PIN, 0);
  }

  // Servo lái
  int servoAngle;
  if (abs((int32_t)rxData.steer - STEER_CENTER_ADC) < DEADZONE) 
  {
    servoAngle = 90;
  } else {
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

  Serial.print("MANUAL | T:"); Serial.print(rxData.throttle);
  Serial.print(" PWM:"); Serial.print(currentPWM);
  Serial.print(" Dir:"); Serial.print(forward ? "FWD" : "REV");
  Serial.print(" Servo:"); Serial.println(servoAngle);
}
