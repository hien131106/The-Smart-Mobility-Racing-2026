#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <math.h>

// Khai báo chân
#define LED_PIN    2
#define RPWM_PIN   25   // BTS7960 tiến
#define LPWM_PIN   26   // BTS7960 lùi
#define SERVO_PIN  27   // Servo lái

// Giá trị joystick thực đo được
#define THROTTLE_MIN        930
#define THROTTLE_MAX        3370
#define THROTTLE_CENTER     1820
#define THROTTLE_RANGE_FWD  (THROTTLE_MAX - THROTTLE_CENTER)   // 1550
#define THROTTLE_RANGE_REV  (THROTTLE_CENTER - THROTTLE_MIN)   // 890

#define STEER_MIN       280
#define STEER_MAX       3730
#define STEER_CENTER    1986       

#define DEADZONE        50        // Vùng chết chống rung
#define LINK_TIMEOUT    500       // Timeout mất sóng (ms)

// Tham số điều khiển
#define PWM_MAX_FWD 255           // Tiến: full 12V
#define PWM_MAX_REV 159           // Lùi/phanh: giới hạn ~7.5V (7.5/12 * 255)
#define EXPO_GAIN 2.5             // Độ cong hàm expo
#define RAMP_UP   4               // Tốc độ tăng ga
#define RAMP_DOWN 8               // Tốc độ giảm ga (phanh)

// Cấu trúc gói dữ liệu
typedef struct {
  uint16_t throttle;
  uint16_t steer;
} ControlData;

ControlData rxData;

// Giám sát kết nối
unsigned long lastRxTime = 0;
bool linkEstablished = false;

// LED
unsigned long ledTimer = 0;
bool ledState = false;

// Biến điều khiển động cơ
int currentPWM = 0;   // PWM thực tế xuất ra
int targetPWM  = 0;   // PWM mong muốn từ joystick
bool forward = true;  // Hướng quay

Servo steeringServo;

// Callback khi nhận gói ESP-NOW
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(ControlData)) {
    memcpy(&rxData, incomingData, sizeof(rxData));
    lastRxTime = millis();        // Cập nhật thời gian nhận
    linkEstablished = true;      // Đánh dấu đã từng kết nối
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);

  analogWrite(RPWM_PIN, 0);      // Dừng động cơ lúc khởi động
  analogWrite(LPWM_PIN, 0);

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(90);       // Lái về giữa

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {

  // Kiểm tra còn kết nối hay không
  bool linkOK = linkEstablished && ((millis() - lastRxTime) < LINK_TIMEOUT);

  /* ===== FAILSAFE ===== */
  if (!linkOK) {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
    steeringServo.write(90);
    digitalWrite(LED_PIN, LOW);

    currentPWM = 0;
    targetPWM  = 0;
  }
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

      float expo = pow(norm, EXPO_GAIN);   // Hàm mũ làm mềm vùng thấp
      targetPWM = (int)(expo * pwmMax);
    }

    // Ramp tăng/giảm mượt
    if (currentPWM < targetPWM)
      currentPWM += RAMP_UP;
    else if (currentPWM > targetPWM)
      currentPWM -= RAMP_DOWN;

    currentPWM = constrain(currentPWM, 0, forward ? PWM_MAX_FWD : PWM_MAX_REV);

    // Xuất ra BTS7960
    if (currentPWM == 0) {
      analogWrite(RPWM_PIN, 0);
      analogWrite(LPWM_PIN, 0);
    } 
    else if (forward) {
      analogWrite(RPWM_PIN, currentPWM);
      analogWrite(LPWM_PIN, 0);
    } 
    else {
      analogWrite(RPWM_PIN, 0);
      analogWrite(LPWM_PIN, currentPWM);
    }

    /* ===== SERVO LÁI ===== */
    int16_t steerError = (int32_t)rxData.steer - STEER_CENTER;
    int servoAngle;

    if (abs(steerError) < DEADZONE) {
      servoAngle = 90;
    } else {
      servoAngle = map(rxData.steer,
                       STEER_MIN, STEER_MAX,
                       55, 125);
      servoAngle = constrain(servoAngle, 55, 125);
    }

    steeringServo.write(servoAngle);

    /* ===== LED ===== */
    if (millis() - ledTimer >= 200) {
      ledTimer = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  }

  /* ===== DEBUG ===== */
  Serial.print("Link: "); Serial.print(linkOK);
  Serial.print(" Throttle: "); Serial.print(rxData.throttle);
  Serial.print(" TargetPWM: "); Serial.print(targetPWM);
  Serial.print(" OutPWM: "); Serial.print(currentPWM);
  Serial.print(" Dir: "); Serial.println(forward ? "FWD" : "REV");
}
