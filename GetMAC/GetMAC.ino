#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);  // Đặt ESP ở chế độ Station (không tạo Wi-Fi AP)

  Serial.println("=== ĐỊA CHỈ MAC ESP32 ===");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());  // In ra địa chỉ MAC
  pinMode(2, OUTPUT);
}

void loop() {
  // Không cần làm gì trong loop
  digitalWrite (2, HIGH);
  Serial.println("=== ĐỊA CHỈ MAC ESP32 ===");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());  // In ra địa chỉ MAC
  delay(1000);
  digitalWrite (2, LOW);
  delay(1000);
  
}
