#include <Servo.h>
Servo s; // MG996R 서보 모터 사용

void setup() {
  Serial.begin(115200);
  Serial.println("=== 부팅됨 ===");
  s.attach(9, 600, 2400);      // 안전 범위
  s.write(45);                 // 중간에서 시작
}

void loop() {
  if (Serial.available()) {
    int a = Serial.parseInt();
    while (Serial.available()) Serial.read();   // 줄바꿈 등 비우기
    if (a >= 0 && a <= 180) {
      s.write(a);
      Serial.print("이동: "); Serial.println(a);
    }
  }
}