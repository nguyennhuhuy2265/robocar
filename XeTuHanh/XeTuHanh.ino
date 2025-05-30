// ==== KHAI BÁO CÁC CHÂN ==== //
#define IR_LEFT_OUT    12
#define IR_LEFT_MID    11
#define IR_CENTER      10
#define IR_RIGHT_MID   9
#define IR_RIGHT_OUT   8
#define CHECHK_BALL    13

// Cảm biến khoảng cách HC-SR04
#define TRIG_PIN       A0
#define ECHO_PIN       A1

// Điều khiển động cơ
#define IN1 2  // Trái
#define IN2 4
#define IN3 6  // Phải
#define IN4 7
#define ENA 3  // PWM cho động cơ trái
#define ENB 5  // PWM cho động cơ phải

enum TurnDirection { NONE = 0, LEFT = -1, RIGHT = 1 };
TurnDirection lastTurnDirection = NONE;

void setup() {
  // Cấu hình output motor
  int motorPins[] = {IN1, IN2, IN3, IN4, ENA, ENB};
  for (int i = 0; i < 6; i++) pinMode(motorPins[i], OUTPUT);

  // Cấu hình cảm biến dò line
  int sensorPins[] = {
    IR_LEFT_OUT, IR_LEFT_MID, IR_CENTER, IR_RIGHT_MID, IR_RIGHT_OUT
  };
  for (int i = 0; i < 5; i++) pinMode(sensorPins[i], INPUT);

  // Cấu hình HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(9600);
}

bool ballChecked = false;
bool atFinish = false;  // đã đến đích


void loop() {
  int sensors[5];
  sensors[0] = digitalRead(IR_LEFT_OUT);
  sensors[1] = digitalRead(IR_LEFT_MID);
  sensors[2] = digitalRead(IR_CENTER);
  sensors[3] = digitalRead(IR_RIGHT_MID);
  sensors[4] = digitalRead(IR_RIGHT_OUT);

  float distance = readDistanceCM();

  debugSensors(sensors, distance);

  // ==== DỪNG KHI ĐẾN ĐÍCH ==== //
  if (!atFinish && sensors[0] && sensors[1] && sensors[2] && sensors[3] && sensors[4]) {
    stopMotors();
    lastTurnDirection = NONE;
    atFinish = true;  // Đánh dấu đã đến đích
    Serial.println("ĐÃ ĐẾN ĐÍCH - DỪNG HẲN CHỜ BÓNG");
    return;
  }

  // ==== ĐANG Ở TRẠNG THÁI DỪNG CHỜ BÓNG ==== //
  if (atFinish) {
    if (!ballChecked && digitalRead(CHECHK_BALL) == 0) {
      Serial.println("NHẬN BÓNG TẠI ĐÍCH - CHẠY THẲNG 1 LẦN");
      delay(1500);
      forward(110);
      delay(400);
      stopMotors();
      delay(200);
      ballChecked = true;
      atFinish = false;  // Thoát trạng thái "đến đích", tiếp tục dò line
    }
    return; // Không làm gì thêm nếu chưa có bóng
  }

  // ==== Ưu tiên leo cầu thang nếu phát hiện ==== //
  if (distance < 3.2) {
    Serial.println("Phát hiện bậc thang - bắt đầu leo...");
    stopMotors();
    delay(400);

    while (readDistanceCM() < 7) {
      forward(210);
      delay(900);
    }

    stopMotors();
    delay(200);
    return;
  }

  // ==== Điều hướng chính (dò line) ==== //
  bool center = sensors[2];
  bool leftMid = sensors[1], rightMid = sensors[3];
  bool leftOut = sensors[0], rightOut = sensors[4];

  if (center && leftMid && leftOut) {
    turnLeft(200); lastTurnDirection = LEFT; Serial.println("Trái mạnh");
  } else if (center && rightMid && rightOut) {
    turnRight(200); lastTurnDirection = RIGHT; Serial.println("Phải mạnh");
  } else if (center || (center && (leftMid || rightMid))) {
    forward(90); lastTurnDirection = NONE; Serial.println("Thẳng");
  } else if (leftOut) {
    turnLeft(180); lastTurnDirection = LEFT; Serial.println("Trái mạnh");
  } else if (rightOut) {
    turnRight(180); lastTurnDirection = RIGHT; Serial.println("Phải mạnh");
  } else if (leftMid) {
    turnLeft(150); lastTurnDirection = LEFT; Serial.println("Trái nhẹ");
  } else if (rightMid) {
    turnRight(150); lastTurnDirection = RIGHT; Serial.println("Phải nhẹ");
  } else {
    Serial.println("Mất line!");
    switch (lastTurnDirection) {
      case LEFT:
        turnLeft(200); delay(300); stopMotors(); Serial.println("Tìm trái rồi dừng"); break;
      case RIGHT:
        turnRight(200); delay(300); stopMotors(); Serial.println("Tìm phải rồi dừng"); break;
      default:
        stopMotors(); Serial.println("Không rõ hướng, dừng"); break;
    }
  }
}



// ===== HÀM ĐIỀU KHIỂN ===== //

void forward(int speed) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void turnLeft(int speed) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void turnRight(int speed) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

// ===== ĐỌC KHOẢNG CÁCH HC-SR04 ===== //
float  readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;
  return distance;
}

// ===== DEBUG ===== //
void debugSensors(int sensors[], long distance) {

  Serial.print(" "); Serial.print(sensors[0]);
  Serial.print("\ "); Serial.print(sensors[1]);
  Serial.print("\ "); Serial.print(sensors[2]);
  Serial.print("\ "); Serial.print(sensors[3]);
  Serial.print("\ "); Serial.print(sensors[4]);
  Serial.print("Check bóng: "); Serial.print(digitalRead(CHECHK_BALL));
  Serial.print("\ Khoảng cách:"); Serial.print(distance); Serial.println(" cm");
}
