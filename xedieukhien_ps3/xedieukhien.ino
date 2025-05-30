#include <ESP32Servo.h>
#include <Ps3Controller.h>

// === Motor pins ===
#define MOTOR_L_1 12
#define MOTOR_L_2 14
#define MOTOR_R_1 27
#define MOTOR_R_2 26

// === Servo pins ===
#define ARM1_SERVO 16
#define ARM2_SERVO 17
#define GRIP_SERVO 5
#define CENTER_SERVO 18
#define SELECT_SERVO 19   // Servo thường (nút L2)
#define BALL_SERVO 4      // Servo 360 độ (nút Select)

// Servo objects
Servo arm1, arm2, grip, center, selectServo, ball;

// Servo angles
float arm1_angle = 180;
float arm2_angle = 180;
float grip_angle = 160;
float ungrip_angle = 110;
float grip_angle1 = 160;
float ungrip_angle1 = 80;
float center_angle = 90;
float center_step = 3;
float select_angle = 180;
float unselect_angle = 90;

// States
bool is_grip = false;
bool is_grip1 = false;
bool is_ball_rotating = false;
bool is_select_on = false;

// Motor speed
int maxspeed = 255;

// === Movement control ===
void stop() { setMotor(0, 0); }
void forward() { setMotor(maxspeed, maxspeed); }
void backward() { setMotor(-maxspeed, -maxspeed); }
void left() { setMotor(-maxspeed, maxspeed); }
void right() { setMotor(maxspeed, -maxspeed); }
void forwardLeft() { setMotor(0, maxspeed); }
void forwardRight() { setMotor(maxspeed, 0); }
void backwardLeft() { setMotor(0, -maxspeed); }
void backwardRight() { setMotor(-maxspeed, 0); }

void setMotor(int speedA, int speedB) {
  speedA = -speedA;
  speedB = -speedB;

  digitalWrite(MOTOR_L_1, speedA > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_L_2, speedA < 0 ? HIGH : LOW);
  digitalWrite(MOTOR_R_1, speedB > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_R_2, speedB < 0 ? HIGH : LOW);
}

// === PS3 callback ===
void notify() {
  int ly = Ps3.data.analog.stick.ly;
  int lx = Ps3.data.analog.stick.lx;

  // === Di chuyển robot ===
  if (ly < -10 && lx < -10) forwardLeft();
  else if (ly < -10 && lx > 10) forwardRight();
  else if (ly > 10 && lx < -10) backwardLeft();
  else if (ly > 10 && lx > 10) backwardRight();
  else if (ly < -10) forward();
  else if (ly > 10) backward();
  else if (lx < -10) left();
  else if (lx > 10) right();
  else if (Ps3.data.button.up) forward();
  else if (Ps3.data.button.down) backward();
  else if (Ps3.data.button.left) left();
  else if (Ps3.data.button.right) right();
  else stop();

  // === Arm1: Triangle / Cross ===
  if (Ps3.data.button.triangle) {
    arm1_angle = constrain(arm1_angle + 3, 0, 180);
    arm1.write(arm1_angle);
  } else if (Ps3.data.button.cross) {
    arm1_angle = constrain(arm1_angle - 3, 0, 180);
    arm1.write(arm1_angle);
  }

  // === Arm2: Square / Circle ===
  if (Ps3.data.button.square) {
    arm2_angle = constrain(arm2_angle + 3, 0, 180);
    arm2.write(arm2_angle);
  } else if (Ps3.data.button.circle) {
    arm2_angle = constrain(arm2_angle - 3, 0, 180);
    arm2.write(arm2_angle);
  }

  // === Center: L1 / R1 ===
  if (Ps3.data.button.l2) {
    center_angle = constrain(center_angle + center_step, 0, 180);
    center.write(center_angle);
  }
  if (Ps3.data.button.r2) {
    center_angle = constrain(center_angle - center_step, 0, 180);
    center.write(center_angle);
  }

  // === Bắn bóng: SELECT (servo 360 độ ở chân 4) ===
  if (Ps3.data.button.select) {
    is_ball_rotating = !is_ball_rotating;
    ball.writeMicroseconds(is_ball_rotating ? 2000 : 1500);  // 2000 quay, 1500 dừng
    delay(200);
  }

  // === Gắp / Thả: R2 ===


  // === Servo thường chân 19: L2 ===
  if (Ps3.data.button.l1) {
    selectServo.write(is_grip1 ? ungrip_angle1 : grip_angle1);
      is_grip1 = !is_grip1;
  }

   if (Ps3.data.button.r1) {
    grip.write(is_grip ? ungrip_angle : grip_angle);
      is_grip = !is_grip;
  }
}

void onConnect() {
  Serial.println("Đã kết nối tay cầm PS3");
}

void setup() {
  Serial.begin(115200);

  // Setup motor pins
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);
  stop();

  // Attach servos
  arm1.attach(ARM1_SERVO);
  arm2.attach(ARM2_SERVO);
  grip.attach(GRIP_SERVO);
  center.attach(CENTER_SERVO);
  selectServo.attach(SELECT_SERVO);  // chân 19
  ball.attach(BALL_SERVO);           // chân 4 (servo 360)

  // Initial positions
  arm1.write(arm1_angle);
  arm2.write(arm2_angle);
  grip.write(ungrip_angle);
  center.write(center_angle);
  selectServo.write(unselect_angle);
  ball.writeMicroseconds(1500); // stop servo 360

  // PS3 connection
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("20:00:00:00:87:91"); // ← thay bằng địa chỉ MAC tay cầm của bạn nếu khác
  Serial.println("Đang chờ kết nối tay cầm PS3...");
}

void loop() {
  // notify() xử lý mọi thao tác
}
