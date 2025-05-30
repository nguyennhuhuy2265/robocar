#include <ESP32Servo.h>
#include "BluetoothSerial.h"

#define MOTOR_L_1 12
#define MOTOR_L_2 14
#define MOTOR_R_1 27
#define MOTOR_R_2 26

#define ARM1_SERVO 16
#define ARM2_SERVO 17
#define GRIP_SERVO 5
#define CENTER_SERVO 18
#define BALL_SERVO 4

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig and enable it
#endif

BluetoothSerial SerialBT;

// Servo objects
Servo arm1, arm2, grip, center, ball;

// Servo angles
float arm1_angle = 180;
float arm2_angle = 180;
float grip_angle = 160;
float ungrip_angle = 95;
float center_angle = 90;
float center_step = 3;

// Ball servo state
bool ball_rotating = false;
unsigned long lastPulseTime = 0;
bool pulseState = false;

// PWM timing for servo (in microseconds)
const int SERVO_ROTATE_PULSE = 2000;  // 2ms = rotate clockwise
const int SERVO_STOP_PULSE = 1500;    // 1.5ms = stop
const int SERVO_PERIOD = 20000;       // 20ms period

// States
bool is_grip = false;

// Motor speed
int maxspeed = 255;
String message = "";

void setup() {
  // Attach servos
  arm1.attach(ARM1_SERVO);
  arm2.attach(ARM2_SERVO);
  grip.attach(GRIP_SERVO);
  center.attach(CENTER_SERVO);
  ball.attach(BALL_SERVO);  // Attach ball servo

  // Setup motor pins
  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);

  // Stop motors initially
  digitalWrite(MOTOR_L_1, LOW);
  digitalWrite(MOTOR_L_2, LOW);
  digitalWrite(MOTOR_R_1, LOW);
  digitalWrite(MOTOR_R_2, LOW);

  Serial.begin(9600);
  SerialBT.begin("HOTP"); // Bluetooth device name

  // Set initial servo positions
  arm1.write(arm1_angle);
  arm2.write(arm2_angle);
  grip.write(ungrip_angle);
  center.write(center_angle);
  ball.write(90); // Stop position for continuous servo
}

void loop() {
  // Handle continuous servo rotation with PWM pulses
  if (ball_rotating) {
    handleBallServoPWM();
  }
  
  if (SerialBT.available()) {
    message = SerialBT.readStringUntil('\n');
    Serial.println("Received: " + message);

    // === Movement commands ===
    if (message == "F") forward();
    else if (message == "B") backward();
    else if (message == "L") left();
    else if (message == "R") right();
    else if (message == "S") stop();
    else if (message == "Q") forwardLeft();
    else if (message == "E") forwardRight();
    else if (message == "Z") backwardLeft();
    else if (message == "C") backwardRight();

    // === Servo commands ===
    else if (message == "M") {
      grip.write(is_grip ? ungrip_angle : grip_angle);
      is_grip = !is_grip;
    }
    else if (message == "N") {
      // Toggle ball servo rotation
      ball_rotating = !ball_rotating;
      if (ball_rotating) {
        ball.write(180); // Rotate clockwise (adjust speed as needed)
        Serial.println("Ball servo: Rotating");
      } else {
        ball.write(90);  // Stop
        Serial.println("Ball servo: Stopped");
      }
    }
    else if (message == "X") {
      center_angle = constrain(center_angle + center_step, 0, 180);
      center.write(center_angle);
    }
    else if (message == "Y") {
      center_angle = constrain(center_angle - center_step, 0, 180);
      center.write(center_angle);
    }

    // === Arm angle set via Bluetooth ===
    else if (message.startsWith("J")) {
      arm1_angle = constrain(message.substring(1).toInt(), 0, 180);
      arm1.write(arm1_angle);
    }
    else if (message.startsWith("K")) {
      arm2_angle = constrain(message.substring(1).toInt(), 0, 180);
      arm2.write(arm2_angle);
    }
  }
}

// === Motor control functions ===
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

  // Left motor
  digitalWrite(MOTOR_L_1, speedA > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_L_2, speedA < 0 ? LOW : HIGH);

  // Right motor
  digitalWrite(MOTOR_R_1, speedB > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_R_2, speedB < 0 ? LOW : HIGH);
}

// === Ball Servo PWM Control Functions ===
void startBallServoRotation() {
  ball_rotating = true;
  ball.detach(); // Detach from Servo library to use manual PWM
  pinMode(BALL_SERVO, OUTPUT);
  lastPulseTime = micros();
  pulseState = false;
}

void stopBallServoRotation() {
  ball_rotating = false;
  // Send stop pulse
  digitalWrite(BALL_SERVO, HIGH);
  delayMicroseconds(SERVO_STOP_PULSE);
  digitalWrite(BALL_SERVO, LOW);
  delayMicroseconds(SERVO_PERIOD - SERVO_STOP_PULSE);
  
  // Re-attach to Servo library if needed
  ball.attach(BALL_SERVO);
  ball.write(90); // Stop position
}

void handleBallServoPWM() {
  unsigned long currentTime = micros();
  
  if (!pulseState) {
    // Start of pulse - set HIGH
    if (currentTime - lastPulseTime >= SERVO_PERIOD) {
      digitalWrite(BALL_SERVO, HIGH);
      lastPulseTime = currentTime;
      pulseState = true;
    }
  } else {
    // End of pulse - set LOW
    if (currentTime - lastPulseTime >= SERVO_ROTATE_PULSE) {
      digitalWrite(BALL_SERVO, LOW);
      pulseState = false;
    }
  }
}