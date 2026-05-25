#include <Servo.h>

Servo myServo;

// --------------------
// Ultrasonic pins
// --------------------
int trigPin = 11;
int echoPin = 12;

// --------------------
// Motor pins
// --------------------
int IN1 = 4;
int IN2 = 10;
int IN3 = 6;
int IN4 = 7;

int enA = 3;
int enB = 5;

int left_speed = 0;
int right_speed = 0;

// --------------------
// Radar sweep
// --------------------
int servoAngle = 20;
int servoStep = 5;

bool objectDetected = false;

long duration;
float distance;

// =====================================================
// Read ultrasonic
// =====================================================
float readDistance() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    return 999;
  }

  distance = duration * 0.034 / 2;

  return distance;
}

// =====================================================
void setup() {

  Serial.begin(9600);

  // Ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Motors
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  // Stop motors initially
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  // Servo
  myServo.attach(9);

  myServo.write(90);

  Serial.println("SYSTEM_READY");
}

// =====================================================
void loop() {

  // =================================================
  // RECEIVE MOTOR COMMANDS FROM RASPBERRY PI
  // Format:
  // L150R150
  // =================================================
  if (Serial.available()) {

    String cmd = Serial.readStringUntil('\n');

    cmd.trim();

    int lIndex = cmd.indexOf('L');
    int rIndex = cmd.indexOf('R');

    if (lIndex != -1 && rIndex != -1) {

      left_speed =
        cmd.substring(lIndex + 1, rIndex).toInt();

      right_speed =
        cmd.substring(rIndex + 1).toInt();
    }
  }

  // =================================================
  // MOTOR CONTROL
  // =================================================
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(enA, left_speed);
  analogWrite(enB, right_speed);

  // =================================================
  // RADAR SWEEP
  // =================================================
  myServo.write(servoAngle);

  delay(25);

  float dist = readDistance();

  // =================================================
  // SEND DISTANCE TO RASPBERRY PI
  // =================================================
  Serial.print("DIST:");
  Serial.println(dist);

  // =================================================
  // OBJECT DETECTION
  // =================================================
  if (dist < 80) {

    objectDetected = true;

  } else {

    objectDetected = false;
  }

  // =================================================
  // MOVE SERVO ONLY IF NO OBJECT
  // =================================================
  if (!objectDetected) {

    servoAngle += servoStep;

    // Reverse direction
    if (servoAngle >= 140 || servoAngle <= 20) {
      servoStep = -servoStep;
    }
  }
}