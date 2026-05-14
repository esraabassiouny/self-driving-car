#include <Servo.h>

// =====================================================
// Ultrasonic pins
// =====================================================
int trigPin = 11;
int echoPin = 12;

// =====================================================
// Motor pins
// =====================================================
int IN1 = 4;
int IN2 = 10;
int IN3 = 6;
int IN4 = 7;

int enA = 3;
int enB = 5;

// =====================================================
// Motor speeds
// =====================================================
int left_speed = 0;
int right_speed = 0;

// =====================================================
long duration;
float distance;

// =====================================================
// Read ultrasonic distance
// =====================================================
float readDistance() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000);

  // No echo
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

  Serial.println("ARDUINO_READY");
}

// =====================================================
void loop() {

  // ===================================================
  // Receive motor command from Raspberry Pi
  // Format:
  // L150R150
  // ===================================================
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

  // ===================================================
  // Motor Direction
  // ===================================================

  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Right motor forward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Apply speed
  analogWrite(enA, left_speed);
  analogWrite(enB, right_speed);

  // ===================================================
  // Read ultrasonic
  // ===================================================
  float dist = readDistance();

  // ===================================================
  // Send distance to Raspberry Pi
  // ===================================================
  Serial.print("DISTANCE: ");
  Serial.print(dist);
  Serial.println(" cm");

  delay(60);
}