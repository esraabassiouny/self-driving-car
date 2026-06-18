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
bool servoSweepEnabled = true;
long duration;
float distance;
float minSweepDist = 999.0;

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
// MOVE FORWARD
// =====================================================
void forward(int leftSpeed, int rightSpeed) {

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(enA, leftSpeed);
  analogWrite(enB, rightSpeed);
}

// =====================================================
// STOP CAR
// =====================================================
void stopCar() {

  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

// =====================================================
// SHARP LEFT TURN
// =====================================================
void sharpLeft(int t) {

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Left motor stopped
  analogWrite(enA, 0);

  // Right motor full speed
  analogWrite(enB, 255);

  delay(t);
}

// =====================================================
// U-TURN
// =====================================================
void uTurnLeft() {

  Serial.println("START_U_TURN");

  stopCar();
  delay(300);

  forward(200, 200);
  delay(500);

  stopCar();
  delay(300);

  // Adjust this value experimentally
  sharpLeft(4400);

  stopCar();
  delay(300);

  Serial.println("END_U_TURN");
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

  stopCar();

  // Servo
  myServo.attach(9);
  myServo.write(90);

  Serial.println("SYSTEM_READY");
}

// =====================================================
void loop() {

  // =================================================
  // RECEIVE COMMANDS FROM RASPBERRY PI
  // =================================================
  if (Serial.available()) {

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "START_SWEEP") {

    servoSweepEnabled = true;
  }
  else if (cmd == "STOP_SWEEP") {

      servoSweepEnabled = false;
      myServo.write(90);   // optional: center servo
      Serial.println("SWEEP_STOPPED");
  }
  else if (cmd == "UTURN_LEFT") {

      uTurnLeft();
  }


    // ---------------------------------
    // NORMAL MOTOR COMMAND
    // FORMAT: L150R150
    // ---------------------------------
    else {

      int lIndex = cmd.indexOf('L');
      int rIndex = cmd.indexOf('R');

      if (lIndex != -1 && rIndex != -1) {

        left_speed =
          cmd.substring(lIndex + 1, rIndex).toInt();

        right_speed =
          cmd.substring(rIndex + 1).toInt();
      }
    }
  }

  // =================================================
  // MOTOR CONTROL
  // =================================================
  forward(left_speed, right_speed);

  // =================================================
  // RADAR SWEEP
  // =================================================
  if (servoSweepEnabled) {

    myServo.write(servoAngle);

    delay(25);

    float dist = readDistance();

    if (dist < 80) {
        objectDetected = true;
        Serial.print("DIST:");
        Serial.println(dist);
        minSweepDist = 999.0;
    } else {
        objectDetected = false;
        if (dist < minSweepDist) {
            minSweepDist = dist;
        }
    }

    if (!objectDetected) {

        servoAngle += servoStep;

        if (servoAngle >= 140 || servoAngle <= 20) {
            Serial.print("DIST:");
            Serial.println(minSweepDist);
            minSweepDist = 999.0;
            servoStep = -servoStep;
        }
    }
}
}



// Send Min Dist Only

// #include <Servo.h>

// Servo myServo;

// // --------------------
// // Ultrasonic pins
// // --------------------
// int trigPin = 11;
// int echoPin = 12;

// // --------------------
// // Motor pins
// // --------------------
// int IN1 = 4;
// int IN2 = 10;
// int IN3 = 6;
// int IN4 = 7;

// int enA = 3;
// int enB = 5;

// // --------------------
// // Servo sweep control
// // --------------------
// bool sweepActive = false;

// int servoAngle = 20;
// int servoStep = 5;

// // --------------------
// // Distance variables
// // --------------------
// long duration;
// float distance;

// // =====================================================
// // Read ultrasonic
// // =====================================================
// float readDistance() {

//   digitalWrite(trigPin, LOW);
//   delayMicroseconds(2);

//   digitalWrite(trigPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin, LOW);

//   duration = pulseIn(echoPin, HIGH, 30000);

//   if (duration == 0) {
//     return 999;
//   }

//   distance = duration * 0.034 / 2;
//   return distance;
// }

// // =====================================================
// // SETUP
// // =====================================================
// void setup() {

//   Serial.begin(9600);

//   pinMode(trigPin, OUTPUT);
//   pinMode(echoPin, INPUT);

//   pinMode(IN1, OUTPUT);
//   pinMode(IN2, OUTPUT);
//   pinMode(IN3, OUTPUT);
//   pinMode(IN4, OUTPUT);

//   pinMode(enA, OUTPUT);
//   pinMode(enB, OUTPUT);

//   myServo.attach(9);
//   myServo.write(90);

//   Serial.println("SYSTEM_READY");
// }

// // =====================================================
// // LOOP
// // =====================================================
// void loop() {

//   // ----------------------------
//   // READ COMMAND FROM PYTHON
//   // ----------------------------
//   if (Serial.available()) {

//     String cmd = Serial.readStringUntil('\n');
//     cmd.trim();

//     if (cmd == "START_SWEEP") {
//       sweepActive = true;
//     }
//   }

//   // ----------------------------
//   // SERVO SWEEP (AUTONOMOUS)
//   // ----------------------------
//   if (sweepActive) {

//     myServo.write(servoAngle);
//     delay(25);

//     servoAngle += servoStep;

//     if (servoAngle >= 140 || servoAngle <= 20) {
//       servoStep = -servoStep;
//     }
//   }

//   // ----------------------------
//   // OPTIONAL: Ultrasonic read (if needed later)
//   // ----------------------------
//   float dist = readDistance();

//   Serial.print("DIST:");
//   Serial.println(dist);
// }
