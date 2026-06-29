// =====================================================
// MOTOR PINS
// =====================================================

int IN1 = 4;
int IN2 = 10;
int IN3 = 6;
int IN4 = 7;

// =====================================================
// ULTRASONIC SENSOR
// =====================================================

const int trigPin = 11;
const int echoPin = 12;

float distance = 999;

int enA = 3;   // Left motor PWM
int enB = 5;   // Right motor PWM

int left_speed = 0;
int right_speed = 0;

// =====================================================
// MOVE FORWARD
// =====================================================


void forward(int leftSpeed, int rightSpeed)
{
    if (leftSpeed >= 0) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(enB, leftSpeed);
    } else {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(enB, abs(leftSpeed));
    }

    if (rightSpeed >= 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(enA, rightSpeed);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(enA, abs(rightSpeed));
    }
}


// =====================================================
// STOP CAR
// =====================================================

void stopCar() {

  analogWrite(enA, 0);
  analogWrite(enB, 0);
}


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  // STOP AT STARTUP
  stopCar();
}

float readDistanceCM() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // timeout 30ms

  float dist = duration * 0.034 / 2.0;

  return dist;
}
// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

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

    forward(left_speed, right_speed);
    }
    }
    static unsigned long lastSend = 0;

  if (millis() - lastSend > 100) {   // كل 100ms
    lastSend = millis();

    distance = readDistanceCM();

    Serial.print("DIST:");
    Serial.println(distance);
  }
}
