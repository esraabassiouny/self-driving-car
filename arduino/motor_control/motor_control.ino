// =====================================================
// MOTOR PINS
// =====================================================

int IN1 = 4;
int IN2 = 10;
int IN3 = 6;
int IN4 = 7;

int enA = 3;   // Left motor PWM
int enB = 5;   // Right motor PWM

int left_speed = 0;
int right_speed = 0;

// =====================================================
// MOVE FORWARD
// =====================================================

void forward(int leftSpeed, int rightSpeed) {

  // LEFT MOTOR
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // RIGHT MOTOR
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

  // LEFT MOTOR FORWARD
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // RIGHT MOTOR FORWARD
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // LEFT MOTOR STOP
  analogWrite(enA, 0);

  // RIGHT MOTOR FAST
  analogWrite(enB, 255);

  delay(t);
}

// =====================================================
// U-TURN
// =====================================================

void uTurnLeft() {

  Serial.println("START_U_TURN");

  // Stop before turning
  stopCar();

  delay(300);

  forward(200, 200);

  delay(500);

  stopCar();

  delay(300);
  // Perform left U-turn
  // 4300s
  sharpLeft(4400);

  // // Stop after turning
  // stopCar();

  // delay(300);

  // forward(200, 200);

  // delay(500);

  stopCar();
  delay(300);

  Serial.println("END_U_TURN");
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  // STOP AT STARTUP
  stopCar();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  // ==========================================
  // READ SERIAL COMMAND
  // ==========================================

  if (Serial.available()) {

    String cmd = Serial.readStringUntil('\n');

    cmd.trim();

    // ==========================================
    // SPECIAL COMMAND:
    // UTURN_LEFT
    // ==========================================

    if (cmd == "UTURN_LEFT") {

      uTurnLeft();
    }

    // ==========================================
    // NORMAL MOTOR COMMAND:
    // FORMAT -> L120R140
    // ==========================================

    else {

  int lIndex = cmd.indexOf('L');
  int rIndex = cmd.indexOf('R');

  if (lIndex != -1 && rIndex != -1) {

    left_speed =
      cmd.substring(lIndex + 1, rIndex).toInt();

    right_speed =
      cmd.substring(rIndex + 1).toInt();

    forward(left_speed, right_speed);
    // delay(700);
    Serial.print("ACK:");
    Serial.print(left_speed);
    Serial.print(",");
    Serial.println(right_speed);
  }
  }
  }
}