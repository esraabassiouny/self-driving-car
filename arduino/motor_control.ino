// Motor pins
int IN1 = 4;
int IN2 = 5;
int IN3 = 6;
int IN4 = 7;

int enA = 3;   // Left motor speed
int enB = 10;  // Right motor speed

int left_speed = 0;
int right_speed = 0;

void setup() {
  Serial.begin(9600);

  // Set motor pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
}

void loop() {
  // Read serial command in format L<value>R<value>\n
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');

    int lIndex = cmd.indexOf('L');
    int rIndex = cmd.indexOf('R');

    if (lIndex != -1 && rIndex != -1) {
      left_speed = cmd.substring(lIndex + 1, rIndex).toInt();
      right_speed = cmd.substring(rIndex + 1).toInt();
    }
  }

  // Set motor directions (forward)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Set motor speeds
  analogWrite(enA, left_speed);
  analogWrite(enB, right_speed);
}