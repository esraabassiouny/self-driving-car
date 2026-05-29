// Test sequence for motor steering
// Runs the exact commands one after another

void setup() {

  Serial.begin(9600);

  // Motor pins
  pinMode(4, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  pinMode(3, OUTPUT); // ENA
  pinMode(5, OUTPUT); // ENB

  forward();
}

void loop() {

  runMotors(155,145);
  delay(700);

  runMotors(154,146);
  delay(700);

  runMotors(156,144);
  delay(700);

  runMotors(157,143);
  delay(700);

  runMotors(158,142);
  delay(700);

  runMotors(161,139);
  delay(700);

  runMotors(162,138);
  delay(700);

  runMotors(164,136);
  delay(700);

  runMotors(165,135);
  delay(700);

  runMotors(167,133);
  delay(700);

  runMotors(168,132);
  delay(700);

  runMotors(171,129);
  delay(700);

  runMotors(173,127);
  delay(700);

  runMotors(176,124);
  delay(700);

  runMotors(178,122);
  delay(700);

  runMotors(182,118);
  delay(700);

  runMotors(189,111);
  delay(700);

  runMotors(193,107);
  delay(700);

  runMotors(199,101);
  delay(700);

  runMotors(204,96);
  delay(700);

  runMotors(208,92);
  delay(700);

  runMotors(219,81);
  delay(700);

  runMotors(225,75);
  delay(700);

  runMotors(233,67);
  delay(700);

  runMotors(242,58);
  delay(700);

  runMotors(250,50);
  delay(700);

  runMotors(255,43);
  delay(700);

  runMotors(255,38);
  delay(3000);

  stopMotors();

  while(true);
}


// =========================
// Motor Functions
// =========================

void forward() {

  // Left motor forward
  digitalWrite(4, HIGH);
  digitalWrite(10, LOW);

  // Right motor forward
  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);
}

void runMotors(int leftPWM, int rightPWM) {

  analogWrite(3, leftPWM);
  analogWrite(5, rightPWM);

  Serial.print("LEFT: ");
  Serial.print(leftPWM);

  Serial.print(" RIGHT: ");
  Serial.println(rightPWM);
}

void stopMotors() {

  analogWrite(3, 0);
  analogWrite(5, 0);

  Serial.println("STOP");
}