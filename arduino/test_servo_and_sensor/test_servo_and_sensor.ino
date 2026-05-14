#include <Servo.h>

Servo myServo;

int trigPin = 11;
int echoPin = 12;

long duration;
float distance;

int angles[] = {30, 90, 150};  // left, center, right

float readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000);
  distance = duration * 0.034 / 2;

  return distance;
}

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  myServo.attach(9);  // signal pin

  Serial.println("Servo + Ultrasonic Test");
}

void loop() {
  for (int i = 0; i < 3; i++) {
    int angle = angles[i];

    myServo.write(angle);
    delay(500);  // wait for servo to reach position

    float d = readDistance();

    Serial.print("Angle: ");
    Serial.print(angle);

    Serial.print(" | Distance: ");
    Serial.print(d);
    Serial.println(" cm");

    delay(500);
  }

  Serial.println("----------------------");
}