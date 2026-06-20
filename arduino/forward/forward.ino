#include <Servo.h>

Servo myServo;

// =====================================================
// Ultrasonic
// =====================================================
const int trigPin = 11;
const int echoPin = 12;

// =====================================================
// Motor Driver
// =====================================================
const int IN1 = 4;
const int IN2 = 10;
const int IN3 = 6;
const int IN4 = 7;

const int enA = 3;
const int enB = 5;

// =====================================================
// Driving Speed
// =====================================================
int left_speed = 150;
int right_speed = 150;

// =====================================================
// Servo Sweep
// =====================================================
int servoAngle = 20;
int servoStep = 5;
bool servoSweepEnabled = true;

// =====================================================
// Distance
// =====================================================
long duration;
float distance;

// =====================================================
// Read Ultrasonic Distance
// =====================================================
float readDistance()
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH, 30000);

    if (duration == 0)
        return 999;

    return duration * 0.034 / 2.0;
}

// =====================================================
// Move Forward
// =====================================================
void forward(int leftSpeed, int rightSpeed)
{

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    analogWrite(enA, leftSpeed);
    analogWrite(enB, rightSpeed);
}


// =====================================================
// Stop Car
// =====================================================
void stopCar()
{
    analogWrite(enA, 0);
    analogWrite(enB, 0);
}

// =====================================================
// Sharp Left
// =====================================================
void sharpLeft(int t)
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    analogWrite(enA, 0);
    analogWrite(enB, 255);

    delay(t);
}

// =====================================================
// U-Turn
// =====================================================
void uTurnLeft()
{
    Serial.println("START_U_TURN");

    stopCar();
    delay(300);

    forward(180, 180);
    delay(500);

    stopCar();
    delay(300);

    sharpLeft(4400);

    stopCar();
    delay(300);

    Serial.println("END_U_TURN");
}

// =====================================================
// Setup
// =====================================================
void setup()
{
    Serial.begin(9600);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);

    myServo.attach(9);
    myServo.write(90);

    Serial.println("SYSTEM_READY");
}

// =====================================================
// Main Loop
// =====================================================
void loop()
{
    // -------------------------------------------------
    // Receive Commands From Raspberry Pi
    // -------------------------------------------------
    if (Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd == "START_SWEEP")
        {
            servoSweepEnabled = true;
        }
        else if (cmd == "STOP_SWEEP")
        {
            servoSweepEnabled = false;
            myServo.write(90);
        }
        else if (cmd == "UTURN_LEFT")
        {
            uTurnLeft();
        }
    }

    // -------------------------------------------------
    // Servo Sweep
    // -------------------------------------------------
    if (servoSweepEnabled)
    {
        myServo.write(servoAngle);

        servoAngle += servoStep;

        if (servoAngle >= 140 || servoAngle <= 20)
        {
            servoStep = -servoStep;
        }
    }

    // -------------------------------------------------
    // Read Distance
    // -------------------------------------------------
    distance = readDistance();

    Serial.print("DIST:");
    Serial.println(distance);

    // -------------------------------------------------
    // Obstacle Avoidance
    // -------------------------------------------------
    if (distance < 25)
    {
        stopCar();
        Serial.println("OBSTACLE");
    }
    else
    {
        forward(left_speed, right_speed);
    }

    delay(30);
}


// // =====================================================
// // Ultrasonic
// // =====================================================
// const int trigPin = 11;
// const int echoPin = 12;

// // =====================================================
// // Motor Driver
// // =====================================================
// const int IN1 = 4;
// const int IN2 = 10;
// const int IN3 = 6;
// const int IN4 = 7;

// const int enA = 3;
// const int enB = 5;

// // =====================================================
// // Driving Speed
// // =====================================================
// int left_speed = 150;
// int right_speed = 150;

// // =====================================================
// // Distance
// // =====================================================
// long duration;
// float distance;

// // =====================================================
// // Read Ultrasonic Distance
// // =====================================================
// float readDistance()
// {
//     digitalWrite(trigPin, LOW);
//     delayMicroseconds(2);

//     digitalWrite(trigPin, HIGH);
//     delayMicroseconds(10);

//     digitalWrite(trigPin, LOW);

//     duration = pulseIn(echoPin, HIGH, 30000);

//     if (duration == 0)
//         return 999;

//     return duration * 0.034 / 2.0;
// }

// // =====================================================
// // Move Forward
// // =====================================================
// void forward(int leftSpeed, int rightSpeed)
// {

//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, HIGH);
    
//     digitalWrite(IN3, HIGH);
//     digitalWrite(IN4, LOW);

//     analogWrite(enA, leftSpeed);
//     analogWrite(enB, rightSpeed);
// }

// // =====================================================
// // Stop Car
// // =====================================================
// void stopCar()
// {
//     analogWrite(enA, 0);
//     analogWrite(enB, 0);
// }

// // =====================================================
// // Sharp Left
// // =====================================================
// void sharpLeft(int t)
// {
//     digitalWrite(IN1, HIGH);
//     digitalWrite(IN2, LOW);

//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, HIGH);

//     analogWrite(enA, 0);
//     analogWrite(enB, 255);

//     delay(t);
// }

// // =====================================================
// // U-Turn
// // =====================================================
// void uTurnLeft()
// {
//     Serial.println("START_U_TURN");

//     stopCar();
//     delay(300);

//     forward(180, 180);
//     delay(500);

//     stopCar();
//     delay(300);

//     sharpLeft(4400);

//     stopCar();
//     delay(300);

//     Serial.println("END_U_TURN");
// }

// // =====================================================
// // Setup
// // =====================================================
// void setup()
// {
//     Serial.begin(9600);

//     pinMode(trigPin, OUTPUT);
//     pinMode(echoPin, INPUT);

//     pinMode(IN1, OUTPUT);
//     pinMode(IN2, OUTPUT);
//     pinMode(IN3, OUTPUT);
//     pinMode(IN4, OUTPUT);

//     pinMode(enA, OUTPUT);
//     pinMode(enB, OUTPUT);

//     stopCar();

//     Serial.println("SYSTEM_READY");
// }

// // =====================================================
// // Main Loop
// // =====================================================
// void loop()
// {
//     // Read commands from Raspberry Pi
//     if (Serial.available())
//     {
//         String cmd = Serial.readStringUntil('\n');
//         cmd.trim();

//         if (cmd == "UTURN_LEFT")
//         {
//             uTurnLeft();
//         }
//     }

//     // Read distance
//     distance = readDistance();

//     Serial.print("DIST:");
//     Serial.println(distance);

//     // Obstacle avoidance
//     if (distance < 25)
//     {
//         stopCar();
//         Serial.println("OBSTACLE");
//     }
//     else
//     {
//         forward(left_speed, right_speed);
//     }

//     delay(50);
// }
