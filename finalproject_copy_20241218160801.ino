//Rajya Kolluri
//Collins Kibet
//Final Project
//CS 328
// Prof. Stam Darrell

//Innitializations
//***************************************************************
#include <Servo.h>
#include <SoftwareSerial.h>

// Motor Control Pins
#define MotorPWM_A 4  // PWM pin for left motor
#define MotorPWM_B 5  // PWM pin for right motor
#define INA1A 32      // Left motor direction control pin 1
#define INA2A 34      // Left motor direction control pin 2
#define INA1B 30      // Right motor direction control pin 1
#define INA2B 36      // Right motor direction control pin 2

#define SPEED 180  // Default motor speed
#define MAX_SPEED 255  // Maximum motor speed

// Bluetooth module pins
SoftwareSerial BTserial(10, 11); // RX, TX pins for Bluetooth

// Ultrasonic Sensor Pins
#define trig_pin A4 // Trigger pin for the rangefinder
#define echo_pin A5 // Echo pin for the rangefinder
#define OBSTACLE_THRESHOLD 20 // Obstacle detection distance threshold in cm

// Servo Configuration
Servo myServo;           // Servo object for scanning
#define SERVO_PIN 9     // Servo control pin
#define SERVO_MIN_ANGLE 0   // Minimum servo angle
#define SERVO_MAX_ANGLE 180 // Maximum servo angle

// Servo Sweep Variables
bool servoDirection = true; // Sweep direction: true = forward, false = backward
int servoAngle = 90; 

// Function to read distance from the rangefinder
//***************************************************************
int readDistance() {
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  long duration = pulseIn(echo_pin, HIGH);
  return duration * 0.034 / 2; // Convert duration to distance in cm
}

// Function to sweep the servo
//***************************************************************
void sweepServo() {
  servoAngle += (servoDirection ? 1 : -1);
  if (servoAngle >= SERVO_MAX_ANGLE) servoDirection = false;
  if (servoAngle <= SERVO_MIN_ANGLE) servoDirection = true;
  myServo.write(servoAngle);
  delay(10);
}

// Function to detect obstacles
//***************************************************************
bool isObstacleDetected() {
  int distance = readDistance();
  return (distance > 0 && distance < OBSTACLE_THRESHOLD);
}

// Function to stop the car
//***************************************************************
void STOP() {
  analogWrite(MotorPWM_A, 0);
  analogWrite(MotorPWM_B, 0);

  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, LOW);

  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, LOW);
  Serial.println("Stopping Car");
}

// Function to process commands
//***************************************************************
void processCommand(String command) {
  command.trim(); // Clean up input

  if (isObstacleDetected()) {
    Serial.println("Obstacle detected! Stopping car.");
    STOP();
    return;
  }

  if (command == "F") {
    FORWARD();
  } else if (command == "B") {
    BACKWARD();
  } else if (command == "L") {
    TURN_LEFT();
  } else if (command == "R") {
    TURN_RIGHT();
  } else if (command == "S") {
    STOP();
  } else {
    Serial.println("Unknown Command!");
  }
}

// Function to move the car forward
//***************************************************************
void FORWARD() {
  analogWrite(MotorPWM_A, SPEED);
  analogWrite(MotorPWM_B, SPEED);

  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
  Serial.println("Moving Forward");
}

// Function to move the car backward
//***************************************************************
void BACKWARD() {
  analogWrite(MotorPWM_A, SPEED);
  analogWrite(MotorPWM_B, SPEED);

  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, HIGH);

  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, HIGH);
  Serial.println("Moving Backward");
}

// Function to turn the car left
//***************************************************************
void TURN_LEFT() {
  analogWrite(MotorPWM_A, 0);  // Left motor stopped
  analogWrite(MotorPWM_B, SPEED);  // Right motor active

  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, LOW);

  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
  Serial.println("Turning Left");
}

// Function to turn the car right
//***************************************************************
void TURN_RIGHT() {
  analogWrite(MotorPWM_A, SPEED);  // Left motor active
  analogWrite(MotorPWM_B, 0);  // Right motor stopped

  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, LOW);
  Serial.println("Turning Right");
}

//setup function
//***************************************************************
void setup() {
  Serial.begin(9600);
  BTserial.begin(38400);

  // Initialize motor control pins
  pinMode(MotorPWM_A, OUTPUT);
  pinMode(MotorPWM_B, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);

  // Initialize ultrasonic sensor pins
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  // Initialize servo
  myServo.attach(SERVO_PIN);
  myServo.write(servoAngle);

  Serial.println("Car ready to receive commands with obstacle detection!");
}

//The loop function
//***************************************************************
void loop() {
  String message = readCommand(); // Read Bluetooth commands
  if (message.length() > 0) {
    Serial.print("Raw Message: ");
    Serial.println(message);
    processCommand(message);
  }
  sweepServo(); // Continuously scan for obstacles
}

// Function to read Bluetooth commands
//***************************************************************
String readCommand() {
  String asciiData = "";  // Store received data
  char bluetoothData;
  while (BTserial.available()) {
    bluetoothData = BTserial.read();
    // Ignore Carriage Return (CR) and Line Feed (LF)
    if (bluetoothData != 13 && bluetoothData != 10) {
      asciiData += (char)bluetoothData;  // Build the message
    }
  }
  return asciiData;  // Return the received data (excluding CR/LF)
}
