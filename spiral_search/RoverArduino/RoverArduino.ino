// Define motor control pins
const int leftMotorForwardPin = 4;
const int leftMotorBackwardPin = 5;
const int rightMotorForwardPin = 2;
const int rightMotorBackwardPin = 3;

const int fullSpeed = 230;
const int halfSpeed = 255;

const unsigned long timeoutDuration = 2000;  // 2 seconds timeout
unsigned long lastSerialTime = 0;

void setup() {
  // Set motor pins as output
  pinMode(leftMotorForwardPin, OUTPUT);
  pinMode(leftMotorBackwardPin, OUTPUT);
  pinMode(rightMotorForwardPin, OUTPUT);
  pinMode(rightMotorBackwardPin, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Check if data is available on serial
  if (Serial.available() > 0) {
    // Read the incoming command
    char command = Serial.read();
    
    // Perform actions based on the command received
    if (command == 'F') {
      moveForward();
    } else if (command == 'B') {
      moveBackward();
    } else if (command == 'L') {
      turnLeft();
    } else if (command == 'R') {
      turnRight();
    } else if (command == 'S') {
      stopMotors();
    } else if (command == 'I') {
      moveForwardLeft();  // forward-left command
    } else if (command == 'J') {
      moveForwardRight();  // forward-right command
    }
    Serial.println(command);
  }

  // lastSerialTime = millis();

  //   // Check for serial timeout
  // if (millis() - lastSerialTime > timeoutDuration) {
  //   stopMotors();  // Stop motors if no command is received within timeout
  // }
}

// Function to move forward
void moveForward() {
  analogWrite(leftMotorForwardPin, 0);
  analogWrite(leftMotorBackwardPin, fullSpeed);
  analogWrite(rightMotorForwardPin, 0);
  analogWrite(rightMotorBackwardPin, fullSpeed);
}

// Function to move backward
void moveBackward() {
  analogWrite(leftMotorForwardPin, 0);
  analogWrite(leftMotorBackwardPin, fullSpeed);
  analogWrite(rightMotorForwardPin, 0);
  analogWrite(rightMotorBackwardPin, fullSpeed);
}

// Function to turn left
void turnLeft() {
  analogWrite(leftMotorForwardPin, 0);
  analogWrite(leftMotorBackwardPin, fullSpeed);  // Reverse left motor
  analogWrite(rightMotorForwardPin, fullSpeed);  // Forward right motor
  analogWrite(rightMotorBackwardPin, 0);
}

// Function to turn right
void turnRight() {
  analogWrite(leftMotorForwardPin, fullSpeed);  // Forward left motor
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, 0);
  analogWrite(rightMotorBackwardPin, fullSpeed);  // Reverse right motor
}

// Function to move forward-left (right motor moves forward, left motor slower or stopped)
void moveForwardLeft() {
  analogWrite(leftMotorForwardPin, 0);   // Stop left motor
  analogWrite(leftMotorBackwardPin, halfSpeed);  // Ensure left motor is not going backward
  analogWrite(rightMotorForwardPin, halfSpeed); // Full speed on right motor
  analogWrite(rightMotorBackwardPin, 0); 
}

// Function to move forward-right (left motor moves forward, right motor slower or stopped)
void moveForwardRight() {
  analogWrite(leftMotorForwardPin, halfSpeed);  // half speed on left motor
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, 0);  // Stop right motor
  analogWrite(rightMotorBackwardPin, halfSpeed); 
}

// Function to stop all motors
void stopMotors() {
  analogWrite(leftMotorForwardPin, 0);
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, 0);
  analogWrite(rightMotorBackwardPin, 0);
}
