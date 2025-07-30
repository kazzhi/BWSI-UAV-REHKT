#include <Servo.h>
#include <PID_v1.h>

// Define the states for the state machine
enum RobotState {
  FORWARD_25,
  FORWARD_50,
  FORWARD_75,
  FORWARD_100,
  FORWARD_125,
  FORWARD_150,
  FORWARD_175,
  FORWARD_200,
  FORWARD_225,
  //backwards
  BACKWARD_25,
  BACKWARD_50,
  BACKWARD_75,
  BACKWARD_100,
  BACKWARD_125,
  BACKWARD_150,
  BACKWARD_175,
  BACKWARD_200,
  BACKWARD_225,
  //turns
  TURN_LEFT,
  TURN_RIGHT,
  STOP
};

// Define the sequence of states for the state machine
RobotState stateSequence[] = { 
  // FORWARD_100,
  // TURN_LEFT,
  // BACKWARD_100,
  // TURN_RIGHT,

  // FORWARD_100,
  // TURN_LEFT,
  // FORWARD_100,
  STOP 
};

// Servo objects for controlling the wheels
Servo leftWheel;
Servo rightWheel;
Servo latch;

// PID control variables
double leftInput, leftOutput, leftSetpoint = 0;
double rightInput, rightOutput, rightSetpoint = 0;

// Define the analog input pins for the left and right encoders
const int leftEncoderPin = A0;
const int rightEncoderPin = A1;

// Gear ratio and wheel properties
const float gearRatio = 1.0;                                                  // 1 spin of the encoder results in 3 spins of the wheel
const float wheelDiameterInches = 2.75591;                                      // Wheel diameter in inches
const float wheelCircumference = wheelDiameterInches * 2.54 * 3.14159265359;  // Circumference of the wheel in centimeters

bool leftActionCompleted = false;
bool rightActionCompleted = false;

// State variables for the left wheel
float previousLeftAngle = 0.0;
float cumulativeLeftDistance = 0.0;

// State variables for the right wheel
float previousRightAngle = 0.0;
float cumulativeRightDistance = 0.0;

// Flags for initialization
bool firstLoopLeft = true;
bool firstLoopRight = true;

// Control parameters
const float kPLeftDrive = 0.025;  // Proportional gain for deceleration
const float kPRightDrive = 0.025;
const float errorTolerance = 0.5;

const float maxPower = 1;  // Maximum power
const float leftPowerScalar = 1;
const float rightPowerScalar = 1;
const float accelerationStep = 0.04;  // Step for uniform acceleration

// PID parameters for the last few cm
double leftkP = 0.025, leftkI = 0.0000, leftkD = 0.0000;        // Left wheel PID parameters
double rightkP = 0.025, rightkI = 0.0000, rightkD = 0.0000;  // Right wheel PID parameters

PID leftPID(&leftInput, &leftOutput, &leftSetpoint, leftkP, leftkI, leftkD, DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, rightkP, rightkI, rightkD, DIRECT);

// Variables for timing
unsigned long previousMillis = 0;
const long interval = 1000;  // Interval to send data in milliseconds

// Acceleration parameters
const unsigned long accelerationDuration = 1500;  // 0.5 seconds in milliseconds
unsigned long accelerationStartTime = 0;

// Index to keep track of the current state in the sequence
int currentStateIndex = 0;

// Target distances for each state
float turnWheelTargetDist = 12; // Turn target distance in cm, UNTESTED

void resetVariables() {
  leftActionCompleted = false;
  rightActionCompleted = false;

  previousLeftAngle = 0.0;
  cumulativeLeftDistance = 0.0;

  previousRightAngle = 0.0;
  cumulativeRightDistance = 0.0;

  firstLoopLeft = true;
  firstLoopRight = true;

  accelerationStartTime = millis();
}

void handleStateTransitions() {
  if (leftActionCompleted && rightActionCompleted) {
    currentStateIndex = (currentStateIndex + 1) % (sizeof(stateSequence) / sizeof(stateSequence[0]));
    resetVariables();
    Serial.print("Transitioned to state: ");
    Serial.println(stateSequence[currentStateIndex]);
  }
}

void moveForward(float distance) {
  leftSetpoint = distance;
  rightSetpoint = distance;
  Serial.println("Executing moveForward");
}

void moveBackward(float distance) {
  leftSetpoint = -distance;
  rightSetpoint = -distance;
  Serial.println("Executing moveBackward");
}

void turnLeft() {
  leftSetpoint = -turnWheelTargetDist;
  rightSetpoint = turnWheelTargetDist;
  Serial.println("Executing turnLeft");
}

void turnRight() {
  leftSetpoint = turnWheelTargetDist;
  rightSetpoint = -turnWheelTargetDist;
  Serial.println("Executing turnRight");
}

void stop() {
  leftWheel.write(90); // Stop the left wheel
  rightWheel.write(90); // Stop the right wheel
  Serial.println("Robot has stopped.");
  while (true); // Infinite loop to stop the robot
}

void setup() {
  // Start the serial communication at 9600 baud
  Serial.begin(115200);

  // Attach the servos to the specified pins
  leftWheel.attach(3);   // attaches the left wheel servo on pin 3
  rightWheel.attach(2);  // attaches the right wheel servo on pin 2
  latch.attach(30);

  latch.write(70);
  //latch.write(100);

  // Initialize the PID controllers
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(-1, 1);  // Set the output limits to match the desired power range

  rightPID.SetMode(AUTOMATIC);
  rightPID.SetOutputLimits(-1, 1);  // Set the output limits to match the desired power range

  // Initialize the acceleration start time
  accelerationStartTime = millis();

  // Set initial setpoints to zero
  leftSetpoint = 0;
  rightSetpoint = 0;

  // Print initial state
  Serial.print("Initial state: ");
  Serial.println(stateSequence[currentStateIndex]);
}

void loop() {
  // Execute current state logic
switch (stateSequence[currentStateIndex]) {
    case FORWARD_25:
      moveForward(25.0);
      break;
    case FORWARD_50:
      moveForward(50.0);
      break;
    case FORWARD_75:
      moveForward(75.0);
      break;
    case FORWARD_100:
      moveForward(100.0);
      break;
    case FORWARD_125:
      moveForward(125.0);
      break;
    case FORWARD_150:
      moveForward(150.0);
      break;
    case FORWARD_175:
      moveForward(175.0);
      break;
    case FORWARD_200:
      moveForward(200.0);
      break;
    case FORWARD_225:
      moveForward(225.0);
      break;
    case BACKWARD_25:
      moveBackward(25.0);
      break;
    case BACKWARD_50:
      moveBackward(50.0);
      break;
    case BACKWARD_75:
      moveBackward(75.0);
      break;
    case BACKWARD_100:
      moveBackward(100.0);
      break;
    case BACKWARD_125:
      moveBackward(125.0);
      break;
    case BACKWARD_150:
      moveBackward(150.0);
      break;
    case BACKWARD_175:
      moveBackward(175.0);
      break;
    case BACKWARD_200:
      moveBackward(200.0);
      break;
    case BACKWARD_225:
      moveBackward(225.0);
      break;
    case TURN_LEFT:
      turnLeft();
      break;
    case TURN_RIGHT:
      turnRight();
      break;
    case STOP:
      stop();
      break;
}
  // Variables to store power values
  double leftPower = 0.0;
  double rightPower = 0.0;

  // --- Left Wheel ---
  int leftEncoderValue = analogRead(leftEncoderPin);
  float leftWheelAngle = gearRatio * (leftEncoderValue * 360.0 / (1023.0 * 3.3 / 5.0));
  leftWheelAngle = fmod(leftWheelAngle, 360.0);
  if (leftWheelAngle < 0) {
    leftWheelAngle += 360.0;
  }
  if (firstLoopLeft) {
    previousLeftAngle = leftWheelAngle;
    firstLoopLeft = false;
  }
  float leftAngleDifference = leftWheelAngle - previousLeftAngle;
  if (leftAngleDifference < -180.0) {
    leftAngleDifference += 360.0;
  } else if (leftAngleDifference > 180.0) {
    leftAngleDifference -= 360.0;
  }
  float leftDistanceTraveled = (leftAngleDifference / 360.0) * wheelCircumference;
  cumulativeLeftDistance -= leftDistanceTraveled;
  previousLeftAngle = leftWheelAngle;

  // --- Right Wheel ---
  int rightEncoderValue = analogRead(rightEncoderPin);
  float rightWheelAngle = gearRatio * (rightEncoderValue * 360.0 / (1023.0 * 3.3 / 5.0));
  rightWheelAngle = fmod(rightWheelAngle, 360.0);
  if (rightWheelAngle < 0) {
    rightWheelAngle += 360.0;
  }
  if (firstLoopRight) {
    previousRightAngle = rightWheelAngle;
    firstLoopRight = false;
  }
  float rightAngleDifference = rightWheelAngle - previousRightAngle;
  if (rightAngleDifference < -180.0) {
    rightAngleDifference += 360.0;
  } else if (rightAngleDifference > 180.0) {
    rightAngleDifference -= 360.0;
  }
  float rightDistanceTraveled = -(rightAngleDifference / 360.0) * wheelCircumference;
  cumulativeRightDistance -= rightDistanceTraveled;
  previousRightAngle = rightWheelAngle;

  // Calculate the error (distance remaining to target)
  float leftError = leftSetpoint - cumulativeLeftDistance;
  float rightError = rightSetpoint - cumulativeRightDistance;

  // --- Acceleration and Deceleration ---
  unsigned long currentMillis = millis();
  unsigned long elapsedMillis = currentMillis - accelerationStartTime;

  if (!leftActionCompleted) {
    if (abs(leftError) > abs(leftSetpoint) / 2) {
      // Linearly interpolate the power for acceleration
      float accelerationFactor = (float)elapsedMillis / accelerationDuration;
      leftPower = (leftSetpoint / abs(leftSetpoint)) * accelerationFactor;
    } else if (abs(leftError) > 6) {
      // Proportional control for the majority of the path
      leftPower = kPLeftDrive * leftError;
    } else if (abs(leftError) > errorTolerance) {
      // PID control for the last few cm
      leftInput = cumulativeLeftDistance;
      leftPID.Compute();
      leftPower = leftOutput;
    } else {
      leftPower = 0;
      leftActionCompleted = true;
      Serial.println("Left action completed");
    }
  }
  if (!rightActionCompleted) {
    if (abs(rightError) > abs(rightSetpoint) / 2) {
      // Linearly interpolate the power for acceleration
      float accelerationFactor = (float)elapsedMillis / accelerationDuration;
      rightPower = (rightSetpoint / abs(rightSetpoint)) * accelerationFactor;
    } else if (abs(rightError) > 6) {
      // Proportional control for the majority of the path
      rightPower = kPRightDrive * rightError;
    } else if (abs(rightError) > errorTolerance) {
      // PID control for the last few cm
      rightInput = cumulativeRightDistance;
      rightPID.Compute();
      rightPower = rightOutput;
    } else {
      rightPower = 0;
      rightActionCompleted = true;
      Serial.println("Right action completed");
    }
  }

  // Ensure power does not exceed maximum limits
  if (leftPower > maxPower) {
    leftPower = maxPower;
  }
  if (rightPower > maxPower) {
    rightPower = maxPower;
  }

  // Set the servos to the calculated power
  leftWheel.write(90 + (leftPower * 90));
  rightWheel.write(90 + (rightPower * 90));

  // --- Output Data for Serial Plotter ---
  Serial.print(cumulativeLeftDistance, 2);  // Print left cumulative distance
  Serial.print(",");
  Serial.print(cumulativeRightDistance, 2);  // Print right cumulative distance
  Serial.print(",");
  Serial.println(leftPower, 4);  // Print left power
  Serial.print(",");
  Serial.println(rightPower, 4);  // Print right power

  // Use millis() to control the output rate
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
  }

  // Handle state transitions
  handleStateTransitions();
}
