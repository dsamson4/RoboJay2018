#include <SoftwareSerial.h>
#include <Kangaroo.h>
#include <math.h>

//=====Pin assignments=====
const int u_pin = 2;

//=====Constants=====
const float piApprox = 3.14159;
const float threshold = 0.5;

//=====for tracking time=====
unsigned long currentMillis; // [ms]

//=====variables from array=====
int moveSpeed = 1; // [m / s]
int xCoordinate; // [cm]
int yCoordinate; // [cm]
int orientation = 0; // [deg]

//=====for determining drive commands=====
long wheelSpeed; // [ticks / s]
float moveDuration; // [s]
float moveDist; // [cm]
float turnDist; // [cm]
float turnDuration; // [s]
const byte stopDuration = 1; // [s]
long driveSpeed; // [ticks / s]
long turnSpeed; // [ticks / s]
boolean ifTurnClockwise = true; // The robot will move clockwise.

//=====for determining motion path=====
float angleOppositeY;
float averageOfXAndY;
float differenceFromAverage;
float angleOppositeY2;
float angleOppositeY1;
float angleOppositeX;
float arcAngle;
float arcRadius;
float clockwiseAngle;
float counterclockwiseAngle;

//=====Robot Dimensions=====
const float turnRad = 9.875; // [cm]
const byte wheelDiam = 65; // [mm]
const float wheelRad = ((float) wheelDiam / 2) / 10; // [cm]

//=====Room Dimensions===== (used for transform, which should be done with ROS)
const float hallWidth = 40; // currently arbitrary number

//const float global_arr[][];
const float global[3] = {1.0,0.0,0.0};
const float local1[3] = {2.0,1.0,0.0};
const float local2[3] = {1.0,0.1,0.0};
float diff[3];
float* locals[2];
float curr_reading[3];
float yError = 0.0;
float xabs = 0.0;
float yabs = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Starting move_coordinates_array.ino");
  int numElements = sizeof(global)/sizeof(float);
  xCoordinate = global[0];
  yCoordinate = global[1];
  orientation = global[2];
  locals[0] = local1;
  locals[1] = local2;

  yabs = getDist(u_pin);
  Serial.println("Start point: (0,0)");
}

void loop() {
  // put your main code here, to run repeatedly:
  int count = 0;
  Serial.print("yError: ");
  Serial.println(yError);
  do { //diff[1] = error in y
    Serial.print("Current point: (");
    Serial.print(xabs);
    Serial.print(",");
    Serial.print(yabs);
    Serial.println(")");
    
    Serial.print("Move to point: (");
    Serial.print(global[0]);
    Serial.print(",");
    Serial.print(global[1]);
    Serial.println(")");
    
    // Set move commands
    driveSpeed = convertLinSpeedToWheelSpeed((float) moveSpeed); // Convert input speed [cm / s] to wheel rotational drive speed [ticks / s]
    
    calculateAngleOppositeY(); // \atan(Y/X)
    calculateAverageOfCoordinates(); // (X + Y) / 2
    calculateDifferenceBetweenYAndAvg(); // y_2
    calculateAngleOppositeY2(); // \theta_2
    calculateAngleOppositeY1(); // \theta_1 = \Theta - \theta_2
    calculateAngleOppositeX(); // \Phi
    calculateArcAngle();
    calculateArcRadius();
    
    calculateMoveDist();
    calculateMoveDuration(); // outputs time in seconds
    turnSpeed = calculateTurnSpeed();
  
    moveToCoordinates();
    stopMoving();
    
    calculateTurningAngles(); //turning to correct position
    calculateTurnDist();
    turnSpeed = driveSpeed;
    calculateTurnDuration();
    turnToOrientation();
  
    Serial.println("STOP");
    
    int numElements = sizeof(global)/sizeof(float);
  
    Serial.println("Global");
    for (int i = 0; i < numElements; i++) {
      Serial.print(global[i]);
      Serial.print(", ");
    }
    Serial.println();

    for (int i = 0; i < numElements; i++) {
      curr_reading[i] = locals[count][i];
    }
    xabs = curr_reading[0];
    yabs = curr_reading[1];
    
    Serial.println("Local");
    for (int i = 0; i < numElements; i++) {
      Serial.print(curr_reading[i]);
      Serial.print(", ");
    }
    Serial.println();

    arraySubtract(diff, global, locals[count], numElements);
    Serial.println("Error");
    for (int i = 0; i < numElements; i++) {
      Serial.print(diff[i]);
      Serial.print(", ");
    }
    Serial.println();

    yError = abs(diff[1]);
    
    if (yError < threshold) {
      Serial.println("BELOW THRESHOLD");
    } else {
      xCoordinate = diff[0];
      yCoordinate = diff[1];
      orientation = diff[2];
    }
    count++;
  } while (yError > threshold);

}

//Takes two arrays, their size, and subtracts the second from the first
int arraySubtract(float diff[], float a[], float b[], int numElements){
    for(int i = 0; i < numElements; i++){
        diff[i] = a[i] - b[i]; 
    }
    return diff; 
}

long convertLinSpeedToWheelSpeed(byte cm) { // linear speed is given by user
  long maxP = 200; //Drive.getMax().value();
  long minP = 200; //Drive.getMin().value();
  // distance traveled after 360 deg rotation = wheel circumference (assuming no slip)
  float wheelCircumf = 2 * piApprox * wheelRad; //centimeters
  long pRange = abs(maxP) + abs(minP); // wheel position range [ticks]
  float ratioTicksPerCm = pRange / wheelCircumf; //ticks per meter
  float ticksPerSecond = (float) cm * ratioTicksPerCm;
  //  Serial.print("wheel speed = ");
  //  Serial.print((long) ticksPerSecond);
  //  Serial.println(" ticks/s");

  return (long) ticksPerSecond; // [ticks / s]
}

void calculateAngleOppositeY() {
  angleOppositeY = atan2(yCoordinate, xCoordinate); // output in radians
  // Do we want to convert to degrees?
  /*Serial.print("angle opposite Y: ");
  Serial.print(angleOppositeY);
  Serial.println(" rad");*/
}

void calculateAverageOfCoordinates() {
  averageOfXAndY = (xCoordinate + yCoordinate) / 2;
  /*Serial.print("average of X and Y: ");
  Serial.print(averageOfXAndY);
  Serial.println(" cm");*/
}

void calculateDifferenceBetweenYAndAvg() {
  differenceFromAverage = abs(yCoordinate - averageOfXAndY); // [cm]
  /*Serial.print("difference between average and y coordinate: ");
  Serial.print(differenceFromAverage);
  Serial.println(" cm");*/
}

void calculateAngleOppositeY2() {
  angleOppositeY2 = atan2(differenceFromAverage, xCoordinate); // [rad]
  /*Serial.print("angle opposite y_2: ");
  Serial.print(angleOppositeY2);
  Serial.println(" rad");*/
}

void calculateAngleOppositeY1() {
  angleOppositeY1 = angleOppositeY - angleOppositeY2; // [rad]
  /*Serial.print("angle opposite y_1: ");
  Serial.print(angleOppositeY1);
  Serial.println(" rad");*/
}

void calculateAngleOppositeX() {
  angleOppositeX = piApprox / 2 - angleOppositeY; // [rad]
  /*Serial.print("angle opposite X: ");
  Serial.print(angleOppositeX);
  Serial.println(" rad");*/
  
}

void calculateArcAngle() {
  arcAngle = piApprox - angleOppositeX - angleOppositeY1; // [rad]
  arcAngle = arcAngle * (180 / piApprox); // [deg]
  /*Serial.print("arc angle: ");
  Serial.print(arcAngle);
  Serial.println(" deg");*/
}

void calculateArcRadius() {
  arcRadius = averageOfXAndY; // [cm]
  /*Serial.print("arc radius: ");
  Serial.print(arcRadius);
  Serial.println(" cm");*/
}

void calculateMoveDist() {
  moveDist = calculateArcLength(arcAngle, arcRadius); // [cm]
}

float calculateArcLength(float angle, float radius) { // parameters are floats because they can be decimal resultants of calculations
  float al = 2 * piApprox * radius * (angle / 360.0);
//  Serial.print("arc angle: ");
//  Serial.print(al);
//  Serial.println(" cm");
  return al;
}

void calculateMoveDuration() {
  moveDuration = moveDist / moveSpeed; // [s] = [cm] / [cm / s]
//  Serial.print("move duration: ");
//  Serial.print(moveDuration);
//  Serial.println(" seconds");
}

long calculateTurnSpeed() {
  // Get ratio of dist per sec to total dist
  float unitTurnAngle = arcAngle / moveDuration;
  float turnDist = calculateArcLength(unitTurnAngle, turnRad);
  long tsTicks = convertLinSpeedToWheelSpeed(turnDist);
//  Serial.print("turnSpeed [ticks / s]: ");
//  Serial.println(tsTicks);
  
  return tsTicks;
}

void stopTurning() {
  //Turn.s(0).wait();
  Serial.println("turn");
}

void stopMoving() {
  currentMillis = millis();
  while (millis() - currentMillis <= (stopDuration * 1000)) {
    //Drive.s(0).wait();
    Serial.println("drive");
    //Turn.s(0).wait();
    Serial.println("turn");
  }
}

void moveToCoordinates() {
  currentMillis = millis();
  do {
    //Drive.s(-driveSpeed).wait(); // Consider removing .wait() if code not working
    Serial.println("drive");
    if (ifTurnClockwise == false) {
      //Turn.s(-turnSpeed).wait(); // Consider removing .wait() if code not working
      Serial.println("turn");
    } else {
      //Turn.s(turnSpeed).wait(); // Consider removing .wait() if code not working
      Serial.println("turn");
    }
  } while (millis() - currentMillis <= (moveDuration * 1000));
}

void calculateTurningAngles() {
  clockwiseAngle = arcAngle - orientation;
  counterclockwiseAngle = 360.0 - clockwiseAngle;
}

void calculateTurnDist() {
  turnDist = calculateArcLength(clockwiseAngle, turnRad);
}

void calculateTurnDuration() {
  turnDuration = turnDist / turnSpeed; // [s] = [cm] / [cm / s]
}

void turnToOrientation() {
  currentMillis = millis();
  if (counterclockwiseAngle < clockwiseAngle) {
    // turn counterclockwise
    do {
      //Drive.s(turnSpeed).wait();
      Serial.println("drive");
    } while (millis() - currentMillis <= (turnDuration * 1000));
  } else {
    // turn clockwise
    do {
      //Drive.s(-turnSpeed).wait();
      Serial.println("drive");
    } while (millis() - currentMillis <= (turnDuration * 1000));
  }
}

//This function takes an angle X (in degrees) and turns the robot clockwise X degrees
void turnClockwise(float angle) {
  //clockwise is a positive turn speed

  float turnCircum = 2 * piApprox * turnRad; // circumference in ticks
  float turnDist = (angle / 360.0) * turnCircum; // [cm] = [deg / deg] * [cm]
  float turnTime = turnDist / moveSpeed; // [s] = [cm] / [cm / s]
  turnSpeed = convertLinSpeedToWheelSpeed(moveSpeed); // Convert input speed [cm / s] to wheel rotational speed [ticks / s]

  while (millis() - currentMillis <= (turnTime * 1000)) {
      //Turn.s(turnSpeed).wait();
      Serial.println("turn");
    }
}

//This function takes an angle X (in degrees) and turns the robot counterclockwise X degrees
void turnCounterClockwise(float angle) {
  //counterclockwise is a negative turn speed

  float turnCircum = 2 * piApprox * turnRad; // circumference in ticks
  float turnDist = (angle / 360.0) * turnCircum; // [cm] = [deg / deg] * [cm]
  float turnTime = turnDist / moveSpeed; // [s] = [cm] / [cm / s]
  turnSpeed = convertLinSpeedToWheelSpeed(moveSpeed); // Convert input speed [cm / s] to wheel rotational speed [ticks / s]

  //see calculateMoveDuration(turnDist) to calculate turnDuration
  while (millis() - currentMillis <= (turnTime * 1000)) {
      //Turn.s(-turnSpeed).wait();
      Serial.println("turn");
    }
}

/*
   I've only had succes with the Kangaroo when I convert inputs to "ticks"
   (it's default unit) before doing calculations. It might have to do with
   the fact that the company makes money on software to customize your units.
*/
long convertCmToTicks(float cm) {
  long maxP = 200; //Drive.getMax().value();
  long minP = -200; //Drive.getMin().value();
  float wheelDiam = 6.5; //centimeters
  // distance traveled after 360 deg rotation = wheel circumference (assuming no slip)
  float wheelCircumf = 2 * 3.14159 * (wheelDiam / 2); //centimeters
  long pRange = abs(maxP) + abs(minP); // wheel position range [ticks]
  float ratioTicksPerCm = pRange / wheelCircumf; //ticks per meter
  float ticks = cm * ratioTicksPerCm;
  return (long) ticks;
}

// For if we allow user to dictate turning angle
long convertDegToTicks(int deg) {
  long maxP = 200; //Drive.getMax().value();
  long minP = -200; //Drive.getMin().value();
  long pRange = abs(maxP) + abs(minP); // wheel position range [ticks]
  long ratioTicksPerDeg = pRange / 360; // [ticks / deg]
  long ticks = deg * ratioTicksPerDeg;

  return ticks;
}

//determine distance between nearest object and given sensor:
float getDist(int pin) {
  //establish variables for duration of the ping and the distance results in inches
  unsigned long timeout = 18500; //microseconds to wait before deciding object out of range
  float ping_duration; //time required to send and receive sound wave
  //float dist_inches; //inches between nearest object and given sensor
  float dist_cm; //centimeters between nearest object and given sensor
  //The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  //Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
  ping_duration = pulseIn(pin, HIGH, timeout); //determine how long it took to send and receive sound wave
  //dist_inches = ping_duration / 73.746 / 2.0; //convert time to inches based on speed of sound
  dist_cm = ping_duration / 29.034 / 2.0; //convert time to centimeters based on speed of sound

  delay(100); //required delay before next measurement
  return dist_cm;
}
