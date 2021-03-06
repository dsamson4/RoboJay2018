#include <SoftwareSerial.h> 
#include <Kangaroo.h> 
#include <math.h>

//=====Constants=====
const float piApprox = 3.14159;
const byte numInputs = 6; // Number of user-defined variables

//=====for tracking time=====
unsigned long currentMillis; // [ms]

//=====variables given by user=====
boolean useDefaultSettings = true; // The program will use the default settings. Set true because true intuitively relates to default.
int sideLength = 40; // [cm]
int arcRadius = 13; // [cm]
int moveSpeed = 11; // [cm / s]
int iterations = 3; // number of times robot zigs or zags
boolean ifTurnClockwise = true; // The robot will move clockwise.
int arcAngle = 90; // [degrees].
int input = 0; // variable that temporarily holds user input for manipulation before reassigning

//====for user question=====
const char questionIfDefault[] = "Would you like to use the default settings, 'y' or 'n'? (y = yes, n = no)"; // question asking if user wants default settings
const char questionSideLength[] = "What side length should the zigzag have? (units: centimeters [cm])";
const char questionArcRadius[] = "What radius should the corners of the square have? (units: centimeters [cm])";
const char questionAngle[] = "What angle should the zigzag be at? (y = yes, n = no)";
const char questionSpeed[] = "How fast should the robot move? (units: centimeters per second [cm / s])";
const char questionIterations[] = "How many times should the robot zigzag?"; // question asking for user desired iterations
boolean waitingForResponse = false;
char* questions[numInputs + 1]; // an array with # of elements equal to # of inputs from user + 1 for 0 (remember to add 1 element when no longer square)
char* question;

//=====for user response=====
const byte buffSize = 47;
char userResponseIfDefault[buffSize];
char userResponseSideLength[buffSize];
char userResponseArcRadius[buffSize];
char userResponseArcAngle[buffSize];
char userResponseSpeed[buffSize];
char userResponseIterations[buffSize]; // don't know size ahead of time so assume won't be larger than largest question

const char endMarker = '\r';
byte bytesRecvd = 0;
boolean ackResponse = false; // There is a response that needs acknowledgment.

//=====for LEDs=====
const byte ledGPin = 6;
const byte ledRPin = 7;
const unsigned long ledOnMillis = 300; // [ms]
const unsigned long ledGBaseInterval = 500; // [ms]
unsigned long ledGInterval = ledGBaseInterval;
byte ledGState = HIGH;
byte ledRState = HIGH;
unsigned long prevLedGMillis; // [ms]

//=====for pushbutton switch=====
const byte buttonPin = 5;
byte buttonState;

//=====for the Kangaroo=====
const byte txPin = 11;
const byte rxPin = 10;
SoftwareSerial serialPort (rxPin,txPin);
KangarooSerial K (serialPort);
KangarooChannel Drive (K, 'D');
KangarooChannel Turn (K,'T');

//=====for determining drive commands=====
long driveSpeed; // [ticks / s]
float turnDuration; // [s]
float turnDist; // [cm]
long turnSpeed; // [ticks / s]
long wheelSpeed; // [ticks / s]
float forwardDuration; // [s]
float arcDuration; // [s]
float forwardDist; // [cm]
float arcDist; // [cm]
const byte stopDuration = 1; // [s]

//=====Robot Dimensions=====
const float turnRad = 9.875; // [cm]
const byte wheelDiam = 65; // [mm]
const float wheelRad = ((float) wheelDiam / 2) / 10; // [cm]

void setup() {
  // put your setup code here, to run once:
  initKangaroo();
  pinMode(ledGPin, OUTPUT);
  pinMode(ledRPin, OUTPUT);
  digitalWrite(ledGPin, HIGH);
  digitalWrite(ledRPin, HIGH);

  pinMode(buttonPin, INPUT_PULLUP);
  
  Serial.begin(9600);
  Serial.println("Starting move_rounded_zigzag.ino");

  questions[0] = questionIfDefault;
  questions[1] = questionSideLength;
  questions[2] = questionArcRadius; 
  questions[3] = questionAngle;
  questions[4] = questionSpeed;
  questions[5] = questionIterations;

  delay(3000);  
  flipLedG(); // Turns LEDg off, leaving LEDr on
}

void loop() {
  //askGetAck(); // Ask for, get, and acknowledge all pre-trial user settings
  flipLedR(); // Turns LEDr off, leaving LEDg off

  driveSpeed = convertLinSpeedToWheelSpeed(moveSpeed); // Convert input speed [cm / s] to wheel rotational speed [ticks / s]

  // Set drive commands
  forwardDuration = calculateMoveDuration((float) sideLength); // outputs time in seconds

 // Set straight line commands
  calculateForwardDist();
  calculateForwardDuration(); // outputs time in seconds

  // Set arc commands
  calculateArcDist();
  calculateArcDuration(); // outputs time in seconds
  turnSpeed = calculateTurnSpeed();
  /*
  turnDist = calculateTurnDist((float) arcAngle); // outputs displacement in centimeters
  turnDuration = calculateMoveDuration(turnDist); // outputs time in seconds based on input in cm
  turnSpeed = convertLinSpeedToWheelSpeed(moveSpeed); // Convert input speed [cm / s] to wheel rotational speed [ticks / s]
  */

  waitForButtonPress(); // Make sure user is ready to observe
  flipLedG(); // Turns LEDg on to indicate moving, leaving LEDr off

  for (byte i = 0; i < iterations; i++) {
    moveForward();
    moveAlongArc();
    stopTurning();
  }
  stopMoving();
  flipLedG(); // Turns LEDg off, leaving LEDr on
  flipLedR(); // Turns LEDr on, leaving LEDg off
  delay(2000);
}

void initKangaroo() {
  serialPort.begin(9600);
  serialPort.listen();
  Drive.start();
  Turn.start();
  Drive.si(0);
  Turn.si(0);
  Serial.println("Done initializing Kangaroo");
}

void askGetAck() {
  askGetAckIfDefault();
  if (useDefaultSettings == false) {
    askGetAckSettings();
  }
}

void askGetAckIfDefault() {
  do {
    askQuestion(0);
    getResponse(0);
  } while (ackResponse == false);
  acknowledgeResponse(0);
}

void askGetAckSettings() {
  if (useDefaultSettings == true) {
    return;
  }
  
  for (byte i = 1; i < numInputs; i++) {
    do {
      askQuestion(i);
      getResponse(i);
    } while (ackResponse == false);
    acknowledgeResponse(i);
  }
}

void askQuestion(byte qNum) {
  if (waitingForResponse == true) {
    return;
  }
  
  // (else clause unnecessary because of use of return above)
  question = questions[qNum]; // get the array address
  Serial.println(question);
  waitingForResponse = true;
  /* The end of an ask function (i.e. here) is the best place to reset received bytes, 
     just in case other parts of the code need to know # of bytes actually received */
  bytesRecvd = 0;
}

void getResponse(byte rNum) {
  if (waitingForResponse == false) {
     return;
  }

  if(Serial.available() == 0) {
    return;
  }
  
  char inChar = Serial.read();

  if (rNum == 0) { // get if defauilt
    if (inChar != endMarker) {
      if (inChar == 'n') {
        useDefaultSettings = false;
      } else if (inChar == 'y') {
        useDefaultSettings = true;
      } else {
        useDefaultSettings = true;
      }
      userResponseIfDefault[bytesRecvd] = inChar;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1; // because of 0 that compiler auto adds
      }
    } else { // inChar is the endMarker
      waitingForResponse = false;
      userResponseIfDefault[bytesRecvd] = 0;
      ackResponse = true;
    }
  } else if (rNum == 1) { // get side length
    if (inChar != endMarker) {
      int c = (int)inChar - 48;
      input *= 10;
      input += c;
      userResponseSideLength[bytesRecvd] = inChar;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1; // because of 0 that compiler auto adds
      }
    } else { // inChar is the endMarker
      sideLength = input;
      input = 0; // Reset input for next question
      waitingForResponse = false;
      userResponseSideLength[bytesRecvd] = 0;
      ackResponse = true;
    }
  } else if (rNum == 2) { // get arc radius
    if (inChar != endMarker) {
      int c = (int)inChar - 48;
      input *= 10;
      input += c;
      userResponseArcRadius[bytesRecvd] = inChar;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1; // because of 0 that compiler auto adds
      }
    } else { // inChar is the endMarker
      arcRadius = input;
      //      Serial.print("arc radius set to ");
      //      Serial.println(arcRadius);
      input = 0; // Reset input for next question
      waitingForResponse = false;
      userResponseArcRadius[bytesRecvd] = 0;
      ackResponse = true;
    }
  } else if (rNum == 3) { // get angle
    if (inChar != endMarker) {
      int c = (int)inChar - 48;
      input *= 10;
      input += c;
      userResponseArcAngle[bytesRecvd] = inChar;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1; // because of 0 that compiler auto adds
      }    
    } else { // inChar is the endMarker
      arcAngle = input;
      input = 0; // Reset input for next question
      waitingForResponse = false;
      userResponseArcAngle[bytesRecvd] = 0;
      ackResponse = true;
    }
  } else if (rNum == 4) { // get velocity
    if (inChar != endMarker) {
      int c = (int)inChar - 48;
      input *= 10;
      input += c;
      userResponseSpeed[bytesRecvd] = inChar;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1; // because of 0 that compiler auto adds
      }
    } else { // inChar is the endMarker
      moveSpeed = input;
      input = 0; // Reset input for next question
      waitingForResponse = false;
      userResponseSpeed[bytesRecvd] = 0;
      ackResponse = true;
    }
  } else if (rNum == 5) { // get iterations
    if (inChar != endMarker) {
      int c = (int)inChar - 48;
      input *= 10;
      input += c;
      userResponseIterations[bytesRecvd] = inChar;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1; // because of 0 that compiler auto adds
      }
    } else { // inChar is the endMarker
      iterations = input;
      input = 0; // Reset input for next question
      waitingForResponse = false;
      userResponseIterations[bytesRecvd] = 0;
      ackResponse = true;
    }
  }
}

void acknowledgeResponse(byte rNum) {
  if (ackResponse == false) {
    return;
  }

  if (rNum == 0) { // acknowledge if default
    Serial.print("The robot will use ");
    if (useDefaultSettings == true) {
      Serial.print("the default");
    } else {
      Serial.print("your custom");
    }
    Serial.println(" settings.");
  }
  if (rNum == 1) { // acknowledge iterations
    Serial.print("The robot will zigzag ");
    Serial.print(userResponseIterations);
    if (iterations == 1) {
      Serial.println(" time.");
    } else {
      Serial.println(" times.");
    }
  } else if (rNum == 2) { // acknowledge side length
    Serial.print("The zigzag will have a side length of ");
    Serial.print(userResponseSideLength);
    Serial.println(" centimeters [cm].");
  } else if (rNum == 3) { // acknowledge angle
    Serial.print("The zigzag turns will be at an angle of ");
    Serial.print(userResponseArcAngle);
    Serial.println(" degrees.");
  } else if (rNum == 4) { // acknowledge move velocity
    Serial.print("The robot will move at a speed of ");
    Serial.print(userResponseSpeed);
    if (moveSpeed == 1) {
      Serial.println(" centimeter per second [cm / s].");
    } else {
      Serial.println(" centimeters per second [cm / s].");
    }
  }
  Serial.println();

  ackResponse = false;

// IF INTERESTED IN NUMBER OF BYTES RECEIVED
//  Serial.print("You entered ");
//  Serial.print(bytesRecvd);
//  if (bytesRecvd == 1) {
//    Serial.print(" byte: ");
//  } else {
//    Serial.print(" bytes: ");
//  }
}

float calculateMoveDuration(float dist) {
  float moveDuration = dist / moveSpeed; // [s] = [cm] / [cm / s]
//  Serial.print("move duration: ");
//  Serial.print(moveDuration);
//  Serial.println(" seconds");
  
  return moveDuration;
}

long convertLinSpeedToWheelSpeed(byte cm) { // linear speed is given by user
  long maxP = Drive.getMax().value();
  long minP = Drive.getMin().value();
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

void calculateForwardDist() { // for instances with straight lines and curves
  forwardDist = sideLength - 2 * arcRadius;
}

void calculateForwardDuration() {
  forwardDuration = forwardDist / moveSpeed; // [s] = [cm] / [cm / s]
  //  Serial.print("move duration: ");
  //  Serial.print(moveDuration);
  //  Serial.println(" seconds");
}

void calculateArcDist() { // for instances with straight lines and curves
  arcDist = calculateArcLength((float) arcAngle, (float) arcRadius); // [cm]
  Serial.print("arc dist: ");
  Serial.print(arcDist);
  Serial.println(" cm");
}

float calculateArcLength(float angle, float radius) { // parameters are floats because they can be decimal resultants of calculations
  float al = 2 * piApprox * radius * (angle / 360.0);
//  Serial.print("arc angle: ");
//  Serial.print(al);
//  Serial.println(" cm");
  return al;
}

void calculateArcDuration() {
  arcDuration = arcDist / moveSpeed; // [s] = [cm] / [cm / s]
  Serial.print("arc duration: ");
  Serial.print(arcDuration);
  Serial.println(" seconds");
}

long calculateTurnSpeed() {
  // Get ratio of dist per sec to total dist
  float unitTurnAngle = arcAngle / arcDuration;
  float turnDist = calculateArcLength(unitTurnAngle, turnRad);
  long tsTicks = convertLinSpeedToWheelSpeed(turnDist);
  Serial.print("tsTicks [ticks / s]: ");
  Serial.println(tsTicks);
  
  return tsTicks;
}
/*
float calculateTurnDist(float turnAngle) {
  float turnCircum = calcTurnCircumTicks(); // circumference in ticks
  float td = (turnAngle / 360.0) * turnCircum; // [cm] = [deg / deg] * [cm]
//  Serial.print("turn dist = ");
//  Serial.print(td);
//  Serial.println(" cm");

  return td;
} */

float calcTurnCircumTicks() {
  float circumference = 2 * piApprox * turnRad;
//  Serial.print("circum = ");
//  Serial.print(circumference);
//  Serial.println(" cm");
  
  return circumference;
}

void waitForButtonPress() {
  // Serial.println("Press button to start moving");
  
  do {
    currentMillis = millis();
    flashLedG(); // Flashes LEDg to indicate waiting for button press before moving
    buttonState = digitalRead(buttonPin);
  } while (buttonState == HIGH); // While button is not pressed
  
  // Serial.println("button pressed");
}

void flipLedG() {
  ledGState = ! ledGState;
  digitalWrite(ledGPin, ledGState);
}

void flipLedR() {
  ledRState = ! ledRState;
  digitalWrite(ledRPin, ledRState);
}

void moveForward() {
  currentMillis = millis();
  do {
    Drive.s(-driveSpeed).wait();
  } while (millis() - currentMillis <= (forwardDuration * 1000));
}

void stopTurning() {
  Turn.s(0).wait();
}

void moveAlongArc() {
  currentMillis = millis();
  do {
    Drive.s(-driveSpeed).wait(); // Consider removing .wait() if code not working
    if (ifTurnClockwise == false) {
      Turn.s(-turnSpeed).wait(); // Consider removing .wait() if code not working
    } else {
      Turn.s(turnSpeed).wait(); // Consider removing .wait() if code not working
    }
  } while (millis() - currentMillis <= (arcDuration * 1000));
  ifTurnClockwise = !ifTurnClockwise;
}

void stopMoving() {
  currentMillis = millis();
  while (millis() - currentMillis <= (stopDuration * 1000)) {
    Drive.s(0).wait();
    Turn.s(0).wait();
  }
}

void flashLedG() {
  if (currentMillis - prevLedGMillis >= ledGInterval) {
    prevLedGMillis += ledGInterval; // This updates the timing ready for the next interval (more accurately than using prevLedGMillis = currentLedMillis)
    ledGState = ! ledGState;
    if (ledGState == HIGH) {
      ledGInterval = ledOnMillis;
    } else {
      ledGInterval = ledGBaseInterval;
    }
    digitalWrite(ledGPin, ledGState);
  }
}
