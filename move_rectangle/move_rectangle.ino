#include <SoftwareSerial.h>
#include <Kangaroo.h>

//=====Constants=====
const float piApprox = 3.14159;
const byte numInputs = 5; // Number of user-defined variables
const int turnAngle = 90; // [deg]

//=====for tracking time=====
unsigned long currentMillis; // [ms]

//=====variables given by user=====
boolean useDefaultSettings = true; // The program will use the default settings. Set true because true intuitively relates to default.
byte sideLength = 30; // [cm]
byte moveSpeed = 12; // [cm / s]
byte iterations = 1; // number of times robot moves around rectangle
boolean ifTurnClockwise = true; // The robot will move clockwise.
//boolean ifMoveForward = true; // The robot will move forward.
byte input = 0; // variable that temporarily holds user input for manipulation before reassigning

//=====for user question=====
const char questionIfDefault[] = "Would you like to use the default settings, 'y' or 'n'? (y = yes, n = no)"; // question asking if user wants default settings
const char questionIterations[] = "How many times should the robot move around the square?"; // question asking for user desired iterations
const char questionSideLength[] = "What side length should the square have? (units: centimeters [cm])";
const char questionIfClockwise[] = "Should the robot move clockwise, 'y' or 'n'? (y = yes, n = no)";
const char questionSpeed[] = "How fast should the robot move? (units: centimeters per second [cm / s])";
boolean waitingForResponse = false;
char* questions[numInputs + 1]; // an array with # of elements equal to # of inputs from user + 1 for 0 (remember to add 1 element when no longer square)
char* question;

//=====for user response=====
const byte buffSize = 47;
char userResponseIfDefault[buffSize];
char userResponseIterations[buffSize]; // don't know size ahead of time so assume won't be larger than largest question
char userResponseSideLength[buffSize];
char userResponseIfClockwise[buffSize];
char userResponseSpeed[buffSize];
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

//=====for moving=====
// LEAVE COMMENTED CODE BECAUSE IT SHOWS HOW TO SET DEFAULT PATH
//const unsigned long driveBaseInterval = 1000;
//const unsigned long turnBaseInterval = 1000;
//unsigned long driveInterval = driveBaseInterval;
//unsigned long turnInterval = turnBaseInterval;
//unsigned long driveMillis = driveBaseInterval;
//unsigned long turnMillis = turnBaseInterval;
//byte driveState = LOW;
//byte turnState = LOW;
//unsigned long prevDriveMillis;
//unsigned long prevTurnMillis;
unsigned long currentMoveMillis;

//=====for determining drive commands=====
//byte wheelMin = 0;
//byte wheelMax = 360;
//byte wheelPosition = wheelMax;
//const byte orientationMin = 0;
//const byte orientationMax = 0;
//byte orientationPosition = orientationMin;
// long driveWheelRot; // [ticks]
// long turnWheelRot; // [ticks]
long wheelSpeed; // [ticks / s]
float driveDuration; // [s]
float turnDuration; // [s]
float driveDist; // [cm]
float turnDist; // [cm]
const byte stopDuration = 1; // [s]
long driveSpeed; // [ticks / s]
long turnSpeed; // [ticks / s]

//=====Robot Dimensions=====
const float turnRad = 14.25; // for nexus: 28.5, for green testbot: 9.875; // [cm]
const byte wheelDiam = 143; //for green testbot: 65; // [mm]
const float wheelRad = ((float) wheelDiam / 2) / 10; // [cm]

void setup() {
  initKangaroo();

  pinMode(ledGPin, OUTPUT);
  pinMode(ledRPin, OUTPUT);
  digitalWrite(ledGPin, HIGH);
  digitalWrite(ledRPin, HIGH);

  pinMode(buttonPin, INPUT_PULLUP);
  
  Serial.begin(9600);
  Serial.println("Starting move_square.ino");

  questions[0] = questionIfDefault;
  questions[1] = questionIterations;
  questions[2] = questionSideLength;
  questions[3] = questionIfClockwise;
  questions[4] = questionSpeed;

  delay(3000);  
  flipLedG(); // Turns LEDg off, leaving LEDr on
}

void loop() {
  askGetAck(); // Ask for, get, and acknowledge all pre-trial user settings
  flipLedR(); // Turns LEDr off, leaving LEDg off

  // Set drive commands
  driveDuration = calculateMoveDuration((float) sideLength); // outputs time in seconds
  driveSpeed = convertLinSpeedToWheelSpeed(moveSpeed); // Convert input speed [cm / s] to wheel rotational speed [ticks / s]

  // Set turn commands
  turnDist = calculateTurnDist(); // outputs displacement in centimeters
  turnDuration = calculateMoveDuration(turnDist); // outputs time in seconds based on input in cm
  turnSpeed = convertLinSpeedToWheelSpeed(moveSpeed); // Convert input speed [cm / s] to wheel rotational speed [ticks / s]

  waitForButtonPress(); // Make sure user is ready to observe
  flipLedG(); // Turns LEDg on to indicate moving, leaving LEDr off
  
  // Move in rectangle i times
  for (byte i = 0; i < iterations; i++) {
    // Move around 4 sides of rectangle
    for (byte j = 0; j < 4; j++) {
      moveForward();
      stopDriving();
      turnToAngle();
      stopTurning();
    }
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
  //Serial.println("Done initializing Kangaroo");
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
  }
  if (rNum == 1) { // get iterations
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
  } else if (rNum == 2) { // get side length
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
  } else if (rNum == 3) { // get if clockwise
    if (inChar != endMarker) {
      if (inChar == 'n') {
        ifTurnClockwise = false;
      } else if (inChar == 'y') {
        ifTurnClockwise = true;
      } else {
        Serial.println("Screw you. I'm picking clockwise.");
        ifTurnClockwise = true;
      }
      userResponseIfClockwise[bytesRecvd] = inChar;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1; // because of 0 that compiler auto adds
      }
    } else { // inChar is the endMarker
      waitingForResponse = false;
      userResponseIfClockwise[bytesRecvd] = 0;
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
    Serial.print("The robot will move around the square ");
    Serial.print(userResponseIterations);
    if (iterations == 1) {
      Serial.println(" time.");
    } else {
      Serial.println(" times.");
    }
  } else if (rNum == 2) { // acknowledge side length
    Serial.print("The square will have a side length of ");
    Serial.print(userResponseSideLength);
    Serial.println(" centimeters [cm].");
  } else if (rNum == 3) { // set if clockwise
    Serial.print("The robot will move ");
    if (ifTurnClockwise == true) {
      Serial.print("CLOCKWISE");
    } else {
      Serial.print("COUNTERCLOCKWISE");
    }
    Serial.println(" around the square.");
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

float calculateTurnDist() {
  float td = calculateArcLength(turnAngle, turnRad);
//  Serial.print("turn dist = ");
//  Serial.print(td);
//  Serial.println(" cm");

  return td;
}

float calculateArcLength(float angle, float radius) { // parameters are floats because they can be decimal resultants of calculations
  float al = 2 * piApprox * radius * (angle / 360.0);
//  Serial.print("arc length: ");
//  Serial.print(al);
//  Serial.println(" cm");
  return al;
}

void waitForButtonPress() {
  Serial.println("Press button to start....");
  do {
    currentMillis = millis();
    flashLedG(); // Flashes LEDg to indicate waiting for button press before moving
    buttonState = digitalRead(buttonPin);
  } while (buttonState == HIGH); // While button is not pressed
  Serial.println("Button pressed. Starting....");
}

void moveForward() {
  currentMillis = millis();
  do {
    Drive.s(-driveSpeed).wait();
  } while (millis() - currentMillis <= (driveDuration * 1000));
}

void turnToAngle() {
  // Angle due north, positive is CCW.
  // For example, 90 is 90 degrees CCW, and 270 is 90 degrees CW
  currentMillis = millis();
  //byte scaleFactor = 2;
  if (ifTurnClockwise == false) {
    while (millis() - currentMillis <= (turnDuration * 1000)) {
      Turn.s(-turnSpeed).wait();
    }
  } else {
    while (millis() - currentMillis <= (turnDuration * 1000)) {
      Turn.s(turnSpeed).wait();
    }
  }
}

void flipLedG() {
  ledGState = ! ledGState;
  digitalWrite(ledGPin, ledGState);
}

void flipLedR() {
  ledRState = ! ledRState;
  digitalWrite(ledRPin, ledRState);
}

void stopMoving() {
  currentMillis = millis();
  while (millis() - currentMillis <= (stopDuration * 1000)) {
    Drive.s(0).wait();
    Turn.s(0).wait();
  }
}

void stopTurning() {
  Turn.s(0).wait();
}

void stopDriving() {
  Drive.s(0).wait();
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

/*
 * I've only had succes with the Kangaroo when I convert inputs to "ticks" 
 * (it's default unit) before doing calculations. It might have to do with
 * the fact that the company makes money on software to customize your units.
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
