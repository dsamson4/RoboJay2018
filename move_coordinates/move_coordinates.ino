#include <SoftwareSerial.h>
#include <Kangaroo.h>

//=====Constants=====
const float piApprox = 3.14159;
const byte numInputs = 6; // Number of user-defined variables

//=====for tracking time=====
unsigned long currentMillis; // [ms]
int currentOrientation = 0; // [deg]
int currentXCoordinate = 0; // [cm]
int currentYCoordinate = 0; // [cm]

//=====variables given by user=====
boolean useDefaultSettings = true; // The program will use the default settings. Set true because true intuitively relates to default.
int xCoordinate = 50; // [cm]
int yCoordinate = 15; // [cm]
int orientation = 45; // [deg]
int moveSpeed = 10; // [cm / s]
int numPositions = 1; // number of positions the robot moves to in succession
//boolean ifTurnClockwise = true; // The robot will move clockwise.
//boolean ifMoveForward = true; // The robot will move forward.
int input = 0; // variable that temporarily holds user input for manipulation before reassigning

//=====for user question=====
const char questionIfDefault[] = "Would you like to use the default settings, 'y' or 'n'? (y = yes, n = no)"; // question asking if user wants default settings
const char questionNumPositions[] = "To how many positions should the robot move in succession?"; // question asking for user desired number of positions to move to in succession
const char questionXCoordinate[] = "What should be the x coordinate? (units: centimeters [cm])";
const char questionYCoordinate[] = "What should be the y coordinate? (units: centimeters [cm])";
const char questionOrientation[] = "At what angle should the robot be oriented? (units: degrees [deg])";
const char questionSpeed[] = "How fast should the robot move? (units: centimeters per second [cm / s])";

boolean waitingForResponse = false;
char* questions[numInputs + 1]; // an array with # of elements equal to # of inputs from user + 1 for 0 (remember to add 1 element when no longer square)
char* question;

//=====for user response=====
const byte buffSize = 47;
char userResponseIfDefault[buffSize];
char userResponseNumPositions[buffSize]; // don't know size ahead of time so assume won't be larger than largest question
char userResponseXCoordinate[buffSize];
char userResponseYCoordinate[buffSize];
char userResponseOrientation[buffSize];
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
SoftwareSerial serialPort (rxPin, txPin);
KangarooSerial K (serialPort);
KangarooChannel Drive (K, 'D');
KangarooChannel Turn (K, 'T');

//=====for determining drive commands=====
long wheelSpeed; // [ticks / s]
float moveDuration; // [s]
float moveDist; // [cm]
const byte stopDuration = 1; // [s]
long driveSpeed; // [ticks / s]
long turnSpeed; // [ticks / s]

//=====for determining motion path=====
float arcAngle;
float chordLength;
float arcRadius;
float clockwiseAngle;
float counterclockwiseAngle;

//=====Robot Dimensions=====
const float turnRad = 9.875; // [cm]
const byte wheelDiam = 65; // [mm]
const float wheelRad = ((float) wheelDiam / 2) / 10; // [cm]

void setup() {
  initKangaroo(); //comment this out for testing without motion

  pinMode(ledGPin, OUTPUT);
  pinMode(ledRPin, OUTPUT);
  digitalWrite(ledGPin, HIGH);
  digitalWrite(ledRPin, HIGH);

  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("Starting move_coordinates.ino");

  questions[0] = questionIfDefault;
  questions[1] = questionNumPositions;
  questions[2] = questionXCoordinate;
  questions[3] = questionYCoordinate;
  questions[4] = questionOrientation;
  questions[5] = questionSpeed;

  delay(3000);
  flipLedG(); // Turns LEDg off, leaving LEDr on
}

void loop() {
  askGetAck(); // Ask for, get, and acknowledge all pre-trial user settings
  flipLedR(); // Turns LEDr off, leaving LEDg off

  waitForButtonPress(); // Make sure user is ready to observe
  flipLedG(); // Turns LEDg on to indicate moving, leaving LEDr off
  
  // Move to i positions in succession
  for (int i = 0; i < numPositions; i++) {
    // If either the x or the y coordinate is unchanged...
    if (currentXCoordinate - xCoordinate == 0 || currentYCoordinate - yCoordinate == 0) {
      // If the orientation has changed...
      if (currentOrientation != orientation) {
        calculateTurningAngles();
        turnToOrientation();
      }
    } else if (currentXCoordinate - xCoordinate != 0 && currentYCoordinate - yCoordinate == 0) { // If only the x coordinate has changed...
      
    } else if (currentXCoordinate - xCoordinate == 0 && currentYCoordinate - yCoordinate != 0) { // If only the y coordinate has changed...
      
    } else {
      // Calculate geometry
      calculateArcAngle(); 
      calculateChordLength();
      calculateArcRadius();
    
      // Determine move commands
      driveSpeed = convertLinSpeedToWheelSpeed((byte) moveSpeed); // Convert input speed [cm / s] to wheel rotational drive speed [ticks / s]
      calculateMoveDist();
      calculateMoveDuration(); // outputs time in seconds
      turnSpeed = calculateTurnSpeed();

      moveToCoordinates();
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

  if (Serial.available() == 0) {
    return;
  }

  char inChar = Serial.read();

  if (rNum == 0) { // get if default
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
  } else if (rNum == 1) { // get number of positions to move to in succession
    if (inChar != endMarker) {
      int c = (int)inChar - 48;
      input *= 10;
      input += c;
      userResponseNumPositions[bytesRecvd] = inChar;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1; // because of 0 that compiler auto adds
      }
    } else { // inChar is the endMarker
      numPositions = input;
      input = 0; // Reset input for next question
      waitingForResponse = false;
      userResponseNumPositions[bytesRecvd] = 0;
      ackResponse = true;
    }
  } else if (rNum == 2) { // get x coordinate
    if (inChar != endMarker) {
      int c = (int)inChar - 48;
      input *= 10;
      input += c;
      userResponseXCoordinate[bytesRecvd] = inChar;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1; // because of 0 that compiler auto adds
      }
    } else { // inChar is the endMarker
      xCoordinate = input;
      input = 0; // Reset input for next question
      waitingForResponse = false;
      userResponseXCoordinate[bytesRecvd] = 0;
      ackResponse = true;
    }
  } else if (rNum == 3) { // get y coordinate
    if (inChar != endMarker) {
      int c = (int)inChar - 48;
      input *= 10;
      input += c;
      userResponseYCoordinate[bytesRecvd] = inChar;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1; // because of 0 that compiler auto adds
      }
    } else { // inChar is the endMarker
      yCoordinate = input;
      //      Serial.print("y coordinate set to ");
      //      Serial.println(yCoordinate);
      input = 0; // Reset input for next question
      waitingForResponse = false;
      userResponseYCoordinate[bytesRecvd] = 0;
      ackResponse = true;
    }
  } else if (rNum == 4) { // get if clockwise
    if (inChar != endMarker) {
      int c = (int)inChar - 48;
      input *= 10;
      input += c;
      userResponseOrientation[bytesRecvd] = inChar;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1; // because of 0 that compiler auto adds
      }
    } else { // inChar is the endMarker
      orientation = input;
      //      Serial.print("y coordinate set to ");
      //      Serial.println(yCoordinate);
      input = 0; // Reset input for next question
      waitingForResponse = false;
      userResponseOrientation[bytesRecvd] = 0;
      ackResponse = true;
    }
  } else if (rNum == 5) { // get velocity
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
  } else if (rNum == 1) { // acknowledge number of positions
    Serial.print("The robot will move to ");
    Serial.print(userResponseNumPositions);
    if (numPositions == 1) {
      Serial.println(" position.");
    } else {
      Serial.println(" positions.");
    }
  } else if (rNum == 2) { // acknowledge x coordinate
    Serial.print("The x coordinate will be ");
    Serial.print(userResponseXCoordinate);
    if (xCoordinate == 1) {
      Serial.println(" centimeter [cm].");
    } else {
      Serial.println(" centimeters [cm].");
    }
  } else if (rNum == 3) { // acknowledge y coordinate
    Serial.print("The y coordinate will be ");
    Serial.print(userResponseYCoordinate);
    if (yCoordinate == 1) {
      Serial.println(" centimeter [cm].");
    } else {
      Serial.println(" centimeters [cm].");
    }
  } else if (rNum == 4) { // acknowledge orientation
    Serial.print("The robot will be oriented at an angle of ");
    Serial.print(userResponseOrientation);
    if (orientation == 1) {
      Serial.println(" degree [deg].");
    } else {
      Serial.println(" degrees [deg].");
    }
  } else if (rNum == 5) { // acknowledge move velocity
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

long convertLinSpeedToWheelSpeed(byte cm) { // linear speed is given by user
  long maxP = Drive.getMax().value();
  long minP = Drive.getMin().value();
  // distance traveled after 360 deg rotation = wheel circumference (assuming no slip)
  float wheelCircumf = 2 * piApprox * wheelRad; //centimeters
  long pRange = abs(maxP) + abs(minP); // wheel position range [ticks]
  float ratioTicksPerCm = pRange / wheelCircumf; //ticks per meter
  float ticksPerSecond = (float) cm * ratioTicksPerCm;
  Serial.print("wheel speed = ");
  Serial.print((long) ticksPerSecond);
  Serial.println(" ticks/s");

  return (long) ticksPerSecond; // [ticks / s]
}

void calculateArcAngle() {
  arcAngle = orientation - currentOrientation;
  Serial.print("arc angle: ");
  Serial.print(arcAngle);
  Serial.println(" deg");
}

void calculateChordLength() {
  chordLength = sqrt(pow(xCoordinate, 2) + pow(yCoordinate, 2));
  Serial.print("chord length: ");
  Serial.print(chordLength);
  Serial.println(" cm");
}

void calculateArcRadius() {
  arcAngle = arcAngle * (piApprox / 180); // [rad]
  arcRadius = chordLength / (2 * sin(arcAngle / 2));
  Serial.print("arc radius: ");
  Serial.print(arcRadius);
  Serial.println(" cm");
}

void calculateMoveDist() {
  arcAngle = arcAngle * (180 / piApprox); // [deg]
  moveDist = calculateArcLength(arcAngle, arcRadius); // [cm]
  Serial.print("move dist: ");
  Serial.print(moveDist);
  Serial.println(" cm");
}

float calculateArcLength(float angle, float radius) { // parameters are floats because they can be decimal resultants of calculations
  float al = 2 * piApprox * radius * (angle / 360.0);
//  Serial.print("arc length: ");
//  Serial.print(al);
//  Serial.println(" cm");
  return al;
}

void calculateMoveDuration() {
  moveDuration = moveDist / moveSpeed; // [s]
  Serial.print("move duration: ");
  Serial.print(moveDuration);
  Serial.println(" s");
}

long calculateTurnSpeed() {
  // Get ratio of dist per sec to total dist
  float unitTurnAngle = arcAngle / moveDuration;
  float turnDist = calculateArcLength(unitTurnAngle, turnRad);
  long tsTicks = convertLinSpeedToWheelSpeed(turnDist);
  Serial.print("tsTicks [ticks / s]: ");
  Serial.println(tsTicks);
  
  return tsTicks;
}

void moveForward() {
  currentMillis = millis();
  do {
    Drive.s(-driveSpeed).wait();
  } while (millis() - currentMillis <= (moveDuration * 1000));
}

void stopTurning() {
  Turn.s(0).wait();
}

void stopMoving() {
  currentMillis = millis();
  while (millis() - currentMillis <= (stopDuration * 1000)) {
    Drive.s(0).wait();
    Turn.s(0).wait();
  }
}

void moveToCoordinates() {
  currentMillis = millis();
  do {
    Drive.s(-driveSpeed).wait(); // Consider removing .wait() if code not working
    Turn.s(turnSpeed).wait(); // Consider removing .wait() if code not working
  } while (millis() - currentMillis <= (moveDuration * 1000));
}

void turnToOrientation() {
  currentMillis = millis();
  if (counterclockwiseAngle < clockwiseAngle) {
    // turn counterclockwise
    do {
      Drive.s(turnSpeed).wait();
    } while (millis() - currentMillis <= (moveDuration * 1000));
  } else {
    // turn clockwise
    do {
      Drive.s(-turnSpeed).wait();
    } while (millis() - currentMillis <= (moveDuration * 1000));
  }
}

void calculateTurningAngles() {
  clockwiseAngle = arcAngle - orientation;
  counterclockwiseAngle = 360.0 - clockwiseAngle;
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

void flipLedG() {
  ledGState = ! ledGState;
  digitalWrite(ledGPin, ledGState);
}

void flipLedR() {
  ledRState = ! ledRState;
  digitalWrite(ledRPin, ledRState);
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

