#include <SoftwareSerial.h>
#include <Kangaroo.h>

//=====Constants=====
const float piApprox = 3.14159;
const byte numInputs = 6; // Number of user-defined variables

//=====for tracking time=====
unsigned long currentMillis; // [ms]

//=====variables given by user=====
boolean useDefaultSettings = true; // The program will use the default settings. Set true because true intuitively relates to default.
int moveRadius = 30; // [cm]
int moveSpeed = 12; // [cm / s]
int arcAngle = 360; // [deg], default is full circle
int iterations = 1; // number of times robot moves around rectangle
boolean ifTurnClockwise = true; // The robot will move clockwise.
//boolean ifMoveForward = true; // The robot will move forward.
int input = 0; // variable that temporarily holds user input for manipulation before reassigning

//=====for user question=====
const char questionIfDefault[] = "Would you like to use the default settings, 'y' or 'n'? (y = yes, n = no)"; // question asking if user wants default settings
const char questionArcAngle[] = "What angle should the arc have? (units: degrees [deg])";
const char questionMoveRadius[] = "What radius should the circle have? (units: centimeters [cm])";
const char questionIfClockwise[] = "Should the robot move clockwise, 'y' or 'n'? (y = yes, n = no)";
const char questionSpeed[] = "How fast should the robot move? (units: centimeters per second [cm / s])";
const char questionIterations[] = "How many times should the robot move along the arc?"; // question asking for user desired iterations
boolean waitingForResponse = false;
char* questions[numInputs + 1]; // an array with # of elements equal to # of inputs from user + 1 for 0 (remember to add 1 element when no longer square)
char* question;

//=====for user response=====
const byte buffSize = 47;
char userResponseIfDefault[buffSize];
char userResponseArcAngle[buffSize];
char userResponseMoveRadius[buffSize];
char userResponseIfClockwise[buffSize];
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
long wheelSpeed; // [ticks / s]
float moveDuration; // [s]
float moveDist; // [cm]
const byte stopDuration = 1; // [s]
long driveSpeed; // [ticks / s]
long turnSpeed; // [ticks / s]

//=====Robot Dimensions=====
const float turnRad = 9.875; // [cm]
const byte wheelDiam = 65; // [mm]
const float wheelRad = ((float) wheelDiam / 2) / 10; // [cm]

void setup() {
  initKangaroo();

  pinMode(ledGPin, OUTPUT);
  pinMode(ledRPin, OUTPUT);
  digitalWrite(ledGPin, HIGH);
  digitalWrite(ledRPin, HIGH);

  pinMode(buttonPin, INPUT_PULLUP);
  
  Serial.begin(9600);
  Serial.println("Starting move_arc.ino");

  questions[0] = questionIfDefault;
  questions[1] = questionArcAngle;
  questions[2] = questionMoveRadius;
  questions[3] = questionIfClockwise;
  questions[4] = questionSpeed;
  questions[5] = questionIterations;

  delay(3000);  
  flipLedG(); // Turns LEDg off, leaving LEDr on
}

void loop() {
  //askGetAck(); // Ask for, get, and acknowledge all user input
  flipLedR(); // Turns LEDr off, leaving LEDg off

  calculateMoveDuration(); // Determine using user input speed [cm] and circle radius [cm]
  // Set move speeds
  driveSpeed = convertLinSpeedToWheelSpeed((float) moveSpeed); // Convert input speed [cm / s] to wheel rotational drive speed [ticks / s]
//  Serial.print("drive speed: ");
//  Serial.print(driveSpeed);
//  Serial.println(" ticks per second");
  turnSpeed = calculateTurnSpeed(); // Use drive speed [cm] and circle radius [cm] to output wheel rotational turn speed [ticks / s]
//  Serial.print("turn speed: ");
//  Serial.print(turnSpeed);
//  Serial.println(" ticks per second");

  waitForButtonPress(); // Make sure user is ready to observe
  flipLedG(); // Turns LEDg on to indicate moving, leaving LEDr off
  
  // Move along arc i times
  for (byte i = 0; i < iterations; i++) {
    moveAlongArc();
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
  } else if (rNum == 1) { // get arc angle
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
      arcAngle = input; // Set variable equal to user input
//      Serial.print("arc radius set to ");
//      Serial.println(arcAngle);
      input = 0; // Reset input for next question
      waitingForResponse = false;
      userResponseArcAngle[bytesRecvd] = 0;
      ackResponse = true;
    }
  } else if (rNum == 2) { // get arc radius
    if (inChar != endMarker) {
      int c = (int)inChar - 48;
      input *= 10;
      input += c;
      userResponseMoveRadius[bytesRecvd] = inChar;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1; // because of 0 that compiler auto adds
      }
    } else { // inChar is the endMarker
      moveRadius = input;
//      Serial.print("arc radius set to ");
//      Serial.println(moveRadius);
      input = 0; // Reset input for next question
      waitingForResponse = false;
      userResponseMoveRadius[bytesRecvd] = 0;
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
  } else if (rNum == 1) { // acknowledge arc angle
    Serial.print("The arc will have an angle of ");
    Serial.print(userResponseArcAngle);
    Serial.println(" degrees [deg].");
  } else if (rNum == 2) { // acknowledge move radius
    Serial.print("The arc will have a radius of ");
    Serial.print(userResponseMoveRadius);
    Serial.println(" centimeters [cm].");
  } else if (rNum == 3) { // acknowledge if clockwise
    Serial.print("The robot will move ");
    if (ifTurnClockwise == true) {
      Serial.print("CLOCKWISE");
    } else {
      Serial.print("COUNTERCLOCKWISE");
    }
    Serial.println(" along the arc.");
  } else if (rNum == 4) { // acknowledge move velocity
    Serial.print("The robot will move at a speed of ");
    Serial.print(userResponseSpeed);
    Serial.println(" centimeters per second [cm / s].");
  } else if (rNum == 5) { // acknowledge iterations
    Serial.print("The robot will move along the arc ");
    Serial.print(userResponseIterations);
    if (iterations == 1) {
      Serial.println(" time.");
    } else {
      Serial.println(" times.");
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

void calculateMoveDuration() {
  moveDist = calculateArcLength((float) arcAngle, (float) moveRadius); // [cm]
  Serial.print("move dist: ");
  Serial.print(moveDist);
  Serial.println(" cm");
  moveDuration = moveDist / moveSpeed; // [s] = [cm] / [cm / s]
  Serial.print("move duration: ");
  Serial.print(moveDuration);
  Serial.println(" seconds");
}

float calculateArcLength(float angle, float radius) { // parameters are floats because they can be decimal resultants of calculations
  float al = 2 * piApprox * radius * (angle / 360.0);
//  Serial.print("arc angle: ");
//  Serial.print(al);
//  Serial.println(" cm");
  return al;
}

long convertLinSpeedToWheelSpeed(float cmPerSecond) { // linear speed is given by user
  long maxP = Drive.getMax().value();
  long minP = Drive.getMin().value();
  // distance traveled after 360 deg rotation = wheel circumference (assuming no slip)
  float wheelCircumf = 2 * piApprox * wheelRad; //centimeters
  long pRange = abs(maxP) + abs(minP); // wheel position range [ticks]
  float ratioTicksPerCm = pRange / wheelCircumf; //ticks per meter
  float ticksPerSecond = cmPerSecond * ratioTicksPerCm;
//  Serial.print("wheel speed = ");
//  Serial.print((long) ticksPerSecond);
//  Serial.println(" ticks/s");

  return (long) ticksPerSecond; // [ticks / s]
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

//long convertRadiansToTicks(float angleRadians) {
//  long maxP = Drive.getMax().value();
//  long minP = Drive.getMin().value();
//  long pRange = abs(maxP) + abs(minP); // wheel position range [ticks]
//  long ratioTicksPerRadian = pRange / (2 * piApprox); // [ticks / deg]
//  long ticks = angleRadians * ratioTicksPerRadian;
//  
//  return ticks;
//}

void waitForButtonPress() {
  // Serial.println("Press button to start moving");
  
  do {
    currentMillis = millis();
    flashLedG(); // Flashes LEDg to indicate waiting for button press before moving
    buttonState = digitalRead(buttonPin);
  } while (buttonState == HIGH); // While button is not pressed
  
  // Serial.println("button pressed");
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
  } while (millis() - currentMillis <= (moveDuration * 1000));
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
