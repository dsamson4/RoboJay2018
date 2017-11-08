#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <SoftwareSerial.h>
#include <Kangaroo.h>

ros::NodeHandle nh;

// Arduino TX (pin 11) goes to Kangaroo S1
// Arduino RX (pin 10) goes to Kangaroo S2
// Arduino GND         goes to Kangaroo 0V
// Arduino 5V          goes to Kangaroo 5V (OPTIONAL, if you want Kangaroo to power the Arduino)
const int tx_pin = 11;
const int rx_pin = 10;

float wheelDiam = 0.3175; //meters
float bodyWidth = 0.53; //meters

// Mixed mode channels on Kangaroo are, by default, 'D' and 'T'.
SoftwareSerial  SerialPort(rx_pin, tx_pin);
KangarooSerial  K(SerialPort);
KangarooChannel Drive(K, 'D');
KangarooChannel Turn(K, 'T');

float vDriveTicks; 
float vTurnTicks;

void messageCb(const geometry_msgs::Twist &tw_msg) {
  vDriveTicks = convertMToTDrive(tw_msg.linear.x);
  float linWVel =  tw_msg.angular.z * (wheelDiam/2 + bodyWidth/2);
  vTurnTicks = convertMToTTurn(linWVel);
  Drive.streaming(true);
  Turn.streaming(true);
  Drive.s(vDriveTicks); //may need .wait()
  Turn.s(vTurnTicks); //may need .wait()
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb);

std_msgs::Float32 fl_msg;
ros::Publisher chatter("chatter", &fl_msg);

void setup() {
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  Serial.begin(57600);
  //required Kangaroo setup commands:
  SerialPort.begin(9600);
  SerialPort.listen();
  Drive.start();
  Turn.start();
  Drive.si(0);
  Turn.si(0);
  //Serial.println("Done initializing Kangaroo.");
}

void loop() {
  fl_msg.data = vDriveTicks;
  chatter.publish(&fl_msg);
  nh.spinOnce();
  //delay(1);
}

float convertMToTDrive(float m) {
  long maxP = Drive.getMax().value();
  long minP = Drive.getMin().value();
  //Serial.print(maxP); Serial.print(", "); Serial.println(minP);
  //distance traveled after 360 deg rotation = wheel circumference (assuming no slip)
  float wheelCircumf = 2 * 3.14159 * (wheelDiam / 2); //meters
  long pRange = abs(maxP) + abs(minP); //ticks
  float ratioTPerM = pRange / wheelCircumf; //ticks per meter
  float t = m * ratioTPerM;
  return t;
}

float convertMToTTurn(float m) {
  long maxP = Turn.getMax().value();
  long minP = Turn.getMin().value();
  //Serial.print(maxP); Serial.print(", "); Serial.println(minP);
  //distance traveled after 360 deg rotation = wheel circumference (assuming no slip)
  float wheelCircumf = 2 * 3.14159 * (wheelDiam / 2); //meters
  long pRange = abs(maxP) + abs(minP); //ticks
  float ratioTPerM = pRange / wheelCircumf; //ticks per meter
  float t = m * ratioTPerM;
  return t;
}

