// Shana Shaw ENGR 1201 Fall 2016 class project based on Instructable project

#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h>

#define TRIG_PIN A0 // Pin A0 on the Motor Drive Shield soldered to the ultrasonic sensor
#define ECHO_PIN A1 // Pin A1 on the Motor Drive Shield soldered to the ultrasonic sensor
#define MAX_DISTANCE 300 // sets maximum useable sensor measuring distance to 300cm
#define MAX_SPEED 200 // sets speed of DC traction motors to 200/256 - to get power drain down.
#define MAX_SPEED_OFFSET 10 // this sets offset to allow for differences between the two DC traction motors
#define COLLISION_DISTANCE 20 // sets distance at which robot stops and reverses to 20cm
#define TURN_DISTANCE COLLISION_DISTANCE+20 // sets distance at which robot veers away from object (not reverse) to 40cm (20+20)

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); // sets up sensor library to use the correct pins to measure distance.

AF_DCMotor motor1(1, MOTOR12_1KHZ); // create motor #1 using M1 output on Motor Drive Shield, set to 1kHz PWM frequency
AF_DCMotor motor2(2, MOTOR12_1KHZ); // create motor #2, using M2 output, set to 1kHz PWM frequency
AF_DCMotor motor3(3, MOTOR12_1KHZ); // create motor #3, using M3 output, set to 1kHz PWM frequency
AF_DCMotor motor4(4, MOTOR12_1KHZ); // create motor #2, using M4 output, set to 1kHz PWM frequency

Servo myServo;  // create servo object to control a servo

int pos = 0; // this sets up variables for use in the sketch (code)
int maxDist = 0;
int maxAngle = 0;
int maxRight = 0;
int maxLeft = 0;
int maxFront = 0;
int course = 0;
int curDist = 0;
//String motorSet = "";
int speedSet = 0;

int readPing() { // read the ultrasonic sensor distance
  delay(70);
  //unsigned int uS = sonar.ping();
  unsigned int uS = sonar.ping_cm();
  //int cm = uS/US_ROUNDTRIP_CM;
  return uS;
}

void move(string toward){
  if(toward == "forward"){
    //motorSet = "FORWARD";
    motor1.run(FORWARD);      // turn it on going forward
    motor2.run(FORWARD);      // turn it on going forward
    motor3.run(FORWARD);      // turn it on going forward
    motor3.run(FORWARD);      // turn it on going forward
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
    {
      motor1.setSpeed(speedSet+MAX_SPEED_OFFSET);
      motor2.setSpeed(speedSet);
      motor3.setSpeed(speedSet);
      motor4.setSpeed(speedSet);
      delay(5);
    }
  } else if (toward == "backward") {
    //motorSet = "BACKWARD";
    motor1.run(BACKWARD);      // turn it on going forward
    motor2.run(BACKWARD);     // turn it on going forward
    motor3.run(BACKWARD);      // turn it on going forward
    motor4.run(BACKWARD);     // turn it on going forward
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
    {
      motor1.setSpeed(speedSet+MAX_SPEED_OFFSET);
      motor2.setSpeed(speedSet);
      motor3.setSpeed(speedSet);
      motor4.setSpeed(speedSet);
      delay(5);
    }
  } else if (toward == "right"){
      //motorSet = "RIGHT";
      motor1.run(FORWARD);      // turn motor 1 forward
      motor2.run(BACKWARD);     // turn motor 2 backward
      delay(400); // run motors this way for 400ms
      //motorSet = "FORWARD";
      motor1.run(FORWARD);      // set both motors back to forward
      motor2.run(FORWARD);
  } else if (toward == "left"){
      //motorSet = "LEFT";
      motor1.run(BACKWARD);     // turn motor 1 backward
      motor2.run(FORWARD);      // turn motor 2 forward
      delay(400); // run motors this way for 400ms
      //motorSet = "FORWARD";
      motor1.run(FORWARD);      // turn motor 1 on going forward
      motor2.run(FORWARD);      // turn motor 2 on going forward
  } else if (toward == "stop"){
    motor1.run(RELEASE); motor2.run(RELEASE); motor3.run(RELEASE); motor4.run(RELEASE);
  }
}

void checkPath() {
  int curLeft = 0;
  int curFront = 0;
  int curRight = 0;
  int curDist = 0;
  myServo.write(144); // set servo to face left 54-degrees from forward

  delay(120); // wait 120milliseconds for servo to reach position

  for(pos = 144; pos >= 36; pos-=18)     // loop to sweep the servo (& sensor) from 144-degrees left to 36-degrees right at 18-degree intervals.
  {
    myServo.write(pos);  // tell servo to go to position in variable 'pos'
    delay(90); // wait 90ms for servo to get to position

    //checkForward(); // check the robot is still moving forward
    //check("forward");
    move("forward");

    curDist = readPing(); // get the current distance to any object in front of sensor


    if (curDist < COLLISION_DISTANCE) { // if the current distance to object is less than the collision distance
      //checkCourse(); // run the checkCourse function
      move("backward");
      break; // jump out of this loop
    }
    if (curDist < TURN_DISTANCE) { // if current distance is less than the turn distance
      changePath(); // run the changePath function
    }

    if (curDist > curDist) {maxAngle = pos;}
    if (pos > 90 && curDist > curLeft) { curLeft = curDist;}
    if (pos == 90 && curDist > curFront) {curFront = curDist;}
    if (pos < 90 && curDist > curRight) {curRight = curDist;}
  }

  maxLeft = curLeft;
  maxRight = curRight;
  maxFront = curFront;
}

 void changePath() {
  //if (pos < 90) {veerLeft();} // if current pos of sensor is less than 90-degrees, it means the object is on the right hand side so veer left
  //if (pos > 90) {veerRight();} // if current pos of sensor is greater than 90-degrees, it means the object is on the left hand side so veer right
  if(pos < 90){
    move("left");
  } else if (pos > 90) {
    move("right");
  }
}

// In some cases, the Motor Drive Shield may just stop if the supply voltage is too low (due to using only four NiMH AA cells).
// The above functions simply remind the Shield that if it's supposed to go forward, then make sure it is going forward and vice versa.

//-------------------------------------------- SETUP LOOP ----------------------------------------------------------------------------
void setup() {
  myServo.attach(9);  // attaches the servo on pin 9 (SERVO_2 on the Motor Drive Shield to the servo object
  myServo.write(90); // tells the servo to position at 90-degrees ie. facing forward.
  delay(1000); // delay for one seconds
  checkPath(); // run the CheckPath routine to find the best path to begin travel
  //motorSet = "FORWARD"; // set the director indicator variable to FORWARD
  //myServo.write(90); // make sure servo is still facing forward
  //moveForward(); // run function to make robot move forward
  move("forward");
}

//---------------------------------------------MAIN LOOP ------------------------------------------------------------------------------
void loop() {
  //checkForward(); // check that if the robot is supposed to be moving forward, that the drive motors are set to move forward - this is needed to overcome some issues with only using 4 AA NiMH batteries
  //check("forward");
  checkPath(); // set ultrasonic sensor to scan for any possible obstacles
  //TODO: navigate();
}
