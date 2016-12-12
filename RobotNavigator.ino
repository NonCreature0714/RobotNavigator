// Shana Shaw ENGR 1201 Fall 2016 class project based on Instructable project

//Libraries. Need to install these if they aren't already.
#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h>

//Define the pins on the Arduino board, which are physically labeled on the board.
#define TRIG_PIN A0 // Pin A0 on the Motor Drive Shield soldered to the ultrasonic sensor
#define ECHO_PIN A1 // Pin A1 on the Motor Drive Shield soldered to the ultrasonic sensor

#define MAX_SPEED 200 // sets speed of DC traction motors to 200/256 - to get power drain down.
#define MAX_SPEED_OFFSET 10 // this sets offset to allow for differences between the two DC traction motors

//Define limits and steps for servo
#define STEP_LEFT 45
#define STEP_RIGHT -45
#define FRONT 90
#define FRONT_RIGHT FRONT+STEP_RIGHT
#define FRONT_LEFT FRONT+STEP_LEFT
#define LEFT_LIMIT 180
#define RIGHT_LIMIT 0


//Define limits on sensor.
#define NUM_READINGS 5
#define MAX_DISTANCE 300 // sets maximum useable sensor measuring distance to 300cm
#define MAX_READ_ATTEMPTS 5
#define SAMPLE_SIZE 10
#define HISTORY_SIZE 20
#define EMERGENCY_STOP 3

//Define navigation limits.
#define COLLISION_DISTANCE 20 // sets distance at which robot stops and reverses to 20cm
#define TURN_DISTANCE COLLISION_DISTANCE+20 // sets distance at which robot veers away from object (not reverse) to 40cm (20+20)

//Instantiate the sonar sensor.
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); // sets up sensor library to use the correct pins to measure distance.

//Instantiate the motors.
AF_DCMotor motor1(1, MOTOR12_1KHZ); // create motor #1 using M1 output on Motor Drive Shield, set to 1kHz PWM frequency
AF_DCMotor motor2(2, MOTOR12_1KHZ); // create motor #2, using M2 output, set to 1kHz PWM frequency
AF_DCMotor motor3(3, MOTOR12_1KHZ); // create motor #3, using M3 output, set to 1kHz PWM frequency
AF_DCMotor motor4(4, MOTOR12_1KHZ); // create motor #2, using M4 output, set to 1kHz PWM frequency

//Instantiate the servo.
Servo myServo;

/*
 * This Boolean value controls which protothread is active.
 * Set it to 'true' so the robot begins scanning.
 */
bool isSensorReadCycle = true;
bool scanComplete = false;

//Define navigation commands.
//Note how the values of the navigation commands are the same as sensorReadingAt array. This is not a mistake.
const uint8_t   fullRight = 4,
                halfRight = 3,
                front = 2,
                halfLeft = 1,
                fullLeft = 0;

struct LinearPath{
    int distance;
    int lineOfBearing;
    uint8_t position;
    bool isSmoothingSensorRead;
    bool isGoodPath;
    LinearPath * nextPath;
    LinearPath * getNextPath(){
        return nextPath;
    }
    void check(){
        if(distance == 0){
            isGoodPath = false;
            return;
        }
        if (distance > 0 && distance < MAX_DISTANCE){
            isGoodPath = true;
            isSmoothingSensorRead = false;
            return;
        }
        if(distance == 255 && isSmoothingSensorRead){
            isGoodPath = false;
        }
    }
    LinearPath(){
        distance = 0;
        lineOfBearing = 0;
        isSmoothingSensorRead = false;
        isGoodPath = false;
        nextPath = 0;
    }
    LinearPath(int d, int l, uint8_t p, bool iS, bool iG, LinearPath * np):
        distance(d), lineOfBearing(l), position(p), isSmoothingSensorRead(iS), isGoodPath(iG), nextPath(np)
        {}
};



//Define movement states.
const uint8_t rollForward = 0,
              rollBackward = 1,
              turnLeft = 2,
              turnRight = 3,
              haltWheels = 4;

//Declare a movement state.
uint8_t movementState = haltWheels; //Start with a 'haltWheels' movement state.

const LinearPath * frontPtr = sensorReadingAt[2];
const LinearPath * frontLeftPtr = sensorReadingAt[1];
const LinearPath * leftPtr = sensorReadingAt[0];
const LinearPath * frontRightPtr = sensorReadingAt[3];
const LinearPath * rightPtr = sensorReadingAt[4];

int sensorLineOfBearing = RIGHT_LIMIT; // this sets up variables for use in the sketch (code)
LinearPath sensorReadingAt[NUM_READINGS] = {LinearPath(0,LEFT_LIMIT,fullLeft,false,false,frontLeftPtr),
                                            LinearPath(0,FRONT_LEFT,halfLeft,false,false,frontPtr),
                                            LinearPath(0,FRONT,front,false,false,frontLeftPtr),
                                            LinearPath(0,FRONT_RIGHT,halfRight,false,false,rightPtr),
                                            LinearPath(0,RIGHT_LIMIT,fullRight,false,false,frontRightPtr)};

bool sensorReadingComplete[NUM_READINGS] = {false, false, false, false, false};//Robot will only move if all sensor readings are complete.

int averages[NUM_READINGS] = {0};

const int LEFT_STARTING_READ_INDEX = 4;
const int RIGHT_STARTING_READ_INDEX = 0;

int speedSetting = 0;

const int SWEEP_READING_LEFT = -1; //True because starting sweep from right.
const int SWEEP_READING_RIGHT = 1;


/********************************************
 *                                          *
 *          FUNCTION DECLARATIONS           *
 *                                          *
 ********************************************/

/*
 *  sensorReadAlong(path) function.
 *
 *  Takes a Linear path as an argument,
 *  reads 
 */
void sensorReadAlong(LinearPath * path){
  int numTries = 0;
  do {
    path->isSmoothingSensorRead = false;
    ++numTries;
    path->distance = sonar.ping_cm();
    if(numTries > MAX_READ_ATTEMPTS){
        path->isSmoothingSensorRead = true;
        return MAX_DISTANCE; //If more than 5 tries, return max range.
    }
  } while (path->distance == 0);
}//End of sensorReadAlong().

int rollingAverage(int average, const int newSample){
  average -= average/SAMPLE_SIZE;
  if(average < 0)
    return 0;
  average += newSample/SAMPLE_SIZE;
  return average;
}//End of rollingAverage.

int interpretDataAt(LinearPath * path, int sensorReading){
    int avg = rollingAverage(sensorIndex, sensorReading);
    if (avg <= 0){
        return sensorReading = 0;
    }
    if (avg >= 255){
        return sensorReading = 255;
    }
    return sensorReading;
}

LinearPath * currentLinearPath; //Cautious use of global pointer variable, only for use in function below.
int readAndStep(){
    //TODO: read along LinearPath.
    //TODO: interpret that data.
    //TODO: step the servo to next LinearPath.
    //TODO: move the currentLinearPath to that sensorPath.
}

void scan(LinearPath * path){
    if(lineOfBearing >= RIGHT_LIMIT && lineOfBearing < LEFT_LIMIT){
        readAndStep(STEP_LEFT, RIGHT_STARTING_READ_INDEX, SWEEP_READING_LEFT, lineOfBearing);
    } else if(lineOfBearing <= LEFT_LIMIT && lineOfBearing > RIGHT_LIMIT){
        readAndStep(STEP_RIGHT, LEFT_STARTING_READ_INDEX, SWEEP_READING_RIGHT, lineOfBearing);
    }
}


/********************************************************
 *                                                      *
 *      NAVIGATION & MOVEMENT FUNCTIONS BELOW           *
 *                                                      *
 ********************************************************/

int pickBestPath(){
    //TODO: add pathfinding algorithm.
    int bestPath = 0;
    for(int i = 0; i<NUM_READINGS; ++i){
        if(sensorReadingAt[i] > bestPath){
            bestPath = i;
        }
    }
    return bestPath;
}

void move(uint8_t toward){
  switch(toward){
    case rollForward:
      //TODO: if movementState is already forward, continue, else speed up slowly.
      motor1.run(FORWARD);      // turn it on going forward
      motor2.run(FORWARD);      // turn it on going forward
      motor3.run(FORWARD);      // turn it on going forward
      motor3.run(FORWARD);      // turn it on going forward
      for (speedSetting = 0; speedSetting < MAX_SPEED; speedSetting +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
      {
        motor1.setSpeed(speedSetting+MAX_SPEED_OFFSET);
        motor2.setSpeed(speedSetting);
        motor3.setSpeed(speedSetting);
        motor4.setSpeed(speedSetting);
        delay(5);
      }
      break;

    case rollBackward:
      //TODO: if movementState is alread backward, continue, else speed up slowly.
      motor1.run(BACKWARD);      // turn it on going forward
      motor2.run(BACKWARD);     // turn it on going forward
      motor3.run(BACKWARD);      // turn it on going forward
      motor4.run(BACKWARD);     // turn it on going forward
      for (speedSetting = 0; speedSetting < MAX_SPEED; speedSetting +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
      {
        motor1.setSpeed(speedSetting+MAX_SPEED_OFFSET);
        motor2.setSpeed(speedSetting);
        motor3.setSpeed(speedSetting);
        motor4.setSpeed(speedSetting);
        delay(5);
      }
      break;

    case turnLeft:
      motor1.run(BACKWARD);     // turn motor 1 backward
      motor2.run(FORWARD);      // turn motor 2 forward
      delay(400); // run motors this way for 400ms
      motor1.run(FORWARD);      // turn motor 1 on going forward
      motor2.run(FORWARD);      // turn motor 2 on going forward
      break;

    case turnRight:
      motor1.run(FORWARD);      // turn motor 1 forward
      motor2.run(BACKWARD);     // turn motor 2 backward
      delay(400); // run motors this way for 400ms
      motor1.run(FORWARD);      // set both motors back to forward
      motor2.run(FORWARD);
      break;

    case haltWheels:
      motor1.run(RELEASE); motor2.run(RELEASE); motor3.run(RELEASE); motor4.run(RELEASE);
      break;
  }
}


uint8_t chooseMovement(){
    int path = pickBestPath();
    switch(path){
        case fullRight:
            return turnRight;
            break;

        case halfRight:
            return turnRight;
            break;

        case front:
            return rollForward;
            break;

        case halfLeft:
            return turnLeft;
            break;

        case fullLeft:
            return turnLeft;
            break;

        default:
            return haltWheels;
            break;
    }
}

void navigate(){
    move(chooseMovement());
}

/************************************************************
 *                                                          *
 *              SWEEP AND NAVIGATE FUNCTION                 *
 *                                                          *
 *  This function is what allows the Arduino to protothread,*
 *  meaning it can handle multiple tasks seemingly at once. *
 *                                                          *
 ************************************************************/


void sweepAndNavigate(){
    if(isSensorReadCycle){
        //TODO: Scan and turn servo.
        if(!scanComplete){
            isSensorReadCycle = true;
        } else {
            isSensorReadCycle = false;
        }

    } else {
        //TODO: Analyse and move.
        isSensorReadCycle = true;
    }
}

// In some cases, the Motor Drive Shield may just stop if the supply voltage is too low (due to using only four NiMH AA cells).
// The above functions simply remind the Shield that if it's supposed to go forward, then make sure it is going forward and vice versa.

//-------------------------------------------- SETUP LOOP ----------------------------------------------------------------------------
void setup() {
  myServo.attach(9);  // attaches the servo on pin 9 (SERVO_2 on the Motor Drive Shield to the servo object
  myServo.write(RIGHT_LIMIT); // tells the servo to position at 90-degrees ie. facing forward.
  delay(1000); // delay for one seconds
}

//---------------------------------------------MAIN LOOP ------------------------------------------------------------------------------
void loop() {
  sweepAndNavigate();
}
