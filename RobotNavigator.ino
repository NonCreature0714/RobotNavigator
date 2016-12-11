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
#define FRONT 90
#define LEFT_LIMIT 180
#define RIGHT_LIMIT 0
#define STEP_LEFT 45
#define STEP_RIGHT -45

//Define limits on sensor.
#define NUM_READINGS 5
#define MAX_DISTANCE 300 // sets maximum useable sensor measuring distance to 300cm
#define MAX_READ_ATTEMPTS 5
#define SAMPLE_SIZE 10
#define HISTORY_SIZE 20

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

//Define movement states.
const uint8_t rollForward = 0,
              rollBackward = 1,
              turnLeft = 2,
              turnRight = 3,
              haltWheels = 4;

//Define navigation commands.
const uint8_t   fullRight = 4,
                halfRight = 3,
                front = 2,
                halfLeft = 1,
                fullLeft = 0;


//Declare a movement state.
uint8_t movementState = haltWheels; //Start with a 'haltWheels' movement state.
bool moving = false; //Detects if it's already moving.
bool isSmoothing = false;

int sensorLineOfBearing = RIGHT_LIMIT; // this sets up variables for use in the sketch (code)
int sensorReadingAt[NUM_READINGS] = {0};

int * frontPtr = sensorReadingAt[2];
int * frontLeftPtr = sensorReadingAt[1];
int * leftPtr = sensorReadingAt[0];
int * frontRightPtr = sensorReadingAt[3];
int * rightPtr = sensorReadingAt[4];

bool goodPathAt[NUM_READINGS] = {false, false, false, false, false};

const int LEFT_STARTING_READ_INDEX = 4;
const int RIGHT_STARTING_READ_INDEX = 0;

int speedSetting = 0;

const int SWEEP_READING_LEFT = -1; //True because starting sweep from right.
const int SWEEP_READING_RIGHT = 1;

int sensorRead(){
  int numTries = 0;
  int uSPing = 0;
  do {
    isSmoothing = false;
    ++numTries;
    uSPing = sonar.ping_cm();
    if(numTries > MAX_READ_ATTEMPTS){
        isSmoothing = true;
        return MAX_DISTANCE; //If more than 5 tries, return max range.
    }
  } while (uSPing == 0);
  return uSPing;
}//End of sensorRead().

int rollingAverage(int average, const int newSample){
  average -= average/SAMPLE_SIZE;
  if(average < 0)
    return 0;
  average += newSample/SAMPLE_SIZE;
  return average;
}//End of rollingAverage.

int interpretDataFrom(int oldSensor, int sensor){
    int avg = rollingAverage(oldSensor, sensor);
    if (avg <= 0){
        return sensor = 0;
    } else if (avg >= 255){
        return sensor = 255;;
    }
    return sensor;
}

int readAndSweep(int nextStep, int startingReadIndex, int arrayStep, int lob){
    for(int i = startingReadIndex; i<NUM_READINGS; i += arrayStep, lob += nextStep){
        sensorReadingAt[i] = interpretDataFrom(sensorReadingAt[i], sensorRead());
        myServo.write(lob);
        sensorLineOfBearing = lob; //Update the global variable with the current value.
    }
}

void sweepFrom(int lineOfBearing){
    if(lineOfBearing >= RIGHT_LIMIT && lineOfBearing < LEFT_LIMIT){
        readAndSweep(STEP_LEFT, RIGHT_STARTING_READ_INDEX, SWEEP_READING_LEFT, lineOfBearing);
    } else if(lineOfBearing <= LEFT_LIMIT && lineOfBearing > RIGHT_LIMIT){
        readAndSweep(STEP_RIGHT, LEFT_STARTING_READ_INDEX, SWEEP_READING_RIGHT, lineOfBearing);
    }
}

int pickBestPath(){
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
  sweepFrom(sensorLineOfBearing);
  navigate();
}
