#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>

#include "v5.h"
#include "v5_vcs.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
using namespace vex;

extern brain Brain;
extern motor leftFWD;
extern motor leftBack;
extern motor rightFWD;
extern motor rightBack;
extern motor CubeRamp;
extern motor intakeRight;
extern motor intakeLeft;
extern inertial turnInertial;
extern controller Controller1;
extern controller Controller2;
using namespace vex;

competition Competition;

brain Brain;
bumper rampBumper        = bumper(Brain.ThreeWirePort.H);//Sets up the Globals of The limit bumpers
bumper rampBumperForward = bumper(Brain.ThreeWirePort.G);
motor rightFWD = motor(PORT12, ratio18_1, true);//Sets up the drivetrain motors(rightFWD)
motor leftFWD = motor(PORT13, ratio18_1, false);//Left FWD
motor rightBack = motor(PORT14, ratio18_1, true);//Right Back
motor leftBack = motor(PORT17, ratio18_1, false);//Letf Back
motor cubeRamp = motor(PORT6, ratio36_1, true);//Cube ramp motor global
motor intakeRight = motor(PORT15, ratio18_1, true);//Right intake global
motor intakeLeft = motor(PORT16, ratio18_1, false);//Left intake
motor oneBar = motor(PORT1, ratio36_1, false);
inertial turnInertial = inertial(PORT11);
controller Controller1        = controller(primary);//Sets up controllers
controller Controller2        = controller(partner);

int cubeRampValue;//Sets up integers to be used later
int intakeValue;
double cms;
double tright;
double tleft;
double oneBarValue;
double turnValue;
double baseRPM;
double degree;
double oneBarRotation;
bool autonSide = false;
bool autonColor;
bool userControlEnabled = false;
bool HUDenabled = false;
std::string tower;

void moveForward(double cm, double speed, bool stopping){
  leftFWD.spinFor(forward, cm, rev, speed, velocityUnits::pct, false);
  rightFWD.spinFor(forward, cm, rev, speed, velocityUnits::pct, false);
  leftBack.spinFor(forward, cm, rev, speed, velocityUnits::pct, false);
  rightBack.spinFor(forward, cm, rev, speed, velocityUnits::pct, stopping);
} 

void moveBackwards(double cm, double speed, bool stopping){
  leftFWD.spinFor(reverse, cm, rev, speed, velocityUnits::pct, false);
  rightFWD.spinFor(reverse, cm, rev, speed, velocityUnits::pct, false);
  leftBack.spinFor(reverse, cm, rev, speed, velocityUnits::pct, false);
  rightBack.spinFor(reverse, cm, rev, speed, velocityUnits::pct, stopping);
}

void moveForwardAccurate(double cm, double speed){
  leftFWD.spinFor(forward, cm-1, rev, speed, velocityUnits::pct, false);
  rightFWD.spinFor(forward, cm-1, rev, speed, velocityUnits::pct, false);
  leftBack.spinFor(forward, cm-1, rev, speed, velocityUnits::pct, false);
  rightBack.spinFor(forward, cm-1, rev, speed, velocityUnits::pct, true);
  leftFWD.spinFor(forward, cm-0.1, rev, speed*0.1, velocityUnits::pct, false);
  rightFWD.spinFor(forward, cm-0.1, rev, speed*0.1, velocityUnits::pct, false);
  leftBack.spinFor(forward, cm-0.1, rev, speed*0.1, velocityUnits::pct, false);
  rightBack.spinFor(forward, cm-0.1, rev, speed*0.1, velocityUnits::pct, true);
}

void moveBackwardsAccurate(double cm, double speed){
  leftFWD.spinFor(reverse, cm-1, rev, speed, velocityUnits::pct, false);
  rightFWD.spinFor(reverse, cm-1, rev, speed, velocityUnits::pct, false);
  leftBack.spinFor(reverse, cm-1, rev, speed, velocityUnits::pct, false);
  rightBack.spinFor(reverse, cm-1, rev, speed, velocityUnits::pct, true);
  leftFWD.spinFor(reverse, cm-0.1, rev, speed*0.1, velocityUnits::pct, false);
  rightFWD.spinFor(reverse, cm-0.1, rev, speed*0.1, velocityUnits::pct, false);
  leftBack.spinFor(reverse, cm-0.1, rev, speed*0.1, velocityUnits::pct, false);
  rightBack.spinFor(reverse, cm-0.1, rev, speed*0.1, velocityUnits::pct, true);
}

void turnRight(double degree, double speed){
  leftFWD.spin(forward, speed, pct);
  rightFWD.spin(reverse, speed, pct);
  leftBack.spin(forward, speed, pct);
  rightBack.spin(reverse, speed, pct);
  task::sleep(200);
  double difference=  degree - std::abs(turnInertial.rotation());
  while(difference>65){
    difference=  degree - turnInertial.rotation();
    task::sleep(20);
  }
  leftFWD.spin(forward, speed*0.1, pct);
  rightFWD.spin(reverse, speed*0.1, pct);
  leftBack.spin(forward, speed*0.1, pct);
  rightBack.spin(reverse, speed*0.1, pct);
  waitUntil(turnInertial.rotation() > (degree-2));
  leftFWD.stop();
  rightFWD.stop();
  leftBack.stop();
  rightBack.stop();
}

void turnLeft(double degree, double speed){
  leftFWD.spin(reverse, speed, pct);
  rightFWD.spin(forward, speed, pct);
  leftBack.spin(reverse, speed, pct);
  rightBack.spin(forward, speed, pct);
  task::sleep(200);
  double difference=  degree - std::abs(turnInertial.rotation());
  while(difference>65){
    difference=  degree - std::abs(turnInertial.rotation());
    task::sleep(20);
  }
  leftFWD.spin(reverse, speed*0.1, pct);
  rightFWD.spin(forward, speed*0.1, pct);
  leftBack.spin(reverse, speed*0.1, pct);
  rightBack.spin(forward, speed*0.1, pct);
  waitUntil(std::abs(turnInertial.rotation()) > (degree-2));
  leftFWD.stop();
  rightFWD.stop();
  leftBack.stop();
  rightBack.stop();
}

void oneBarChecker(){
  while(1){
    oneBarRotation = oneBar.rotation(rev);
    task::sleep(100);
  }
}

void cubeRampVertical (bool degree, double speed){
  cubeRamp.setVelocity(speed, rpm);//sets the velocity to the specified 
  if(degree == true){
    double speeds;
    while(cubeRamp.rotation(rev)<3.5){
      speeds = (cubeRamp.rotation(rev)*-26)+100;
      cubeRamp.spin(forward, speeds, pct);
    }
    cubeRamp.stop();//Stops the mmotor
  }else if (degree == false) {
    cubeRamp.spin(reverse);//moves the motor backwards
    waitUntil(cubeRamp.rotation(rev)< 0);//Waits until the bumper is pressed
    cubeRamp.stop();//Stops the mmotor
  }
  cubeRamp.setVelocity(100, percent);//Resets the velocity

}

void intake (double speed){
  intakeValue = speed*-1; //Conversion factor
  intakeLeft.spin(forward, intakeValue, rpm);//spins both intakes
  intakeRight.spin(forward, intakeValue, rpm);

}

int stack(){
  double speeds;
  while(cubeRamp.rotation(rev)<3.5){
    speeds = (cubeRamp.rotation(rev)*-26)+100;
    cubeRamp.spin(forward, speeds, pct);
    if(cubeRamp.rotation(rev)>2){
      intake(-100);
    }
  }
  moveForward(5, 50, true);
  intake(0);
  cubeRamp.spin(reverse);//Puts the cube ramp down
  waitUntil(cubeRamp.rotation(rev)<0);
  cubeRamp.stop();
  intake(-100);//Stops the intake
  moveBackwards(30, 100, true);
  intake(0);
  return 1;
}

void motorWait(){
  waitUntil(cubeRamp.isDone());//Waits Until the motors are done
  waitUntil(leftFWD.isDone());
  waitUntil(rightFWD.isDone());
  waitUntil(leftBack.isDone());
  waitUntil(rightBack.isDone());

}

void motorHold(bool holding){
  if(holding == true){
  intakeRight.setStopping(hold);//Holds if the code says to
  intakeLeft.setStopping(hold);
  cubeRamp.setStopping(hold);
  leftFWD.setStopping(hold);
  rightFWD.setStopping(hold);
  leftBack.setStopping(hold);
  rightBack.setStopping(hold);
  oneBar.setStopping(hold);
 }else{
  intakeRight.setStopping(coast);//Lets the motors coast if not
  intakeLeft.setStopping(coast);
  cubeRamp.setStopping(coast);
  leftFWD.setStopping(coast);
  rightFWD.setStopping(coast);
  leftBack.setStopping(coast);
  rightBack.setStopping(coast);
  oneBar.setStopping(coast);
 }
}

int oneBarUp(int distance, int speeds, bool stopping){
  if(oneBar.rotation(rev) < 5 and distance > 0){
    oneBar.spinFor(forward, distance/15, degrees, speeds, velocityUnits::rpm, stopping);//1:15 gear ratio
  }else if (oneBar.rotation(rev) > 0 and distance < 0) {
    oneBar.spinFor(forward, distance/15, degrees, speeds, velocityUnits::rpm, stopping);//1:15 gear ratio
  }else {
    oneBar.stop();
    return false;
  }
  return true;
}

void oneBarTower(std::string tower, bool waiting){
  double goal;
  if(tower == "High" or tower =="high"){
    goal = 2.3;
  } else if (tower == "Mid" or tower == "Middle" or tower == "mid" or tower == "middle") {
    goal = 2.2;
  }else if (tower == "Low" or tower == "low" or tower == "alliance" or tower == "Alliance") {
    goal = 0.7;
  } else {};
  oneBar.spinTo(goal, rev, waiting);
}

void flipOut(){
  
}

int redAutonBottom(){
  intake(170);
  moveForward(4.2, 30, true);
  intake(-50);
  wait(0.5, seconds);
  intake(0);
  moveBackwards(1.6, 60, true);
  turnRight(135, 60);
  moveForward(2.2, 60, true);
  stack();
  return 1;
}

int blueAutonBottom(){
  intake(170);
  moveForward(4.2, 30, true);
  intake(-50);
  wait(0.5, seconds);
  intake(0);
  moveBackwards(1.6, 60, true);
  turnRight(135, 60);
  moveForward(2.2, 60, true);
  stack();
  return 1;
}

int redAutonTop(){
  return 1;
}

int blueAutonTop(){
  moveForwardAccurate(1.9, 50);
  turnRight(90, 60);
  moveForwardAccurate(2.2, 60);
  turnLeft(45, 60);
  moveForwardAccurate(2, 60);
  stack();
  return 1;
}

int pickAuton (){
  task::sleep(100);
  userControlEnabled = false;
  while(!Controller1.ButtonA.pressing()){
    if(Controller1.ButtonRight.pressing() or Controller1.ButtonLeft.pressing()){
      autonColor = !autonColor;     
      task::sleep(200);
    }else if(Controller1.ButtonUp.pressing() or Controller1.ButtonDown.pressing()){
      autonSide = !autonSide;     
      task::sleep(200);
    }else{}
    if(autonColor and autonSide){
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.clearScreen();
      Controller1.Screen.print("RED, TOP");
    } else if(autonColor and !autonSide){
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.clearScreen();
      Controller1.Screen.print("RED, BOTTOM");
    } else if(!autonColor and autonSide){
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.clearScreen();
      Controller1.Screen.print("BLUE, TOP");
    } else if(!autonColor and !autonSide){
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.clearScreen();
      Controller1.Screen.print("BLUE, BOTTOM");
    }
    task::sleep(20);
  }
  userControlEnabled = true;
  return 1;
}

void runAuton(){
  if(autonColor and autonSide){
    task redAutonomousTop(redAutonTop);
  } else if(autonColor and !autonSide){
    task redAutonomousBottom(redAutonBottom);
  } else if(!autonColor and autonSide){
    task blueAutonomousTop(blueAutonTop);
  } else if(!autonColor and !autonSide){
    task blueAutonomousBottom(blueAutonBottom);
  }
}

void calibrateInertial(){
  turnInertial.calibrate();
  // waits for Inertial Sensor to calibrate 
  while (turnInertial.isCalibrating()) {
    wait(100, msec);
  }
}

void resetEncoders(){
  rightBack.resetPosition();
  leftBack.resetPosition();
  rightFWD.resetPosition();
  leftFWD.resetPosition();
  oneBar.resetPosition();
  cubeRamp.resetPosition();
  intakeLeft.resetPosition();
  intakeRight.resetPosition();
}


int HUD(){
  while(HUDenabled){
    int cap = Brain.Battery.capacity();
    int timeInMatch = 105;
    int intakeTemp;
    while(1){
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Time: ");
      Controller1.Screen.print(timeInMatch);
      Controller1.Screen.setCursor(2, 1);
      Controller1.Screen.print("Battery: ");
      Controller1.Screen.print(cap);
      Controller1.Screen.setCursor(3, 1);
      Controller1.Screen.print("Intake Temp: ");
      Controller1.Screen.print(intakeTemp);
      task::sleep(1000);
      cap = Brain.Battery.capacity();
      timeInMatch = timeInMatch - 1;
      intakeTemp = (intakeLeft.temperature()+ intakeRight.temperature())/2;
    }
  }
  return 1;
}