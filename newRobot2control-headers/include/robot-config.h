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

extern brain Brain;  // sets up all of the external objects
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

int cubeRampValue;//Sets up varables to be used later
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
  leftFWD.spinFor(forward, cm, rev, speed, velocityUnits::pct, false);//starts the motors
  rightFWD.spinFor(forward, cm, rev, speed, velocityUnits::pct, false);
  leftBack.spinFor(forward, cm, rev, speed, velocityUnits::pct, false);
  rightBack.spinFor(forward, cm, rev, speed, velocityUnits::pct, stopping);
} 

void moveBackwards(double cm, double speed, bool stopping){
  leftFWD.spinFor(reverse, cm, rev, speed, velocityUnits::pct, false);//starts the motors
  rightFWD.spinFor(reverse, cm, rev, speed, velocityUnits::pct, false);
  leftBack.spinFor(reverse, cm, rev, speed, velocityUnits::pct, false);
  rightBack.spinFor(reverse, cm, rev, speed, velocityUnits::pct, stopping);
}

void moveForwardAccurate(double cm, double speed){
  leftFWD.spinFor(forward, cm-0.3, rev, speed, velocityUnits::pct, false);//moves close
  rightFWD.spinFor(forward, cm-0.3, rev, speed, velocityUnits::pct, false);
  leftBack.spinFor(forward, cm-0.3, rev, speed, velocityUnits::pct, false);
  rightBack.spinFor(forward, cm-0.3, rev, speed, velocityUnits::pct, true);
  leftFWD.spinFor(forward, 0.1, rev, speed*0.1, velocityUnits::pct, false);//moves there and stops
  rightFWD.spinFor(forward, 0.1, rev, speed*0.1, velocityUnits::pct, false);
  leftBack.spinFor(forward, 0.1, rev, speed*0.1, velocityUnits::pct, false);
  rightBack.spinFor(forward, 0.1, rev, speed*0.1, velocityUnits::pct, true);
}

void moveBackwardsAccurate(double cm, double speed){
  leftFWD.spinFor(reverse, cm-0.3, rev, speed, velocityUnits::pct, false);// moves close
  rightFWD.spinFor(reverse, cm-0.3, rev, speed, velocityUnits::pct, false);
  leftBack.spinFor(reverse, cm-0.3, rev, speed, velocityUnits::pct, false);
  rightBack.spinFor(reverse, cm-0.3, rev, speed, velocityUnits::pct, true);
  leftFWD.spinFor(reverse, 0.1, rev, speed*0.1, velocityUnits::pct, false);//move there and stops
  rightFWD.spinFor(reverse, 0.1, rev, speed*0.1, velocityUnits::pct, false);
  leftBack.spinFor(reverse, 0.1, rev, speed*0.1, velocityUnits::pct, false);
  rightBack.spinFor(reverse, 0.1, rev, speed*0.1, velocityUnits::pct, true);
}

void turnRight(double degree, double speed){
  turnInertial.setRotation(0, degrees);
  leftFWD.spin(forward, speed, pct);// moves close at full speed
  rightFWD.spin(reverse, speed, pct);
  leftBack.spin(forward, speed, pct);
  rightBack.spin(reverse, speed, pct);
  task::sleep(200);
  double difference=  degree - std::abs(turnInertial.rotation());
  while(difference>65){
    difference=  degree - turnInertial.rotation();
    task::sleep(20);
  }
  leftFWD.spin(forward, speed*0.1, pct);//slows down
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
  turnInertial.setRotation(0, degrees);
  leftFWD.spin(reverse, speed, pct);//moves close at full speed
  rightFWD.spin(forward, speed, pct);
  leftBack.spin(reverse, speed, pct);
  rightBack.spin(forward, speed, pct);
  task::sleep(200);
  double difference=  degree - std::abs(turnInertial.rotation());
  while(difference>65){
    difference=  degree - std::abs(turnInertial.rotation());
    task::sleep(20);
  }
  leftFWD.spin(reverse, speed*0.1, pct);//slows down
  rightFWD.spin(forward, speed*0.1, pct);
  leftBack.spin(reverse, speed*0.1, pct);
  rightBack.spin(forward, speed*0.1, pct);
  waitUntil(std::abs(turnInertial.rotation()) > (degree-2));
  leftFWD.stop();
  rightFWD.stop();//stops
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
    cubeRamp.spinTo(3.7, rev, false);//Stops the mmotor
  }else if (degree == false) {
    cubeRamp.spinTo(0, rev, false);
  }
  cubeRamp.setVelocity(100, percent);//Resets the velocity

}

void intake (double speed){
  intakeValue = speed*-1; //Conversion factor
  if(speed == 0){
    intakeLeft.stop();
    intakeRight.stop();
  }
  intakeLeft.spin(forward, intakeValue, rpm);//spins both intakes
  intakeRight.spin(forward, intakeValue, rpm);

}

int stack(){
  double speeds;
  while(cubeRamp.rotation(rev)<3){
    speeds = (cubeRamp.rotation(rev)*-22)+100;
    cubeRamp.spin(forward, speeds, pct);//puts ramp up in a linear function
    if(cubeRamp.rotation(rev)>1.5){
      intake(-70);
    }
  }
  intake(0);
  cubeRamp.spin(reverse);//Puts the cube ramp down
  moveBackwards(1, 100, true);
  cubeRamp.stop();
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
  if (tower == "Mid" or tower == "Middle" or tower == "mid" or tower == "middle") {//moves to mid tower
    goal = 2;
  }else if (tower == "Low" or tower == "low" or tower == "alliance" or tower == "Alliance") {//moves to low tower
    goal = 1.5;
  } else {
    goal = 0;
  };
  cubeRamp.spinTo(1.5, rev, false);
  wait(0.25, seconds);
  oneBar.spinTo(goal, rev, waiting);// moves to goal
}

int oneBarTowerLow(){
  oneBarTower("low",true);
  return 1;
}

int oneBarTowerMid(){
  oneBarTower("mid",true);
  return 1;
}

void flipOut(){
  oneBarTowerLow();
  oneBarTower("none",true);
}

int redAutonBottom(){
  intake(170);
  moveForward(4.2, 30, true);
  intake(-50);
  wait(0.5, seconds);
  intake(0);
  moveBackwards(1.8, 60, true);
  turnRight(135, 60);
  moveForward(2.1, 60, true);
  stack();
  return 1;
}

int blueAutonBottom(){
  intake(170);
  moveForward(4.2, 30, true);
  intake(-50);
  wait(0.5, seconds);
  intake(0);
  moveBackwards(1.8, 60, true);
  turnLeft(135, 60);
  moveForward(2.2, 60, true);
  stack();
  return 1;
}

int redAutonTop(){
  intake(150);
  moveForward(2.2, 50, true);
  intake(0);
  turnRight(90, 60);
  intake(150);
  moveForward(2.4, 60, true);
  intake(-50);
  task::sleep(500);
  intake(0);
  task::sleep(20);
  turnLeft(35, 60);
  moveForward(1.8, 60, true);
  stack();
  return 1;
}

int blueAutonTop(){
  intake(150);
  moveForward(2.2, 50, true);
  intake(0);
  turnRight(90, 60);
  intake(150);
  moveForward(2.4, 60, true);
  intake(-50);
  task::sleep(500);
  intake(0);
  task::sleep(20);
  turnRight(35, 60);
  moveForward(1.8, 60, true);
  stack();
  return 1;
}

void skills(){
  moveForward(10.5, 50, true);
  turnRight(45, 60);
  moveForward(1.8, 60, true);
  stack();
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
      Controller2.Screen.clearScreen();
      Controller2.Screen.setCursor(1, 1);
      Controller2.Screen.print("Time: ");
      Controller2.Screen.print(timeInMatch);
      Controller2.Screen.setCursor(2, 1);
      Controller2.Screen.print("Battery: ");
      Controller2.Screen.print(cap);
      Controller2.Screen.setCursor(3, 1);
      Controller2.Screen.print("Intake Temp: ");
      Controller2.Screen.print(intakeTemp);
      task::sleep(1000);
      cap = Brain.Battery.capacity();
      timeInMatch = timeInMatch - 1;
      intakeTemp = (intakeLeft.temperature()+ intakeRight.temperature())/2;
    }
  }
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
  HUD();
  userControlEnabled = true;
  return 1;
}

void runAuton(){
  if(autonColor and autonSide){
    redAutonTop();
  } else if(autonColor and !autonSide){
    redAutonBottom();
  } else if(!autonColor and autonSide){
    blueAutonTop();
  } else if(!autonColor and !autonSide){
    blueAutonBottom();
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

int baseStop(){
  rightFWD.stop();
  rightBack.stop();//Stops the base motors
  leftFWD.stop();
  leftBack.stop();
  return 1;
}

int userControl(){
  while (1){
    if (userControlEnabled) {
      rightFWD.spin(forward, (Controller1.Axis2.position()/ turnValue)/baseRPM , vex::velocityUnits::pct);//Tank Drive controls
      leftFWD.spin(forward, (Controller1.Axis3.position()/ turnValue)/baseRPM , vex::velocityUnits::pct);
      rightBack.spin(forward, (Controller1.Axis2.position()/ turnValue)/baseRPM , vex::velocityUnits::pct);
      leftBack.spin(forward, (Controller1.Axis3.position()/ turnValue)/baseRPM , vex::velocityUnits::pct);
      if (Controller2.ButtonL1.pressing() and !(cubeRamp.rotation(rev)>3)){//if button is pressing it will
        cubeRampValue = (-23*(cubeRamp.rotation(rev)))+100;//sets cube ramp to 85 RPM
     } else if (Controller2.ButtonL2.pressing() and (!(cubeRamp.rotation(rev)<0) or Controller2.ButtonY.pressing())) {//if button is pressing it will
       cubeRampValue = -100;//sets cube ramp to -100 RPM
      } else {//if no others are true
        cubeRampValue = 0;//Stops cube ramp
      }
      cubeRamp.spin(forward, cubeRampValue , vex::velocityUnits::rpm);//applies the changes
      if (Controller1.ButtonR1.pressing() or Controller2.ButtonRight.pressing()){//if button is pressing it will
        intakeValue = 100;//sets cube ramp to 100 RPM
      } else if (Controller1.ButtonR2.pressing() or Controller2.ButtonLeft.pressing()) {//if button is pressing it will
        intakeValue = -150;//sets cube ramp to -200 RPM
      } else if (Controller1.ButtonA.pressing()){//if button is pressing it will
        intakeValue = -50;//sets cube ramp to -50 RPM
      }else if(Controller1.ButtonB.pressing()){//if button is pressing it will
        intakeValue = 45;//sets cube ramp to 45 RPM
      } /*else if(Controller2.ButtonY.pressing()){//if button is pressing it will
        task stacking(stack);//Stacks
      }*/ else {//If no other conditions are true
        intakeValue = 0;//sets cube ramp to -100 RPM
      }
      if(Controller2.ButtonUp.pressing() and (!(oneBar.rotation(rev)>2.3) or Controller2.ButtonY.pressing())){
        oneBarValue = 100;
        cubeRampValue += 50;
     }else if (Controller2.ButtonDown.pressing()and !(oneBar.rotation(rev)<0)) {
        oneBarValue = -100;
      } else {
      oneBarValue = 0;
     }
     if (Controller2.ButtonR1.pressing() or Controller1.ButtonL1.pressing()){
        baseRPM = 6;
      } else if (Controller2.ButtonR2.pressing() or Controller1.ButtonL2.pressing()){
        baseRPM = 2;
      }else {
        baseRPM = 1;
     }
     if (((Controller1.Axis3.value() > 60) and (Controller1.Axis2.value() < -60)) or ((Controller1.Axis3.value() < -60) and (Controller1.Axis2.value() > 60))){
       turnValue = 3;
      } else{
      turnValue = 1;
      }
      if(Controller2.ButtonY.pressing()){
        resetEncoders();
      }
      if (Controller2.ButtonX.pressing()){
        task getOneBarTower(oneBarTowerMid);
      } else if (Controller2.ButtonB.pressing()) {
        task getOneBarTower(oneBarTowerLow);
      }
      oneBar.spin(forward, oneBarValue, pct);
      intakeLeft.spin(forward, intakeValue , vex::velocityUnits::rpm);//applies the changes
      intakeRight.spin(forward, intakeValue , vex::velocityUnits::rpm);
      wait(20, msec); // Sleep the task for a short amount of time to
      Brain.Screen.print(oneBar.rotation(rev));
    }
    task::sleep(50);
  }
  return 1;
}