#include "robot-config.h"
using namespace vex;


void pre_auton(void) {
  userControlEnabled = false;
  task userControls(userControl);
  pickAuton();
  motorHold(true);
  calibrateInertial();
}

void autonomous(void) {
  calibrateInertial();
  userControlEnabled=false;
  flipOut();
  cubeRampVertical(false, 100);
  runAuton();
}

void usercontrol(void) {//User Control
  HUDenabled = true;
  userControlEnabled = true;
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) {
    wait(100, msec);
  }
}
