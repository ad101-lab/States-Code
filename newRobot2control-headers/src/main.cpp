#include "userControl.h"
using namespace vex;

void pre_auton(void) {
  userControlEnabled = false;
  HUDenabled = false;
  task userControls(userControl);
  motorHold(true);
  calibrateInertial();
  task pickAutonmous(pickAuton);
}

void autonomous(void) {
  calibrateInertial();
  task::sleep(500);
  userControlEnabled=false;
  redAutonBottom();
  //runAuton();
}

void usercontrol(void) {//User Control
  task showHUD(HUD);
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
