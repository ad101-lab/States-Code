#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "robot-config.h"
#include "v5.h"
#include "v5_vcs.h"

int userControl(){
  while (1){
    if (userControlEnabled) {
      rightFWD.spin(forward, (Controller1.Axis2.position()/ turnValue)/baseRPM , vex::velocityUnits::pct);//Tank Drive controls
      leftFWD.spin(forward, (Controller1.Axis3.position()/ turnValue)/baseRPM , vex::velocityUnits::pct);
      rightBack.spin(forward, (Controller1.Axis2.position()/ turnValue)/baseRPM , vex::velocityUnits::pct);
      leftBack.spin(forward, (Controller1.Axis3.position()/ turnValue)/baseRPM , vex::velocityUnits::pct);
      if (Controller2.ButtonL1.pressing() and !(cubeRamp.rotation(rev)>3.5)){//if button is pressing it will
        cubeRampValue = (-26*(cubeRamp.rotation(rev)))+100;//sets cube ramp to 85 RPM
     } else if (Controller2.ButtonL2.pressing() and !(cubeRamp.rotation(rev)<0)) {//if button is pressing it will
       cubeRampValue = -100;//sets cube ramp to -100 RPM
      } else {//if no others are true
        cubeRampValue = 0;//Stops cube ramp
      }
      cubeRamp.spin(forward, cubeRampValue , vex::velocityUnits::rpm);//applies the changes
      if (Controller1.ButtonR1.pressing()){//if button is pressing it will
        intakeValue = 100;//sets cube ramp to 100 RPM
      } else if (Controller1.ButtonR2.pressing()) {//if button is pressing it will
        intakeValue = -150;//sets cube ramp to -200 RPM
      } else if (Controller1.ButtonA.pressing()){//if button is pressing it will
        intakeValue = -50;//sets cube ramp to -50 RPM
      }else if(Controller1.ButtonB.pressing()){//if button is pressing it will
        intakeValue = 45;//sets cube ramp to 45 RPM
      } else if(Controller2.ButtonY.pressing()){//if button is pressing it will
        task stacking(stack);//Stacks
      } else {//If no other conditions are true
        intakeValue = 0;//sets cube ramp to -100 RPM
      }
      if(Controller2.ButtonUp.pressing() and !(oneBar.rotation(rev)>2.3)){
        oneBarValue = 100;
        cubeRampValue += 50;
     }else if (Controller2.ButtonDown.pressing()and !(oneBar.rotation(rev)<0)) {
        oneBarValue = -100;
      } else {
      oneBarValue = 0;
     }
     if (Controller2.ButtonR1.pressing()){
        baseRPM = 6;
      } else if (Controller2.ButtonR2.pressing()){
        baseRPM = 2;
      }else {
        baseRPM = 1;
     }
     if (((Controller1.Axis3.value() > 60) and (Controller1.Axis2.value() < -60)) or ((Controller1.Axis3.value() < -60) and (Controller1.Axis2.value() > 60))){
       turnValue = 3;
      } else{
      turnValue = 1;
      }
      /*if(Controller2.ButtonY.pressing()){
        resetEncoders();
      }*/
      if (Controller2.ButtonX.pressing()){
        oneBarTower("mid", true);
      } else if (Controller2.ButtonB.pressing()) {
        oneBarTower("low", true);
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