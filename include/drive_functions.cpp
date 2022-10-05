#include "include.cpp"

bool intakeToggle = false;

void intakeRollerControl(){
  if(!intakeToggle){
    float topTrackerAvg = (topTrackerLeft.get_value() + topTrackerRight.get_value())/2;
    if(master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_R1)){
      leftintake = 127;rightintake = 127;mainroller1.move_voltage(12000);mainroller2.move_voltage(12000);}
    else{
      if(indexTracker.get_value() > lineSensorSense || topTrackerAvg > lineSensorSense){
        if(master.get_digital(DIGITAL_R1)){leftintake = 127;rightintake = 127;mainroller1.move_voltage(4000);mainroller2.move_voltage(4000);}
        else if(master.get_digital(DIGITAL_R2)){leftintake = -127;rightintake = -127;mainroller1.move_voltage(-12000);mainroller2.move_voltage(-12000);}
        else if(master.get_digital(DIGITAL_L1)){mainroller1.move_voltage(12000);mainroller2.move_voltage(12000);leftintake = 0;rightintake = 0;}
        else if(master.get_digital(DIGITAL_L2)){mainroller1.move_voltage(-12000);mainroller2.move_voltage(-12000);leftintake = 0;rightintake = 0;}
        else{leftintake = 0;rightintake = 0;mainroller1 = 0;mainroller2 = 0;}
        }
      if(indexTracker.get_value() <= lineSensorSense || topTrackerAvg <= lineSensorSense){
        if(master.get_digital(DIGITAL_L1)){mainroller1.move_voltage(12000);mainroller2.move_voltage(12000);}
        else if(master.get_digital(DIGITAL_L2)){mainroller1.move_voltage(-12000);mainroller2.move_voltage(-12000);}
        else{mainroller1 = 0;mainroller2 = 0;}
        if(master.get_digital(DIGITAL_R1)){leftintake = 127;rightintake = 127;}
        else if(master.get_digital(DIGITAL_R2)){leftintake = -127;rightintake = -127;mainroller1.move_voltage(-12000);mainroller2.move_voltage(-12000);}
        else{leftintake = 0;rightintake = 0;}
      }
    }
  }
  else{
    if(master.get_digital(DIGITAL_L1)){mainroller1.move_voltage(12000);mainroller2.move_voltage(12000);}
    else if(master.get_digital(DIGITAL_L2)){mainroller1.move_voltage(-12000);mainroller2.move_voltage(-12000);}
    else{mainroller1 = 0;mainroller2 = 0;}

    if(master.get_digital(DIGITAL_R1)){leftintake = 127;rightintake = 127;}
    else if(master.get_digital(DIGITAL_R2)){leftintake = -127;rightintake = -127;}
    else{leftintake = 0;rightintake = 0;}

  }
}
