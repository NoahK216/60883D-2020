#pragma once
#include "include.cpp"

int ballCount = 0;
int topBallCount = 0;
bool inBottom = false;
bool inIndex = false;
bool inTop = false;

unsigned int taskNum = 0;
float taskTimer;
bool taskNotification = false;

void DriveTrainBrake(){
  frontright.set_brake_mode(MOTOR_BRAKE_BRAKE);
  backright.set_brake_mode(MOTOR_BRAKE_BRAKE);
  frontleft.set_brake_mode(MOTOR_BRAKE_BRAKE);
  backleft.set_brake_mode(MOTOR_BRAKE_BRAKE);
}

void DriveTrainCoast(){
  frontright.set_brake_mode(MOTOR_BRAKE_COAST);
  backright.set_brake_mode(MOTOR_BRAKE_COAST);
  frontleft.set_brake_mode(MOTOR_BRAKE_COAST);
  backleft.set_brake_mode(MOTOR_BRAKE_COAST);
}

void DriveTrainHold(){
  frontright.set_brake_mode(MOTOR_BRAKE_HOLD);
  backright.set_brake_mode(MOTOR_BRAKE_HOLD);
  frontleft.set_brake_mode(MOTOR_BRAKE_HOLD);
  backleft.set_brake_mode(MOTOR_BRAKE_HOLD);
}

void moveLeftDriveTrain(int voltage){
  frontright.move_voltage(voltage);
  backright.move_voltage(voltage);
}

void moveRightDriveTrain(int voltage){
  frontleft.move_voltage(voltage);
  backleft.move_voltage(voltage);
}

void moveDriveVoltage(int voltage){
  frontright.move_voltage(voltage);
  backright.move_voltage(voltage);
  frontleft.move_voltage(voltage);
  backleft.move_voltage(voltage);
}

void moveDriveTrain(int voltage, double time){
  moveDriveVoltage(voltage);
  pros::delay(time*1000);
}

int motorAvgLeft(){
  return (backleft.get_position()+backright.get_position())/2;
}

int motorAvgRight(){
  return (frontright.get_position()+backright.get_position())/2;
}

int motorAvgAll(){
  return ((frontleft.get_position()+backleft.get_position()+frontright.get_position()+backright.get_position())/4);
}

double radToDeg(double radian){
    return(radian * (180/3.14159));
}

double degToRad(double degree){
  return(degree * (3.14159/180));
}

//300 ticks for a full rotation on a 600 rpm cartridge
//500 ticks for one full wheel rotation (300 * (60/36) or (5/3))
//Circumference of 3.25 in omni = 10.21 (3.25*pi)
//500 ticks / 10.21 in = 48.972 ticks per inch
int inchToTick(float inch){
  return(inch * 48.972);
}

int degreesToTicks (float degrees){
  return (degrees * 15);
}

float percentToVoltage (float percent){
  return(percent * 120);
}

float imuTarget(float target){
  if(target < 0){return 360 - fabs((fabs(target) - imu.get_heading()));}
  else{return fabs(target - imu.get_heading());}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float imuTarget2(float target){
   float ang = fabs((fabs(target)) - (imu.get_heading()));
   float ang2 = 360 - ang;
   /**
   *Classic code optimization, losing readability for speed
   *
   *While avoiding using conditionals I multiply the two values by either 1 or 0
   *and then add them to get the desired value
   */
   return((ang>ang2)*ang2+(ang2>=ang)*ang)*(target>0) + ((ang>ang2)*ang + (ang2>=ang)*ang2)*(target<0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void intake(int velocity){
  leftintake.move_velocity(velocity);
  rightintake.move_velocity(velocity);
}

void rollers(int velocity){
  mainroller1.move_velocity(velocity);
  mainroller2.move_velocity(velocity);
}

void index_fn(void* param){
  std::uint32_t startTime = pros::millis();
  while(true){
    if(indexTracker.get_value() > lineSensorSense && (topTrackerLeft.get_value()+topTrackerLeft.get_value())/2 > lineSensorSense){
      mainroller1.move_voltage(4000);
      mainroller2.move_voltage(4000);
    }
    else{mainroller1.move_voltage(0);mainroller2.move_voltage(0);}
    pros::Task::delay_until(&startTime, 10);
  }
}

void indexCount_fn(void* param){
  ballCount = 0;
  bool inTracker = false, last;
  bool inTrackerT = false, lastT;
  float topTrackerAvg;
  float bottomTrackerAvg;
  float bottomSensorThresh = lineSensorSense + bottomSensorThresh;
  std::uint32_t startTime = pros::millis();
  while(true){
    topTrackerAvg = (topTrackerLeft.get_value() + topTrackerRight.get_value())/2;
    bottomTrackerAvg = (bottomTrackerLeft.get_value() + bottomTrackerRight.get_value())/2;
    //Bool to check if ball is currently in bottom sensors
    if(bottomTrackerAvg < lineSensorSense){inBottom = true;}else{inBottom = false;}

    //Bool to check if ball is currently in indexing sensor
    if(indexTracker.get_value() < lineSensorSense){inIndex = true;}else{inIndex = false;}

    //Bool to check if ball is currently in top sensors
    if(topTrackerAvg < lineSensorSense){inTop = true;}else{inTop = false;}

    //Ball counter for bottom sensor
    if(bottomTrackerAvg < lineSensorSense){
      last = inTracker;
      inTracker = true;
      if(last == false && inTracker){ballCount++;}}
    else{inTracker = false;}

    //Ball counter for top sensors
    if(topTrackerAvg < lineSensorSense){
      lastT = inTrackerT;
      inTrackerT = true;
      if(lastT == false && inTrackerT){topBallCount++;}}
    else{inTrackerT = false;}
    master.print(2, 0, "Count: %i  Count: %i ", ballCount, topBallCount);
    pros::Task::delay_until(&startTime, 10);
  }
}


void endAIO(){taskNotification = false;}
void notifyAIO(std::string funcName, float timer){
  if     (funcName == "outtake"){taskNum = 1;}
  else if(funcName == "example"){taskNum = 2;}
  else if(funcName == "example"){taskNum = 3;}
  else if(funcName == "example"){taskNum = 4;}
  taskTimer = pros::millis() + timer;
  taskNotification = true;
}


void AIO_task_fn(void* ign) {
  std::uint32_t startTime = pros::millis();
  while(true){
    //If taskNotification is true start the stuff
    if(taskNotification == true){
      taskNotification = false;
      //Until the timer is up to start doing things
      while(taskTimer > pros::millis()){pros::Task::delay_until(&startTime, 10);}}
      //After timer is over begin new loop
      switch(taskNum){
        case(1): intake(-200); rollers(-200); return;

        case(2): return;

        case(3): return;

        case(4): return;
      }
    pros::Task::delay_until(&startTime, 10);
  }
}




void smartDriveMove(int voltage, int ballNum, long time){
  float endTime = pros::millis() + time;
  while(pros::millis() < endTime){
    frontright.move_voltage(voltage);
    backright.move_voltage(voltage);
    frontleft.move_voltage(voltage);
    backleft.move_voltage(voltage);
    if(topBallCount >= ballNum){rollers(0);}
    pros::delay(10);
  }
}

void deploy(){
  rollers(-200);
  pros::delay(200);
  mainroller1.move_voltage(0);
  mainroller2.move_voltage(0);
}

void intakeTilInBot(float timeOut, int rollerSpeed){
  float endTime = pros::millis() + timeOut;
  float movementTimer;
  intake(200);
  rollers(rollerSpeed);
  while(!inBottom && pros::millis() < endTime){
    movementTimer = pros::millis() + 75;
    while(pros::millis() < movementTimer){
      frontright.move_voltage(-7000);
      backright.move_voltage(-7000);
      frontleft.move_voltage(-7000);
      backleft.move_voltage(-7000);
      if(inBottom){intake(0);rollers(0);}
      pros::delay(10);
    }
    movementTimer = pros::millis() + 125;
    while(pros::millis() < movementTimer){
      frontright.move_voltage(12000);
      backright.move_voltage(12000);
      frontleft.move_voltage(12000);
      backleft.move_voltage(12000);
      if(inBottom){intake(0);rollers(0);}
      pros::delay(10);
    }
  }
  intake(0);
  rollers(0);
}

void intakeTilFull(float timeOut, int rollerSpeed){
  float endTime = pros::millis() + timeOut;
  float movementTimer;
  intake(200);
  rollers(rollerSpeed);
  while((!inBottom || !(inIndex || inTop)) && pros::millis() < endTime){
    movementTimer = pros::millis() + 50;
    while(pros::millis() < movementTimer){
      frontright.move_voltage(-7000);
      backright.move_voltage(-7000);
      frontleft.move_voltage(-7000);
      backleft.move_voltage(-7000);
      if(inIndex || inTop){rollers(0);}
      else{rollers(rollerSpeed);}
      pros::delay(10);
    }
    movementTimer = pros::millis() + 100;
    while(pros::millis() < movementTimer){
      frontright.move_voltage(12000);
      backright.move_voltage(12000);
      frontleft.move_voltage(12000);
      backleft.move_voltage(12000);
      if(inIndex || inTop){rollers(0);}
      else{rollers(rollerSpeed);}
      pros::delay(10);
    }
  }
  intake(0);
  rollers(0);
}


void cycle(float timeOut, int ball1, int ball2, int speed, bool shake = false){
  ballCount = 0; topBallCount = 0;
  rollers(speed);
  if(shake){
    while(ballCount < ball1 && topBallCount < ball2 && pros::millis() < timeOut){
      smartDriveMove(-7000, ball1, 50);
      smartDriveMove(12000, ball1, 100);
      pros::delay(10);
    }
    intake(-50);
    while(topBallCount < ball2 && pros::millis() < timeOut){
      rollers(speed);
      smartDriveMove(-7000, ball2, 50);
      smartDriveMove(12000, ball2, 100);
      pros::delay(10);
    }
  }
  else{
    while(ballCount < ball1 && topBallCount < ball2 && pros::millis() < timeOut){pros::delay(10);}
    intake(-50);
    while(topBallCount < ball2 && pros::millis() < timeOut){pros::delay(5);}
  }
}

void goodCycle(float timeOut, int ball1, int ball2, int speed, bool shake = false){
  rollers(speed);
  if(shake){
    while(ballCount < ball1 && pros::millis() < timeOut){
      smartDriveMove(-7000, ball1, 50);
      smartDriveMove(12000, ball1, 100);
      pros::delay(10);
    }
    intake(-50);
    while(topBallCount < ball2 && pros::millis() < timeOut){
      rollers(speed);
      smartDriveMove(-7000, ball2, 50);
      smartDriveMove(12000, ball2, 100);
      pros::delay(10);
    }
  }
  else{
    while(ballCount < ball1 && topBallCount < ball1 && pros::millis() < timeOut){pros::delay(10);}
    intake(-50);
    while(topBallCount < ball2 && pros::millis() < timeOut){pros::delay(5);}
  }
}

void resetBallCount(){
  ballCount = 0;
  topBallCount = 0;
}

class triangle {
  public:
    float a;
    float b;
    float hyp;
    float alpha;
    float beta;
}; triangle tri;

void findTri(float a, float reference){
  tri.beta = degToRad(imu.get_heading()-reference);
  tri.alpha = 90 - tri.beta;
  tri.a = a;
  tri.b =  a * tan(tri.beta);
  tri.hyp = sqrt(a*a + tri.b * tri.b);
}
