#include "auton_functions.cpp"

class Drive{
  private:
    //Initialize PID Tuning Values
    float pidConstants[4][7] = {
      {25,600,15,100,70,2000,100},
      {25,400 ,15,750 ,70,1500 ,100},
      {25,400 ,15,750 ,70,1100 ,100},
      {25,600,15,100,70,1200,100}
    };
    float kP;
    float kP_t;
    float kI;
    float kI_t;
    float kD;
    float kD_t;
    float kP_d;

  public:
    //Set the PID to the first profile when a new instance is initiliazed
    Drive(){setPID(1);}

    // Member functions declaration
    void setPID(int n);
    void move(const std::string&direction, float target, float timeout, float maxVelocity);
    void swerve(const std::string&direction, float targetDistance, float targetAngle, float timeout, float maxVelDist, float maxVelAng);
    void hardStop(const std::string&direction, float targetCutOff, float target, float maxVelocity);
};Drive drive;

//Member functions definitions

//Set PID tuning values
void Drive::setPID(int n){
  kP = pidConstants[n-1][0];
  kP_t = pidConstants[n-1][1];
  kI = pidConstants[n-1][2];
  kI_t = pidConstants[n-1][3];
  kD = pidConstants[n-1][4];
  kD_t = pidConstants[n-1][5];
  kP_d = pidConstants[n-1][6];;
}

//Basic PID movement function
void Drive::move(const std::string &direction, float target, float timeOut, float maxVelocity){
  //Error values//
  float error;
  float progress;
  float error_drift;
  float lastError;
  float lastError_d;
  float inertRoll;
  float initialAngle = imu.get_rotation() + 360;
  float initialMotorAvg = motorAvgAll();
  //Calc var declarations//
  float proportion;
  float proportion_drift;
  float intergral;
  float derivative;
  float derivative_d;
  ////////////////////////
  float intergralActive = inchToTick(8);
  float intergralActive_t = 3;
  //Motor output var declarations//
  float targetVolt;
  float finalVolt;
  float finalPct;
  //Timeout var declaration
  int errorCycles = 0;
  int standStillTolerance = 5;
  bool standStill = false;
  float endTime = pros::millis() + timeOut*1000;
  if(direction == "forward" || direction == "backward"){
    float intergralLimit = (maxVelocity/kI)/50;
    //Begin the PID loop
    while((pros::millis() < endTime && !standStill)){
      progress = fabs(motorAvgAll() - initialMotorAvg);
      error = inchToTick(target) - progress;
      proportion = error;
      if(fabs(error) < intergralActive){intergral += error;}
      else{intergral = 0;}
      if(intergral > intergralLimit){intergral = intergralLimit;}
      else if(intergral < intergralLimit){intergral = -intergralLimit;}
      derivative = error - lastError;
      lastError = error;
      if(error == 0){derivative = 0;}
      targetVolt = percentToVoltage(maxVelocity);
      finalVolt = kP*proportion + kI*intergral + kD*derivative;
      if(finalVolt > targetVolt){finalVolt = targetVolt;}
      else if(finalVolt < -targetVolt){finalVolt = -targetVolt;}
      error_drift = imu.get_rotation() + 360 - initialAngle;
      derivative = error_drift - lastError_d;
      lastError_d = error_drift;
      proportion_drift = error_drift * kP_d;
      if(fabs(error)/30 < 1 && standStill == false){errorCycles += 1;
        if(errorCycles > standStillTolerance){standStill = true;}}
      if(direction == "forward"){
        moveRightDriveTrain(finalVolt-proportion_drift);
        moveLeftDriveTrain(finalVolt+proportion_drift);}
      else if(direction == "backward"){
        moveRightDriveTrain(-finalVolt-proportion_drift);
        moveLeftDriveTrain(-finalVolt+proportion_drift);}
      //Give PROS time to keep itself in order
      pros::delay(20);
    }
    //Stop the robot for 20 milliseconds and then exit the function
    DriveTrainBrake();
    moveDriveVoltage(0);
    pros::delay(20);
    DriveTrainCoast();
    return;
  }
  else if(direction == "right" || direction == "left"){
    float intergralLimit_t = (maxVelocity/kI_t)*10;
    //Begin the PID loop
    while((pros::millis() < endTime && !standStill)){
      error = target - fabs(imu.get_rotation() + 360 - initialAngle);
      proportion = error;
      if(fabs(error) < intergralActive_t){
        intergral = intergral + error;
        if(intergral > intergralLimit_t){intergral = intergralLimit_t;}
        else if(intergral < intergralLimit_t){intergral = -intergralLimit_t;}
      }
      else{intergral = 0;}
      derivative = error - lastError;
      lastError = error;
      if(error == 0){derivative = 0;}
      targetVolt = percentToVoltage(maxVelocity);
      finalVolt = kP_t*proportion + kI_t*intergral + kD_t*derivative;
      if(finalVolt > targetVolt){finalVolt = targetVolt;}
      else if(finalVolt < -targetVolt){finalVolt = -targetVolt;}
      //master.print(1, 0, "%.2f, %.2f, %.2f   ", error, target, progress);
      if(fabs(error) < 1.5){errorCycles += 1;
        if(errorCycles > standStillTolerance*2){standStill = true;}}
        if(direction == "right"){
          moveRightDriveTrain(finalVolt);
          moveLeftDriveTrain(-finalVolt);}
        else if(direction == "left"){
          moveRightDriveTrain(-finalVolt);
          moveLeftDriveTrain(finalVolt);}
        //Give PROS time to keep itself in order
        pros::delay(20);
    }
    //Stop the robot for 20 milliseconds and then exit the function
    DriveTrainBrake();
    moveDriveVoltage(0);
    pros::delay(20);
    DriveTrainCoast();
    return;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Drive::swerve(const std::string&direction,float targetDistance,float targetAngle,float timeOut,float maxVelDist,float maxVelAng){
  float finalValueLeft;
  float finalValueRight;
  float progress;
  float errorDistance;
  float errorAngle;
  float lastErrorDistance;
  float lastErrorAngle;
  //Calc var declarations//
  float proportionDistance;
  float proportionAngle;
  float intergralDistance;
  float intergralAngle;
  float derivativeDistance;
  float derivativeAngle;
  ////////////////////////
  float intergralActiveDistance = inchToTick(4);
  float intergralActiveAngle = 10;
  float intergralLimitDistance = (maxVelDist/kI)/50;

  float intergralActive = inchToTick(8);
  float intergralActive_t = 3;
  //Motor output var declarations//
  float targetVolt;
  float finalVolt;
  float finalPct;

  float intergralLimitAngle = (maxVelAng/kI_t)/50;
  float initialMotorAvg = motorAvgAll();
  float initialAngle = imu.get_rotation() + 360;


  float endTime = pros::millis() + timeOut*1000;
  //Begin the PID loop
  while(pros::millis() < endTime){
    finalValueRight = 0;
    finalValueLeft = 0;
    /////////////////////////////////////////////////////////////////////////// DRIVE
    progress = fabs(motorAvgAll() - initialMotorAvg);
    errorDistance = inchToTick(targetDistance) - progress;
    proportionDistance = errorDistance;
    if(fabs(errorDistance) < intergralActiveDistance){intergralDistance += errorDistance;}
    else{intergralDistance = 0;}
    if(intergralDistance > intergralLimitDistance){intergralDistance = intergralLimitDistance;}
    else if(intergralDistance < intergralLimitDistance){intergralDistance = -intergralLimitDistance;}
    derivativeDistance = errorDistance - lastErrorDistance;
    lastErrorDistance = errorDistance;
    if(errorDistance == 0){derivativeDistance = 0;}
    targetVolt = percentToVoltage(maxVelDist);
    finalVolt = kP*proportionDistance + kI*intergralDistance + kD*derivativeDistance;
    if(finalVolt > targetVolt){finalVolt = targetVolt;}
    else if(finalVolt < -targetVolt){finalVolt = -targetVolt;}
    if(direction.find("forward") != std::string::npos){
      finalValueRight += finalVolt;
      finalValueLeft += finalVolt;}
    else{
      finalValueRight += -finalVolt;
      finalValueLeft += -finalVolt;}
    /////////////////////////////////////////////////////////////////////////// TURN
    errorAngle = targetAngle - fabs(imu.get_rotation() + 360 - initialAngle);
    proportionAngle = errorAngle;
    //master.print(2, 0, "%.2f, %.2f,            ", errorAngle, imu.get_rotation() + 360 - initialAngle);
    if(fabs(errorAngle) < intergralActiveAngle){intergralAngle = intergralAngle + errorAngle;}
    else{intergralAngle = 0;}
    if(intergralAngle > intergralLimitAngle){intergralAngle = intergralLimitAngle;}
    else if(intergralAngle < intergralLimitAngle){intergralAngle = -intergralLimitAngle;}
    derivativeAngle = errorAngle - lastErrorAngle;
    lastErrorAngle = errorAngle;
    if(errorAngle == 0){derivativeAngle = 0;}
    targetVolt = percentToVoltage(maxVelAng);
    finalVolt = kP_t*proportionAngle + kI_t*intergralAngle + kD_t*derivativeAngle;
    if(finalVolt > targetVolt){finalVolt = targetVolt;}
    else if(finalVolt < -targetVolt){finalVolt = -targetVolt;}
    if(direction.find("Right") != std::string::npos){
      finalValueRight += finalVolt;
      finalValueLeft += -finalVolt;}
    else{
      finalValueRight += -finalVolt;
      finalValueLeft += finalVolt;}

    moveRightDriveTrain(finalValueRight);
    moveLeftDriveTrain(finalValueLeft);
    //Give PROS time to keep itself in order
    pros::delay(20);
  }
  //Stop the robot for 20 milliseconds and then exit the function
  DriveTrainBrake();
  moveDriveVoltage(0);
  pros::delay(20);
  DriveTrainCoast();
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Drive::hardStop(const std::string&direction, float targetCutOff, float target, float maxVelocity){
  float error;
  float progress;
  float error_drift;
  float lastError;
  float lastError_d;
  float inertRoll;
  float initialAngle = imu.get_rotation() + 360;
  float initialMotorAvg = motorAvgAll();
  //Calc var declarations//
  float proportion;
  float proportion_drift;
  float intergral;
  float derivative;
  float derivative_d;
  ////////////////////////
  float intergralActive = inchToTick(8);
  float intergralActive_t = 3;
  //Motor output var declarations//
  float targetVolt;
  float finalVolt;
  float intergralLimit = (maxVelocity/kI)/50;
  //Begin PID
  while(targetCutOff > fabs(motorAvgAll()-initialMotorAvg)){
    progress = fabs(motorAvgAll() - initialMotorAvg);
    error = inchToTick(target) - progress;
    proportion = error;
    if(fabs(error) < intergralActive){intergral += error;}
    else{intergral = 0;}
    if(intergral > intergralLimit){intergral = intergralLimit;}
    else if(intergral < intergralLimit){intergral = -intergralLimit;}
    derivative = error - lastError;
    lastError = error;
    if(error == 0){derivative = 0;}
    targetVolt = percentToVoltage(maxVelocity);
    finalVolt = kP*proportion + kI*intergral + kD*derivative;
    if(finalVolt > targetVolt){finalVolt = targetVolt;}
    else if(finalVolt < -targetVolt){finalVolt = -targetVolt;}
    error_drift = imu.get_rotation() + 360 - initialAngle;
    derivative = error_drift - lastError_d;
    lastError_d = error_drift;
    proportion_drift = error_drift * kP_d;

    if(direction == "forward"){
      moveRightDriveTrain(finalVolt-proportion_drift);
      moveLeftDriveTrain(finalVolt+proportion_drift);}
    else if(direction == "backward"){
      moveRightDriveTrain(-finalVolt-proportion_drift);
      moveLeftDriveTrain(-finalVolt+proportion_drift);}

    //Give PROS time to keep itself in order
    pros::delay(20);
  }
  //Exit the function
  return;
}
