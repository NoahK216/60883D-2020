#include "auton_functions.cpp"

void drivePID(const std::string direction, float input1, float input2, float input3, float input4 = 1, float input5 = 1, float input6 = 1){
  //Constants//
  //Assign pidValue depending on direction value
  int pidValue;
  float target;
  float targetDistance;
  float targetAngle;
  float targetCutOff;
  float timeOut;
  float maxVelocity;
  float maxVelocityDistance;
  float maxVelocityAngle;
  if(direction.find("hardstop") != std::string::npos){pidValue = (int)input4;}
  else if(direction.find("swerve") != std::string::npos){pidValue = (int)input6; timeOut = input3;}
  else{pidValue = (int)input4; timeOut = input2;}
  //Assign PID tuning values depending on direction value//
  float pidConstants[4][7] = {
  {25,600,15,100,70,2000,100},

  {25,400 ,15,750 ,70,1500 ,100},

  {25,400 ,15,750 ,70,1100 ,100},

  {25,600,15,100,70,1200,100}
  };
  float kP = pidConstants[pidValue-1][0];
  float kP_t = pidConstants[pidValue-1][1];
  float kI = pidConstants[pidValue-1][2];
  float kI_t = pidConstants[pidValue-1][3];
  float kD = pidConstants[pidValue-1][4];
  float kD_t = pidConstants[pidValue-1][5];
  //Drift constants//
  float kP_d = pidConstants[pidValue-1][6];
  //Error var declarations//
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
  float endTime = pros::millis() + timeOut*1000;
  int errorCycles = 0;
  int standStillTolerance = 5;
  bool standStill = false;

  //Check for direction value and then run PID loop accordingly
  if(direction == "forward" || direction == "backward"){
    target = input1;
    maxVelocity = input3;
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  else if(direction == "right" || direction == "left"){
    target = input1;
    maxVelocity = input3;
    float intergralLimit_t = (maxVelocity/kI_t)*10;
    //Begin the PID loop
    while((pros::millis() < endTime && !standStill)){
      error = target - fabs(imu.get_rotation() + 360 - initialAngle);
      proportion = error;

      //intergralLimit_t = (maxVelocity/kI_t)/1;

      if(fabs(error) < intergralActive_t){
        intergral = intergral + error;
        if(intergral > intergralLimit_t){intergral = intergralLimit_t;}
        else if(intergral < intergralLimit_t){intergral = -intergralLimit_t;}
      }

      else{intergral = 0;}



      //master.print(1, 0, "%.2f, %.2f %.2f             ", proportion, intergral, derivative);

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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  else if(direction.find("hardstop") != std::string::npos){
    targetCutOff = inchToTick(input1);
    target = input2;
    maxVelocity = input3;
    float intergralLimit = (maxVelocity/kI)/50;
    float intergralLimit_t = (maxVelocity/kI_t)/1;
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

      if(direction == "hardstopForward"){
        moveRightDriveTrain(finalVolt-proportion_drift);
        moveLeftDriveTrain(finalVolt+proportion_drift);}
      else if(direction == "hardstopBackward"){
        moveRightDriveTrain(-finalVolt-proportion_drift);
        moveLeftDriveTrain(-finalVolt+proportion_drift);}

      //Give PROS time to keep itself in order
      pros::delay(20);
    }
    //Exit the function
    return;
  }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  else if(direction.find("swerve") != std::string::npos){
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
    float intergralLimitDistance = (maxVelocityDistance/kI)/50;
    float intergralLimitAngle = (maxVelocityAngle/kI_t)/50;
    targetDistance = input1;
    targetAngle = input2;
    maxVelocityDistance = input4;
    maxVelocityAngle = input5;
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
      targetVolt = percentToVoltage(maxVelocityDistance);
      finalVolt = kP*proportionDistance + kI*intergralDistance + kD*derivativeDistance;
      if(finalVolt > targetVolt){finalVolt = targetVolt;}
      else if(finalVolt < -targetVolt){finalVolt = -targetVolt;}
      if(direction.find("backward") != std::string::npos){
        finalValueRight += -finalVolt;
        finalValueLeft += -finalVolt;}
      else{
        finalValueRight += finalVolt;
        finalValueLeft += finalVolt;}
      ///////////////////////////////////////////////////////////////////////// TURN
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
      targetVolt = percentToVoltage(maxVelocityAngle);
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
}
