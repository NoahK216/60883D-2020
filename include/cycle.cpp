#include "include.cpp"
#include "auton_functions.cpp"

void newCycle(std::string type, float timeOut, int speed, int ball1, int ball2 = 0){
  ballCount = 0; topBallCount = 0;
  rollers(speed);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(type == "cycleTop"){while(topBallCount < ball1 && pros::millis() < timeOut){pros::delay(10);}}
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  else if(type == "cycleTopShake"){while(topBallCount < ball1 && pros::millis() < timeOut){
    smartDriveMove(-7000, ball1, 50);
    smartDriveMove(12000, ball1, 100);
    pros::delay(10);}
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  else if(type == "cycle"){
    while(ballCount < ball1 && topBallCount < ball1 && pros::millis() < timeOut){pros::delay(10);}
    intake(-50);
    while(topBallCount < ball2 && pros::millis() < timeOut){pros::delay(10);}
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  else if(type == "cycleShake"){
  while(ballCount < ball1 && pros::millis() < timeOut){
    smartDriveMove(-7000, ball1, 50);
    smartDriveMove(12000, ball1, 100);
    pros::delay(10);}
  intake(-50);
  while(topBallCount < ball2 && pros::millis() < timeOut){
    rollers(speed);
    smartDriveMove(-7000, ball2, 50);
    smartDriveMove(12000, ball2, 100);
    pros::delay(10);}
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
