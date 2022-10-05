#include "include.cpp"
#include "auton_functions.cpp"
#include "PID.cpp"

bool threegoal = true;
void homerow(){
  //Record start time of auton for future timing
  float autonStart = pros::millis();
  //Initialize Tasks
  pros::Task Indexing (index_fn, (void*)"PROS", "Indexing");
  pros::Task Count (indexCount_fn, (void*)"PROS", "Count Balls Cycled");
  Indexing.suspend();
  //Deploy
  deploy();
  Indexing.resume();
  //Swerve to the right and intake
  intake(200);
  drivePID("swerveRight", 32, 40, 0.7, 80, 23);
  //Suspend indexing
  Indexing.suspend();
  //Cycle three balls and leave one in the bot to descore
  moveDriveVoltage(1000);
  rollers(200);
  cycle(autonStart + 3500, 3, 3, 200, true);
  pros::delay(100);
  rollers(-200);
  //Calculate the distance needed to back up for next movements
  findTri(20,0);
  //Back away and turn to absolute 180 degrees
  intake(-150);

  drivePID("backward", tri.hyp-5.65, 1, 70);
  rollers(0);
  intake(0);
  drivePID("right", imuTarget(180), 1, 65 , 3);
  //Drive forward and then swerve
  drivePID("hardstopForward", 44-tri.b, 65-tri.b, 75);//43, 67
  resetBallCount();
  intake(200);
  drivePID("swerveLeft", 40, imuTarget(90), 1, 60, 90);
  //Cycle 1 ball and keep 1 in the bot to descore
  goodCycle(pros::millis() + 3000, 2, 1, 200, true);
  //Drive forward briefly to ensure bot is alligned with the goal
  moveDriveTrain(4000, 0.1);//was 0.2
  rollers(-200);
  moveDriveTrain(4000, 0.15);
  //Calculate the distance needed to back up for next movements
  findTri(22,90);//was 20
  //master.print(2, 0, "%.2f, %.2f, %.2f        ", tri.hyp, tri.b, tri.b);
  //Back away from goal and turn to absolute 180 degrees
  intake(0);
  rollers(0);
  drivePID("backward", tri.hyp-5.65, 0.7, 80);


  if(threegoal){
  intake(-200);
  rollers(-200);
  drivePID("left", 45, 0.7, 100);
  intake(0);
  intake(0);
  drivePID("right", imuTarget(180), 1.3, 60, 3);
  rollers(0);
  intake(0);
  //Drive forward and turn into last homerow goal
  drivePID("forward", 41-(-tri.b), 1.2, 65);//was 43
  drivePID("left", imuTarget(135), 0.8, 80, 3);
  intake(200);
  resetBallCount();
  drivePID("forward", 40, 1, 60);
  //Cycle
  moveDriveVoltage(2000);
  goodCycle(pros::millis() + 2000, 3, 2, 200);
  pros::delay(100);
  rollers(0);
  pros::delay(100);
  rollers(-100);
  drivePID("backward", 20, 1, 40);
  }
  else{rollers(0);intake(0);}

  Count.remove();
  Indexing.remove();
}

void gaming(){
  //Record start time of auton for future timing
  float autonStart = pros::millis();
  //Initialize Tasks
  pros::Task Indexing (index_fn, (void*)"PROS", "Indexing");
  pros::Task Count (indexCount_fn, (void*)"PROS", "Count Balls Cycled");
  Indexing.suspend();
  //Deploy
  deploy();
  Indexing.resume();
  //Swerve to the right and intake
  intake(200);
  drivePID("swerveleft", 32, 40, 0.7, 80, 23);
  //Suspend indexing
  Indexing.suspend();
  //Cycle three balls and leave one in the bot to descore
  moveDriveVoltage(1000);
  rollers(200);
  cycle(autonStart + 3500, 3, 3, 200, true);
  pros::delay(100);
  rollers(-200);
  //Back away and turn to absolute 180 degrees
  intake(-150);

  drivePID("backward", 16, 1, 70);
}

void middle_row(){
  float autonStart = pros::millis();
  pros::Task Count (indexCount_fn, (void*)"PROS", "Count Balls Cycled");
  //DEPLOY HOOD
  deploy();
  //CREATE INDEXING Task
  pros::Task Indexing (index_fn, (void*)"PROS", "Indexing");

  intake(200);
  drivePID("forward", 47, 2, 60);
  if(imu.get_heading() > 180){drivePID("left", imuTarget(300), 0.8, 80, 3);}
  else{drivePID("left", imuTarget(-300), 0.8, 80, 3);}

  intake(0);

  drivePID("backward", 44, 2, 90);

  drivePID("right", imuTarget(360), 0.8, 80, 3);
  Indexing.suspend();
  rollers(0);

  intake(-100);
  moveDriveTrain(6000, 0.4);

  intake(200);
  moveDriveTrain(6000, 0.4);
  intake(0);
  moveDriveVoltage(0);

  rollers(200);
  pros::delay(1000);

  rollers(0);

  drivePID("backward", 12, 2, 80);

  if(imu.get_heading() > 180){drivePID("right", imuTarget(-90), 0.8, 70, 3);}
  else{drivePID("right", imuTarget(90), 0.8, 70, 3);}


  intake(-200);
  rollers(-200);
  drivePID("hardstopForward", 24, 55, 80);
//////////////////////////////////////////////////////
  Indexing.resume();
  intake(200);
  drivePID("swerveLeft", 37.5, imuTarget(50), 1, 70, 80);
  moveDriveVoltage(2000);

  pros::delay(1500);

  drivePID("backward", 6, 2, 60);
  drivePID("right", 90, 2, 100);

  Indexing.suspend();



  resetBallCount();
  intake(-200);
  rollers(200);
  float timeOut = pros::millis() + 1000;
  while(topBallCount < 1 && pros::millis() < timeOut){pros::delay(10);}



  intake(0);
  rollers(-100);
  pros::delay(200);

  drivePID("left", imuTarget(50), 0.8, 60);

  intake(-200);
  moveDriveVoltage(3000);

  Indexing.remove();
  Count.remove();

  pros::delay((autonStart + 14500) - pros::millis());
  moveDriveTrain(-6000, 0.5);


  moveDriveVoltage(1000);
  pros::delay(50);
}



void skills(){
  float autonStart = pros::millis();
  float funcStart;
  int inBot;
  pros::Task Count (indexCount_fn, (void*)"PROS", "Count Balls Cycled");

  intake(200);
  rollers(-200);
  drivePID("hardstopForward", 5, 15, 50);//43, 67
  pros::Task Indexing (index_fn, (void*)"PROS", "Indexing");
  drivePID("swerveRight", 49, 20, 1.3, 70, 80);


  intake(100);
  drivePID("backward", 12.5, 0.9, 85);
  drivePID("left", imuTarget(-300), 0.5, 60);
                                                              //FIRST GOAL//
  //Drive to goal and cycle
  Indexing.suspend();
  rollers(0);
  intake(25);
  drivePID("forward", 25, 0.7, 80);

  moveDriveVoltage(2000);
  cycle(pros::millis() + 2000, 2, 2, 200);
  intake(0);
  rollers(200);
  pros::delay(100);
  rollers(0);
  pros::delay(50);
  intakeTilFull(2000, 75);

  rollers(0);
  intake(0);
  moveDriveVoltage(0);

  //Calculate the distance needed to back up for next movements
  findTri(39, 270);//was 41
  //Drive backwards with new triangle information
  rollers(-200);
  intake(-200);
  pros::delay(25);
  drivePID("backward", tri.hyp-5.65, 1.5, 80);

  //pros::delay(10000);

  //Turn to absolute 180 degrees and drive to intake ball
  rollers(0);
  intake(0);
  drivePID("left", imuTarget(180), 1, 60);
  Indexing.resume();
  intake(200);


  drivePID("forward", 65-tri.b, 1, 80);
  //Turn after intaking ball and drive to second goal
  drivePID("right", imuTarget(270), 0.8, 80, 2);
  intake(200);
  drivePID("forward", 35, 0.7, 100);


                                                              //SECOND GOAL//
  Indexing.suspend();

  intake(0);
  moveDriveVoltage(2000);
  cycle(pros::millis() + 1000, 2, 2, 200);
  rollers(0);
  intakeTilInBot(1500, 75);

  intake(-200);

  //pros::delay(200);

  //Calculate the distance needed to back up for next movements
  findTri(14,270);
  //Back away from goal


  drivePID("backward", tri.hyp-5.65, 0.8, 60);
  //Turn to outtake ball then to drive forward and get a ball then turn left again to get another
  rollers(-200);
  drivePID("right", 45, 0.5, 100);
  drivePID("left", imuTarget(180), 0.8, 65);
  Indexing.resume();
  intake(200);

  drivePID("hardstopForward", 30-tri.b, 60-tri.b, 75);//43, 67
  drivePID("swerveLeft", 32, imuTarget(155), 1, 70, 80);


  //Back up and turn towards goal
  drivePID("backward", 20, 1, 80);
  intake(0);
  drivePID("right", imuTarget(215), 1, 80, 3);
                                                              //GOAL THREE//
  //Drive forward and cycle 2
  Indexing.suspend();
  rollers(0);
  intake(200);
  drivePID("forward", 26, 0.7, 75);//was 25
  inBot = inBottom + inIndex;

  moveDriveVoltage(2000);
  cycle(pros::millis() + 2000, 2, inBot, 200);
  intake(0);
  rollers(200);
  pros::delay(100);
  rollers(0);
  pros::delay(50);
  intakeTilFull(2000, 75);

  rollers(0);
  intake(0);
  moveDriveVoltage(0);
  //Calculate triangle needed for next movements
  findTri(41, 180);
  //Back away from goal and outtake balls
  rollers(-200);
  intake(-100);
  pros::delay(25);
  drivePID("backward", tri.hyp-5.65, 1.5, 85);
  rollers(0);
  intake(0);
  drivePID("left", imuTarget(90), 1, 75, 3);
  //Drive forward to intake ball then turn
  Indexing.resume();
  intake(200);


  drivePID("forward", 66-tri.b, 1, 80);//was 65


  intake(0);
  drivePID("right", imuTarget(180), 1, 80, 1);
                                                                //GOAL FOUR//
  intake(200);
  drivePID("hardstopForward", 18, 24, 80);
  Indexing.suspend();
  rollers(0);
  drivePID("forward", 18, 0.55, 80);

  moveDriveVoltage(2000);
  inBot = inBottom + inIndex;
  intake(0);
  cycle(pros::millis() + 1000, inBot, inBot, 200);
  rollers(0);
  intakeTilInBot(1500, 75);


  //Calculate triangle needed for next movements
  findTri(27, 180); // was 26
  //Backup and turn left to intake a red ball and score 5th goal
  intake(-200);


  drivePID("backward", tri.hyp-5.65, 2, 85);

  lineSensorSense = 2800;


  //Turn to outtake ball then to drive forward and get a ball then turn left again to get another
  rollers(-200);
  drivePID("right", 45, 0.5, 100);


  drivePID("left", imuTarget(90), 2, 60);
  intake(200);
  Indexing.resume();
  drivePID("forward", 49-tri.b, 2, 80);//was 50
  drivePID("right", imuTarget(200), 2, 80, 2);

  drivePID("forward", 25, 1, 80);

  drivePID("backward", 14, 0.9, 90);

  drivePID("left", imuTarget(130), 0.9, 70);

  intake(0);
  //Turn towards goal 5

  Indexing.suspend();
  rollers(0);
  //Drive forward and score 1;
  inBot = inBottom + inIndex;
  drivePID("forward", 45, 0.7, 100);
                                                              //GOAL FIVE//
  intake(200);
  moveDriveVoltage(2000);
  cycle(pros::millis() + 2000, 2, inBot, 200, true);
  intake(0);
  rollers(200);
  pros::delay(100);
  rollers(0);
  pros::delay(50);
  intakeTilFull(2000, 75);
  rollers(0);
  intake(0);
  moveDriveVoltage(0);
  findTri(41  ,90);//was 41
  //Drive backwards with new triangle information
  rollers(-200);
  intake(-200);
  pros::delay(25);
  drivePID("backward", tri.hyp-5.65, 1.5, 80);
  //Turn to absolute 180 degrees and drive to intake ball
  rollers(0);
  intake(0);
  lineSensorSense = 2750;
  drivePID("left", imuTarget(0), 1, 60);
  Indexing.resume();
  intake(200);
  drivePID("forward", 64.5-tri.b, 1, 80);//was 66-tri.b
  //Turn after intaking ball and drive to second goal
  if(imu.get_heading() > 180){drivePID("right", imuTarget(-90), 0.8, 60);}
  else{drivePID("right", imuTarget(90), 0.8, 60);}
  Indexing.suspend();
  rollers(0);
  intake(0);
  drivePID("forward", 40, 0.8, 100);
                                                                  //GOAL 6//
  inBot = inBottom + inIndex;
  resetBallCount();
  moveDriveVoltage(2000);
  intake(200);
  cycle(pros::millis() + 2000, 1, inBot, 200);
  rollers(100);
  pros::delay(200);
  if(!ballCount){intakeTilInBot(2000, 75);}
  //Calculate the distance needed to back up for next movements
  rollers(0);
  findTri(14,90);
  //Back away from goal
  intake(-200);
  drivePID("backward", tri.hyp-5.65, 0.8, 60);
  //Turn to outtake ball then to drive forward and get a ball then turn left again to get another
  rollers(-200);
  drivePID("right", 45, 0.5, 100);
  drivePID("left", imuTarget(0), 0.9, 60);
  Indexing.resume();
  intake(200);


  drivePID("hardstopForward", 30-tri.b, 60-tri.b, 75);//43, 67
  if(imu.get_heading() > 180){drivePID("swerveLeft", 32, imuTarget(335), 1, 70, 80);}
  else{drivePID("swerveLeft", 32, imuTarget(-335), 1, 70, 80);}


  //Back up and turn towards goal
  drivePID("backward", 20, 1, 60);

  drivePID("right", imuTarget(-35), 1, 60);
  intake(0);



                                                                  //GOAL 7//
  //Drive forward and cycle 2
  Indexing.suspend();
  rollers(0);

  intake(200);

  drivePID("forward", 23, 0.7, 80);
  inBot = inBottom + inIndex;

  moveDriveVoltage(2000);
  cycle(pros::millis() + 2000, 2, inBot, 200);
  intake(0);
  rollers(200);
  pros::delay(100);
  rollers(0);
  pros::delay(50);
  intakeTilFull(2000, 75);

  rollers(0);
  intake(0);
  moveDriveVoltage(0);


  //Calculate triangle needed for next movements
  findTri(41, 0);
  //Back away from goal and outtake balls
  rollers(-200);
  intake(-200);
  pros::delay(25);
  drivePID("backward", tri.hyp-5.65, 2, 80);
  rollers(0);
  intake(0);
  if(imu.get_heading() > 180){drivePID("left", imuTarget(270), 1, 75);}
  else{drivePID("left", imuTarget(-270), 1, 75);}
  //Drive forward to intake ball then turn
  Indexing.resume();
  intake(200);
  drivePID("forward", 66-tri.b, 2, 60);
  intake(0);
  drivePID("right", imuTarget(360), 0.8, 80, 3);
  intake(200);

  drivePID("forward", 36, 1, 95);
                                                              //GOAL EIGHT//
  Indexing.suspend();
  rollers(0);
  intake(0);
  moveDriveVoltage(2000);
  cycle(pros::millis() + 1500, 1, 1, 200);
  pros::delay(100);
  intakeTilInBot(1000, 0);
  pros::delay(75);
  rollers(-50);
  //Do funky turning stuff


  if(imu.get_heading() > 180){drivePID("left", imuTarget(334), 0.6, 100);}
  else{drivePID("left", imuTarget(-334), 0.6, 100);}

  rollers(0);

  drivePID("backward", 8, 2, 60);

  Indexing.resume();
  intake(-200);
  drivePID("left", imuTarget(180), 1, 50);

                                                              //GOAL NINE//
  drivePID("forward", 39, 1, 80);
  Indexing.suspend();
  rollers(0);
  drivePID("backward", 6, 0.3, 40);
  drivePID("forward", 6, 0.5, 50);

  drivePID("backward", 6, 0.3, 40);
  drivePID("forward", 6, 0.5, 50);

  moveDriveVoltage(4000);
  rollers(200);
  pros::delay(400);
  moveDriveTrain(-4000, 0.5);
  moveDriveVoltage(0);



  Count.remove();
  Indexing.remove();
}

void empty(){}
