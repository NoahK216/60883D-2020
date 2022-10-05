#pragma once
#include "include.cpp"
#include "auton_functions.cpp"
#define L_DISTANCE_IN 5.565
#define R_DISTANCE_IN 5.565
#define S_DISTANCE_IN 4.5

struct _pos{
	float a = 0;
	float y = 0;
	float x = 0;
	int leftLst = 0;
	int backLst = 0;
	int rightLst = 0;
}position; //Position of the robot

//Define other values that can be used in other functions
float globalA;
float globalY;
float globalX;

void trackPosition_fn(void* param){
	_pos position; //Allows the void to read the external position values
	float fakeBackEncoder;
  while(true){
		std::uint32_t startTime = pros::millis();

    float L = (motorAvgLeft() - position.leftLst) / 48.972; //The amount the left tracking wheel moved in inches
  	float R = (motorAvgRight() - position.rightLst) / 48.972; //The amount the right tracking wheel moved in inches
  	float S = (fakeBackEncoder - position.backLst) / 41.671; //The amount the tracking wheel moved in inches

    //Update the last values
    position.leftLst = motorAvgLeft();
    position.rightLst = motorAvgRight();
    position.backLst = fakeBackEncoder;

    float h; //The hypotenuse of the triangle formed by the middle of the robot on the starting position and ending position and the middle of the circle it travels around
  	float i; //Half on the angle that I've traveled
  	float h2; //The same as h but using the back instead of the side wheels
  	float a = (L - R) / (L_DISTANCE_IN + R_DISTANCE_IN); // The angle that I've traveled

    if (a){
      i = (a/2.0);
  		float sinI = sin(i);
  		h = (((R / a) + R_DISTANCE_IN) * sinI) * 2.0;
  		h2 = (((S / a) + S_DISTANCE_IN) * sinI) * 2.0;
  	}
  	else{
  		h = R; //Sets relative distance traveled to the right wheel as since their deltas are equal they traveled the same distance
  		i = 0; //Half of angle traveled will always be zero if the angle traveled is zero
  		h2 = S; //Back wheel movement is simply the difference in the back wheel if the angle didnt change
  	}

    float p = i + position.a; //The global ending angle of the robot
    float cosP = cos(p); //Cleans up final variable storage math
    float sinP = sin(p); //Cleans up final variable storage math

    //Update the global position
    position.y += h * cosP;
    position.x += h * sinP;

    //Update global position with back wheeldata
    position.y += h2 * -sinP; //-sin(x) = sin(-x)
    position.x += h2 * cosP; //cos(x) = cos(-x)

    //Update the global rotation value according to relative value calculated earlier
    position.a += radToDeg(a);


		//master.print(0, 1, "%.2f, %.2f, %.2f        ", position.x, position.y, position.a);

    pros::Task::delay_until(&startTime, 10);
  }
}
