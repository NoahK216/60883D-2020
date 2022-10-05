#pragma once
#include "main.h"
#include "gif-pros\gifclass.hpp"

//Global Variable Declaration
//int lineSensorDif = 2700;
//More sensitive = higher
float lineSensorSense = 2750;//2800, 2725,
float bottomSensorDif = 0;
int auton = 4;
int autonumber = auton;
bool autoTouch = false;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor frontright(19);
pros::Motor backright(20);

pros::Motor frontleft(16, true);
pros::Motor backleft(5, true);

pros::Motor leftintake(3);
pros::Motor rightintake(10, true);

pros::Motor mainroller1(4,true);
pros::Motor mainroller2(8);

pros::Imu imu(17);

pros::ADIEncoder backEncoder();

pros::ADIAnalogIn bottomTrackerLeft (8);
pros::ADIAnalogIn bottomTrackerRight (5);
pros::ADIAnalogIn indexTracker (2);
pros::ADIAnalogIn topTrackerLeft (1);
pros::ADIAnalogIn topTrackerRight (4);
