#include "include.cpp"
#include "autons.cpp"
#include "drive_functions.cpp"
#include "movement.cpp"

lv_obj_t * statsLabel;
void initialize() {
	statsLabel = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(statsLabel, "IMU Rotation:\nIMU Heading:\n"); //sets label text
  lv_obj_align(statsLabel, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 0); //set the position to center
}

void disabled() {}

void competition_initialize() {}

void autonomous(){
	switch(auton%autonumber){
		case 0: homerow(); 					break;
		
		case 1: skills();         	break;
		case 2: middle_row();		  	break;
		case 3: empty(); 						break;
	}
}

void opcontrol(){
	//pros::Task BallCount (indexCount_fn, (void*)"PROS", "BalCount");
	while (true) {
		//Assign variables to joystick values for ease of use
		int leftX = master.get_analog(ANALOG_LEFT_X);
		int leftY = master.get_analog(ANALOG_LEFT_Y);
		int rightX = master.get_analog(ANALOG_RIGHT_X);
		int rightY = master.get_analog(ANALOG_RIGHT_Y);


		//Change auton value
		if(master.get_digital_new_press(DIGITAL_LEFT)){auton--;}
		else if(master.get_digital_new_press(DIGITAL_RIGHT)){auton++;}


		//Display current autonomous on the controller
		if(autoTouch){switch(auton%autonumber){
			case 0: master.print(2, 0, "Homerow                     "); break;
			case 1: master.print(2, 0, "Skills                      "); break;
			case 2: master.print(2, 0, "Middle Row                  "); break;
			case 3: master.print(2, 0, "No Auton                    "); break;
		}}
		else{master.print(2, 0, "%.2f                   ", imu.get_heading());}


		//Set drive motor speeds
		frontright = leftY - rightX;
		backright = leftY - rightX;
		frontleft = leftY + rightX;
		backleft = leftY + rightX;


		//Confine roller and intake speed assignment to a function to keep opcontrol cleaner
		intakeRollerControl();

		//Run the currently selected autonomous when B is pressed
		if(master.get_digital_new_press(DIGITAL_B)){
			skills();
		}

		//Calibrates the IMU on a button press
		if(master.get_digital_new_press(DIGITAL_UP)){
			imu.reset();
			while (imu.is_calibrating()) {master.print(2, 0, "Calibrating IMU        "); pros::delay(20);}
			float start = pros::millis();
			while(pros::millis() < start + 2000){master.print(2, 0, "IMU Calibrated         ");pros::delay(20);}
		}


		//Changes controller display to toggle autoTouch on button press
		if(master.get_digital_new_press(DIGITAL_DOWN)){autoTouch=!autoTouch;}

		//Intake toggle for skills
		if(master.get_digital_new_press(DIGITAL_Y)){intakeToggle=!intakeToggle;}

		//Lineup for middle row
		if(master.get_digital_new_press(DIGITAL_X)){drivePID("backward", 25, 3, 50);}


		char buffer[150];
		sprintf(buffer,
			"IMU Rotation: %.2f\nIMU Heading: %.2f",
			imu.get_rotation(), imu.get_heading()
		);
		lv_label_set_text(statsLabel, buffer);


		pros::delay(20);
	}
}
