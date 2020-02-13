#include "main.h"
void poleDistanceMacro(){

	Right_Front_Drive.tare_position();
	Right_Back_Drive.tare_position();
	Left_Front_Drive.tare_position();
	Left_Back_Drive.tare_position(); //Reset Drive Encoders

	Right_Front_Drive.move_absolute(300, 120);
	Right_Back_Drive.move_absolute(300, 120);
	Left_Front_Drive.move_absolute(300, 120);
	Left_Back_Drive.move_absolute(300, 120); //Move Drive Train Forward X Ticks

while (!((Right_Front_Drive.get_position() < 310) && (Right_Front_Drive.get_position() > 290))) {
	// Run Until Within Tolerance of 20 ticks
	pros::delay(20);
}
	}

bool CatapultStop = false;

void opcontrol() {
	while (true){
		Left_Front_Drive.set_brake_mode(MOTOR_BRAKE_BRAKE);
		Left_Back_Drive.set_brake_mode(MOTOR_BRAKE_BRAKE);
		Right_Front_Drive.set_brake_mode(MOTOR_BRAKE_BRAKE);
		Right_Back_Drive.set_brake_mode(MOTOR_BRAKE_BRAKE);



		//--------------------------------------------// Pole Distance Macro

		if (master.get_digital(DIGITAL_A)) {
			poleDistanceMacro();
		}


		//--------------------------------------------// Drive Train

			Left_Front_Drive.move(master.get_analog(ANALOG_LEFT_Y));
			Left_Back_Drive.move(master.get_analog(ANALOG_LEFT_Y));
			Right_Front_Drive.move(master.get_analog(ANALOG_RIGHT_Y));
			Right_Back_Drive.move(master.get_analog(ANALOG_RIGHT_Y));
			Left_Front_Drive.set_brake_mode(MOTOR_BRAKE_BRAKE);
			Left_Back_Drive.set_brake_mode(MOTOR_BRAKE_BRAKE);
			Right_Front_Drive.set_brake_mode(MOTOR_BRAKE_BRAKE);
			Right_Back_Drive.set_brake_mode(MOTOR_BRAKE_BRAKE);


		//--------------------------------------------// Ball Intake

		if (master.get_digital(DIGITAL_L1) && pros::c::adi_analog_read('C') < 2890){
			Ball_Intake.move_velocity(200);
		}
		else if (master.get_digital(DIGITAL_L2)){
			Ball_Intake.move_velocity(-200);
		}
		else {
			Ball_Intake.set_brake_mode(MOTOR_BRAKE_BRAKE);
			Ball_Intake.move_velocity(0);
		}

		//--------------------------------------------// Catapult

  pros::lcd::print(1, "%d\n",pros::c::adi_analog_read('H'));
		if (pros::c::adi_analog_read('C') > 2890){
			Catapult.move_velocity(100);
		}
		else if (master.get_digital(DIGITAL_DOWN)){
			Catapult.move_velocity(100);
		}
		else {
			Catapult.set_brake_mode(MOTOR_BRAKE_BRAKE);
			Catapult.move_velocity(0);
		}

		//--------------------------------------------// Cap Flipper

		if (master.get_digital(DIGITAL_R1)){
			Left_Arm.move_velocity(200);
			Right_Arm.move_velocity(200);
		}
		else if (master.get_digital(DIGITAL_R2)){
			Left_Arm.move_velocity(-200);
			Right_Arm.move_velocity(-200);
		}
		else {
			Left_Arm.set_brake_mode(MOTOR_BRAKE_BRAKE);
			Right_Arm.set_brake_mode(MOTOR_BRAKE_BRAKE);
			Left_Arm.move_velocity(0);
			Right_Arm.move_velocity(0);
		}
		pros::delay(20);  //Cortex Boy Needs a Break
	}
}
