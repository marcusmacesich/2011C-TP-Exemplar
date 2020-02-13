#include "main.h"

/*
Define motors and sensor in this document
Don't forget to add the extern in the main.h file
*/

//prosv5 upload --slot [number]

#define Left_Front_Port 4
#define Left_Back_Port 5
#define Right_Front_Port 2
#define Right_Back_Port 3
#define Ball_Intake_Port 7
#define Catapult_Port 11
#define Left_Arm_Port 9
#define Right_Arm_Port 8
#define Catapult_Potentiometer_Port 'C'
#define Drive_Gyro_Port 'E'
#define Auton_Pot_Port 'H'

pros::Motor Left_Front_Drive (Left_Front_Port,MOTOR_GEARSET_18,false,MOTOR_ENCODER_DEGREES);
pros::Motor Left_Back_Drive (Left_Back_Port,MOTOR_GEARSET_18,false,MOTOR_ENCODER_DEGREES);
pros::Motor Right_Front_Drive (Right_Front_Port,MOTOR_GEARSET_18,true,MOTOR_ENCODER_DEGREES);
pros::Motor Right_Back_Drive (Right_Back_Port,MOTOR_GEARSET_18,true,MOTOR_ENCODER_DEGREES);
pros::Motor Ball_Intake (Ball_Intake_Port,MOTOR_GEARSET_18,MOTOR_ENCODER_DEGREES);
pros::Motor Catapult (Catapult_Port,MOTOR_GEARSET_36,true,MOTOR_ENCODER_DEGREES);
pros::Motor Left_Arm (Left_Arm_Port,MOTOR_GEARSET_18,false,MOTOR_ENCODER_DEGREES);
pros::Motor Right_Arm (Right_Arm_Port,MOTOR_GEARSET_18,true,MOTOR_ENCODER_DEGREES);
pros::ADIAnalogIn Catapult_Potentiometer (Catapult_Potentiometer_Port);
pros::Controller master (CONTROLLER_MASTER);
pros::ADIGyro gyro (Drive_Gyro_Port);
pros::ADIAnalogIn Auton_Pot (Auton_Pot_Port);
