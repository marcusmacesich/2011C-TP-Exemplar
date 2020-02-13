#include "main.h"

void resetSensors(){
   Left_Front_Drive.tare_position();
   Left_Back_Drive.tare_position();
   Right_Front_Drive.tare_position();
   Right_Back_Drive.tare_position();
   Ball_Intake.tare_position();
   Catapult.tare_position();
   Left_Arm.tare_position();
   Right_Arm.tare_position();
}

void resetDriveTrain(){
    Left_Front_Drive.tare_position();
    Left_Back_Drive.tare_position();
    Right_Front_Drive.tare_position();
    Right_Back_Drive.tare_position();
}

void driveTrainBreaking(){
   Left_Front_Drive.set_brake_mode(MOTOR_BRAKE_BRAKE);
   Left_Back_Drive.set_brake_mode(MOTOR_BRAKE_BRAKE);
   Right_Front_Drive.set_brake_mode(MOTOR_BRAKE_BRAKE);
   Right_Back_Drive.set_brake_mode(MOTOR_BRAKE_BRAKE);
}

void driveTrainCoasting(){
   Left_Front_Drive.set_brake_mode(MOTOR_BRAKE_COAST);
   Left_Back_Drive.set_brake_mode(MOTOR_BRAKE_COAST);
   Right_Front_Drive.set_brake_mode(MOTOR_BRAKE_COAST);
   Right_Back_Drive.set_brake_mode(MOTOR_BRAKE_COAST);
}

void driveTrainHolding(){
   Left_Front_Drive.set_brake_mode(MOTOR_BRAKE_HOLD);
   Left_Back_Drive.set_brake_mode(MOTOR_BRAKE_HOLD);
   Right_Front_Drive.set_brake_mode(MOTOR_BRAKE_HOLD);
   Right_Back_Drive.set_brake_mode(MOTOR_BRAKE_HOLD);
}

void moveLeftDriveTrain(int velocity){
   Left_Front_Drive.move_velocity(velocity);
   Left_Back_Drive.move_velocity(velocity);
}

void moveRightDriveTrain(int velocity){
  Right_Front_Drive.move_velocity(velocity);
  Right_Back_Drive.move_velocity(velocity);
}

void moveDriveTrain(int velocity){
  Left_Front_Drive.move_velocity(velocity);
  Left_Back_Drive.move_velocity(velocity);
  Right_Front_Drive.move_velocity(velocity);
  Right_Back_Drive.move_velocity(velocity);
}

//360 for one full wheel rotation
//Reading from 4 encoder values each side averaged
//360 * 2
//One full rotation of 4in omni = 12.566 in traveled (circumference)
//inchToTicks = 360*2/12.566 = 57.296
int inchToTicks (float inch){
  int ticks;
  ticks = inch * 57.296;
  return ticks;
}


//One full 360 degree turn of the robot =
//1400 degree counting in the motor
int degreesToTicks(float degrees){
  float ticksPerTurn = 5.8;
  int ticks = degrees * ticksPerTurn;
  return ticks;


}

void driveForwardsPID(float target,int timeOut,int maxVelocity){
  float Kp = 0.4;
  float Ki = 0.000001;
  float Kd = 0.3;
  int error;
  int proportion;
  int intergralRaw;
  float intergralActiveZone = inchToTicks(3);
  float intergral;
  int lastError;
  int derivative;
  int finalVelocity;
  float intergralVelocityLimit = 5/Ki;
  float Kp_C = 0.01;
  int error_drift;
  float proportion_drift;
  resetDriveTrain();
  long endTime;
  endTime = pros::millis() + timeOut;
  while (pros::millis() < endTime){
    error = inchToTicks(target) - (((Left_Front_Drive.get_position() + Left_Back_Drive.get_position())/2) + ((Right_Front_Drive.get_position() + Right_Back_Drive.get_position())/2));

    proportion = Kp * error;
    if (abs(error) < intergralActiveZone && error != 0){
      intergralRaw = intergralRaw + error;
    }
    else {
      intergralRaw = 0;
    }
    if (intergralRaw > intergralVelocityLimit){
      intergralRaw = intergralVelocityLimit;
    }
    if (intergralRaw < intergralVelocityLimit){
      intergralRaw = -intergralVelocityLimit;
    }
    intergral = Ki * intergralRaw;
    derivative = Kd * (error - lastError);
    lastError = error;
    if (error == 0){
      derivative = 0;
    }
    finalVelocity = proportion + intergral + derivative;
    if (finalVelocity > maxVelocity){
      finalVelocity = maxVelocity;
    }
    else if (finalVelocity < -maxVelocity){
      finalVelocity = -maxVelocity;
    }
    error_drift  = gyro.get_value();//(((Left_Front_Drive.get_position() + Left_Back_Drive.get_position())/2) - ((Right_Front_Drive.get_position() + Right_Back_Drive.get_position())/2));
    proportion_drift = Kp_C * error_drift;
    moveLeftDriveTrain(finalVelocity - proportion_drift);
    moveRightDriveTrain(finalVelocity + proportion_drift);

    if (pros::c::adi_analog_read('C') > 2890){
      Catapult.move_velocity(100);
    }
    else {
			Catapult.set_brake_mode(MOTOR_BRAKE_BRAKE);
			Catapult.move_velocity(0);
		}

    pros::delay(20);
  }
  driveTrainBreaking();
  moveDriveTrain(0);
  gyro.reset();
}

void driveBackwardsPID(float target,int timeOut,int maxVelocity){
  float Kp = 0.4;
  float Ki = 0.000001;
  float Kd = 0.3;
  int error;
  int proportion;
  int intergralRaw;
  float intergralActiveZone = inchToTicks(3);
  float intergral;
  int lastError;
  int derivative;
  int finalVelocity;
  float intergralVelocityLimit = 5/Ki;
  float Kp_C = 0.05;
  int error_drift = gyro.get_value();
  float proportion_drift;
  resetDriveTrain();
  long endTime;
  endTime = pros::millis() + timeOut;
  while (pros::millis() < endTime){
    error = inchToTicks(target) + (((Left_Front_Drive.get_position() + Left_Back_Drive.get_position())/2) + ((Right_Front_Drive.get_position() + Right_Back_Drive.get_position())/2));
    proportion = Kp * error;

    if (abs(error) < intergralActiveZone && error != 0){
      intergralRaw = intergralRaw + error;
    }
    else {
      intergralRaw = 0;
    }
    if (intergralRaw > intergralVelocityLimit){
      intergralRaw = intergralVelocityLimit;
    }
    if (intergralRaw < intergralVelocityLimit){
      intergralRaw = -intergralVelocityLimit;
    }
    intergral = Ki * intergralRaw;
    derivative = Kd * (error - lastError);
    lastError = error;
    if (error == 0){
      derivative = 0;
    }
    finalVelocity = proportion + intergral + derivative;
    if (finalVelocity > maxVelocity){
      finalVelocity = maxVelocity;
    }
    else if (finalVelocity < -maxVelocity){
      finalVelocity = -maxVelocity;
    }
    error_drift  = gyro.get_value(); //(((Left_Front_Drive.get_position() + Left_Back_Drive.get_position())/2) - ((Right_Front_Drive.get_position() + Right_Back_Drive.get_position())/2));
    proportion_drift = Kp_C * error_drift;
    moveLeftDriveTrain(-finalVelocity - proportion_drift);
    moveRightDriveTrain(-finalVelocity + proportion_drift);

    if (pros::c::adi_analog_read('C') > 2890){
      Catapult.move_velocity(100);
    }
    else {
			Catapult.set_brake_mode(MOTOR_BRAKE_BRAKE);
			Catapult.move_velocity(0);
		}

    pros::delay(20);
  }
  driveTrainBreaking();
  moveDriveTrain(0);
  gyro.reset();
}

void turnLeftPID(float target,int timeOut,int maxVelocity){
  float Kp = 0.2;
  float Ki = 0.1;
  float Kd = 0.4;
  int error;
  int proportion;
  int intergralRaw;
  float intergralActiveZone = inchToTicks(3);
  float intergral;
  int lastError;
  int derivative;
  int finalVelocity;
  float Kp_C = 0.1;
  int error_drift;
  float proportion_drift;
  float intergralVelocityLimit = 5/Ki;
  resetDriveTrain();
  long endTime;
  endTime = pros::millis() + timeOut;
  while (pros::millis() < endTime){
    error = fabs(target * 10) - fabs(gyro.get_value());
    proportion = Kp * error;

    if (fabs(error) < intergralActiveZone && fabs(error) != 0){
         intergralRaw = intergralRaw + error;
    }
    else {
      intergralRaw = 0;
    }
    if (intergralRaw > intergralVelocityLimit){
      intergralRaw = intergralVelocityLimit;
    }
    if (intergralRaw < intergralVelocityLimit){
      intergralRaw = -intergralVelocityLimit;
    }
    intergral = Ki * intergralRaw;
    derivative = Kd * (error - lastError);
    lastError = error;
    if (error == 0){
      derivative = 0;
    }
    finalVelocity = proportion + intergral + derivative;
    if (finalVelocity > maxVelocity){
      finalVelocity = maxVelocity;
    }
    else if (finalVelocity < -maxVelocity){
      finalVelocity = -maxVelocity;
    }
    error_drift  = gyro.get_value();//(((Left_Front_Drive.get_position() + Left_Back_Drive.get_position())/2) - ((Right_Front_Drive.get_position() + Right_Back_Drive.get_position())/2));
    proportion_drift = Kp_C * error_drift;
    moveLeftDriveTrain(-finalVelocity - proportion_drift);
    moveRightDriveTrain(finalVelocity + proportion_drift);

    if (pros::c::adi_analog_read('C') > 2890){
      Catapult.move_velocity(100);
    }
    else {
			Catapult.set_brake_mode(MOTOR_BRAKE_BRAKE);
			Catapult.move_velocity(0);
		}

    pros::delay(20);
  }
  driveTrainBreaking();
  moveDriveTrain(0);
  gyro.reset();
}

void turnRightPID(float target,int timeOut,int maxVelocity){
  float Kp = 0.2;
  float Ki = 0.1;
  float Kd = 0.4;
  int error;
  int proportion;
  int intergralRaw;
  float intergralActiveZone = inchToTicks(3);
  float intergral;
  int lastError;
  int derivative;
  int finalVelocity;
  float Kp_C = 0.1;
  int error_drift;
  float proportion_drift;
  float intergralVelocityLimit = 5/Ki;
  resetDriveTrain();
  long endTime;
  endTime = pros::millis() + timeOut;
  while (pros::millis() < endTime){
    error = fabs(target * 10) - fabs(gyro.get_value());
    proportion = Kp * error;

    if (fabs(error) < intergralActiveZone && fabs(error) != 0){
         intergralRaw = intergralRaw + error;
    }
    else {
      intergralRaw = 0;
    }
    if (intergralRaw > intergralVelocityLimit){
      intergralRaw = intergralVelocityLimit;
    }
    if (intergralRaw < intergralVelocityLimit){
      intergralRaw = -intergralVelocityLimit;
    }
    intergral = Ki * intergralRaw;
    derivative = Kd * (error - lastError);
    lastError = error;
    if (error == 0){
      derivative = 0;
    }
    finalVelocity = proportion + intergral + derivative;
    if (finalVelocity > maxVelocity){
      finalVelocity = maxVelocity;
    }
    else if (finalVelocity < -maxVelocity){
      finalVelocity = -maxVelocity;
    }
    error_drift  = gyro.get_value();//(((Left_Front_Drive.get_position() + Left_Back_Drive.get_position())/2) - ((Right_Front_Drive.get_position() + Right_Back_Drive.get_position())/2));
    proportion_drift = Kp_C * error_drift;
    moveLeftDriveTrain(finalVelocity - proportion_drift);
    moveRightDriveTrain(-finalVelocity + proportion_drift);

    if (pros::c::adi_analog_read('C') > 2890){
      Catapult.move_velocity(100);
    }
    else {
			Catapult.set_brake_mode(MOTOR_BRAKE_BRAKE);
			Catapult.move_velocity(0);
		}

    pros::delay(20);
  }
  driveTrainBreaking();
  moveDriveTrain(0);
  gyro.reset();
}

void RollUptake(int velocity){
  Ball_Intake.tare_position();
  Ball_Intake.set_brake_mode(MOTOR_BRAKE_COAST);
  Ball_Intake.move_velocity(velocity);
}

void CatapultShoot(int sensor,int velocity){
  Catapult.move_velocity(velocity);
  pros::delay(sensor);
  if (pros::c::adi_analog_read('C') > 2830){
    Catapult.move_velocity(100);
  }
  else {
    Catapult.set_brake_mode(MOTOR_BRAKE_BRAKE);
    Catapult.move_velocity(0);
  }
  Catapult.move_velocity(0);
}

void prepCatapult(){
  while (Catapult_Potentiometer.get_value() < 100){
    Catapult.move_velocity(200);
  }
  Catapult.set_brake_mode(MOTOR_BRAKE_BRAKE);
  Catapult.move_velocity(0);
}

void armUp(int sensor,int velocity){
  Left_Arm.set_brake_mode(MOTOR_BRAKE_BRAKE);
  Right_Arm.set_brake_mode(MOTOR_BRAKE_BRAKE);
  Left_Arm.move_velocity(velocity);
  Right_Arm.move_velocity(velocity);
  pros::delay(sensor);
  Left_Arm.move_velocity(0);
  Right_Arm.move_velocity(0);
}

void armDown(int sensor,int velocity){
  Left_Arm.set_brake_mode(MOTOR_BRAKE_BRAKE);
  Right_Arm.set_brake_mode(MOTOR_BRAKE_BRAKE);
  Left_Arm.move_velocity(-velocity);
  Right_Arm.move_velocity(-velocity);
  pros::delay(sensor);
  Left_Arm.move_velocity(0);
  Right_Arm.move_velocity(0);
}


void autonomous(){
gyro.reset();
pros::delay(250);
// Perfect 90 both sides turn----PID(135,10000,150);
//Blue Auton -- Pot Screw Left

  if (pros::c::adi_analog_read('H') < 1000)
{
  pros::lcd::print(1, "%d",pros::c::adi_analog_read('H')); //Display pot value
  pros::lcd::set_text(3, "Blue Auton"); //Display running auton
  // bluePart();
}
//Skills Auton -- Pot Screw Centered
  else if(pros::c::adi_analog_read('H') >= 2000 && pros::c::adi_analog_read('H') <= 3000)
{
  pros::lcd::print(1, "%d",pros::c::adi_analog_read('H')); //Display pot value
  pros::lcd::set_text(3, "Skills Auton"); //Display running auton
  // skillsAuton();
}
//Red Auton -- Pot Screw Right
  else if(pros::c::adi_analog_read('H') > 3000)
{
  pros::lcd::print(1, "%d",pros::c::adi_analog_read('H')); //Display pot value
  pros::lcd::set_text(3, "Red Auton"); //Display running auton
  // redPark();
}
  }
