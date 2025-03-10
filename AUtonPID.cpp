#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
pros::Motor babybrown(5);
pros::MotorGroup leftweels({-1,	-2,-3},pros::MotorGearset::blue);
pros::MotorGroup rightweels({13,12,18},pros::MotorGearset::blue);
pros::Motor convey(14);
pros::adi::Pneumatics clamp1('a',true);
pros::adi::Pneumatics clamp2('b',true);
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor intake(11);
lemlib::Drivetrain drivetrain(&leftweels,
								&rightweels,
								11.5,
								lemlib::Omniwheel::NEW_275,
								600,
								2




	);
pros::Rotation rot1(8);
pros::Rotation rot2(9);
lemlib::TrackingWheel track1(&rot1, lemlib::Omniwheel::NEW_2,-4.75);
lemlib::TrackingWheel track2(&rot2,lemlib::Omniwheel::NEW_2,4.5);
lemlib::OdomSensors sensors(nullptr,nullptr,nullptr,nullptr,nullptr);
lemlib::ControllerSettings lateral_movement(17, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              30 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_movement(8, // proportional gain (kP)
                                              5, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                                 1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_movement, // lateral PID settings
                        angular_movement, // angular PID settings
                        sensors // odometry sensors
);


void initialize(){
	chassis.calibrate();
}
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */ 


void allianceAndSingularRingright(){
	chassis.moveToPoint(0,12,2000);
	chassis.turnToHeading(90,2000);
	chassis.moveToPoint(5, 12, 2000);
	babybrown.move(-100);
	pros::delay(200);
	babybrown.brake();
	babybrown.move(100);
	pros::delay(200);
	babybrown.brake();
	chassis.moveToPoint(-48, -12, 0);
	chassis.turnToHeading(-90, 5000);
	clamp1.extend();
	clamp2.extend();
	chassis.turnToHeading(-180, 3000);
	intake.move(100);
	convey.move(100);
	
}
void allianceAndSingularRingrightey(){
	chassis.moveToPoint(0,12,2000);
	chassis.turnToHeading(-90,2000);
	chassis.moveToPoint(5, 12, 2000);
	babybrown.move(-100);  
	pros::delay(200);
	babybrown.brake();
	babybrown.move(100);
	pros::delay(200);
	babybrown.brake();
	chassis.moveToPoint(-48, -12, 0);
	chassis.turnToHeading(-90, 2000);
	clamp1.extend();
	clamp2.extend();
	chassis.turnToHeading(0, 2000);
	chassis.moveToPoint(-48, 12, 2000);
	intake.move(100);
	convey.move(100);
	
}
void skilz(){
	chassis.setPose(0,0,0);
	intake.move(100);
	convey.move(100);
	chassis.moveToPoint(0, 12, 2000);//moves up
	chassis.turnToHeading(90,5000 );//turns toward mogo1
	chassis.moveToPoint(-24,12, 2000);//moves backward to mogo1
	clamp1.extend();//CLAMPS
	clamp2.extend();
	chassis.turnToHeading(0, 4000);//turns to a ring
	chassis.moveToPoint(-24, 36,4000);//goes around to get four rings
	chassis.turnToHeading(-90, 2000);
	chassis.moveToPoint(-48, 36, 2000);
	chassis.turnToHeading(180, 1000);
	chassis.moveToPoint(-48, 0, 2000);//done getting four rings
	chassis.moveToPoint(-48, 12, 1200);//goes back
	chassis.turnToHeading(-90, 2000);//turns to fifth ring
	chassis.moveToPoint(-72, 12, 2000);//intakes fifth ringy
	chassis.moveToPoint(-48, 12, 1000);//goes back
	chassis.turnToHeading(45, 2000);//turns to cornerr
	chassis.moveToPoint(-72, 0, 2000);//goes into corner
	clamp1.retract();
	clamp2.retract();
	chassis.moveToPoint(0, 0, 2000);//goes back to beginning position for ease

	chassis.setPose(0,0,0);
	intake.move(100);
	convey.move(100);
	chassis.moveToPoint(0, 12, 2000);//moves ujp
	chassis.turnToHeading(-90,5000 );//turns to mogo
	chassis.moveToPoint(24,12, 2000);//goes back into mogo
	clamp1.extend();//clamps
	clamp2.extend();
	chassis.turnToHeading(0, 4000);//turns to the four rings 
	chassis.moveToPoint(24, 36,4000);//goes around in a circle for four rings
	chassis.turnToHeading(90, 60000);
	chassis.moveToPoint(48, 36, 2000);
	chassis.turnToHeading(180, 1000);
	chassis.moveToPoint(48, 0, 2000);//done going in circle for four rings
	chassis.moveToPoint(48, 12, 1200);//goes back
	chassis.turnToHeading(-90, 2000);//turns to fifth ring
	chassis.moveToPoint(72, 12, 2000);//intakes fifth ring
	chassis.moveToPoint(48, 12, 1000);//goes back
	chassis.turnToHeading(-45, 2000);//turns to corner
	chassis.moveToPoint(72, 0, 2000);//goes into corner

	chassis.moveToPoint(60, 12, 2000);//goes to line up with ring for lady brown
	chassis.turnToHeading(0, 2000);//turns to ring for lady brown
	babybrown.move_relative(30, 20);//moves lady brown up to be primed
	chassis.moveToPoint(60,60,2000 );//intakes ring for lady brown
	chassis.turnToHeading(90, 2000);//turns for wall stake
	babybrown.move(100);//moves lady brown for quarter second
	pros::delay(250);
	babybrown.brake();//stops lady brown
	chassis.setPose(0,0,0);
	chassis.moveToPoint(0, -12, 3000);//moves back a little
	chassis.turnToHeading(22.5, 2000);//turns to blue goal
	chassis.moveToPoint(60, -36, 3000);//moves it into the goal
	clamp1.extend();//clamps
	clamp2.extend();
	chassis.turnToHeading(180, 2000);//turns to the corner
	chassis.moveToPoint(60, 0, 2000);//puts into corner
	clamp1.retract();//unclamps
	clamp2.retract();
	chassis.moveToPoint(60,-156,3000);//puts other blue goal into corner




}
void autonomous(){
	skilz();
}



/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::MotorGroup leftweels({-1,	-2,-3},pros::MotorGearset::blue);
	pros::MotorGroup rightweels({13,12,18},pros::MotorGearset::blue);
	pros::Motor convey(14);
	pros::adi::Pneumatics clamp1('a',false);
	pros::adi::Pneumatics clamp2('b',false);
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor intake(11);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	convey.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	bool sigma=true;
	pros::Optical racistmech(4);
	pros::Motor babybrown(5);
	babybrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	bool conveyormoving=true;
	leftweels.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightweels.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	
	while (true) {
		int race=racistmech.get_hue();
		
		leftweels.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		rightweels.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake.move(-1000);
			convey.move(1000);
			conveyormoving=false;
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			intake.move(1000);
			convey.move(-1000);
			conveyormoving=true;
		}
		else{
			intake.brake();
			convey.brake();
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && ((clamp2.is_extended()==false)&&(clamp1.is_extended()==false))){
			clamp1.extend();
			clamp2.extend();
			pros::delay(250);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && ((clamp2.is_extended()==true)&&(clamp1.is_extended()==true))){
			clamp1.retract();
			clamp2.retract();
			pros::delay(250);
		}
		//anti blue
		if ((race<=180&&race>=300)&&(conveyormoving==true)){
			convey.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			pros::delay(0);
			convey.brake();
			pros::delay(250);
			convey.brake();
			

			convey.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			babybrown.move(50);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			babybrown.move(-50);
		}
		else{
			babybrown.brake();
		}




	pros::delay(20);
}
}
