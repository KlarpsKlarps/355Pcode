#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
pros::MotorGroup leftweels({-1,	-2,-3},pros::MotorGearset::blue);
pros::MotorGroup rightweels({13,12,18},pros::MotorGearset::blue);
pros::Motor convey(6);
pros::adi::Pneumatics clamp1('g',false);
pros::adi::Pneumatics clamp2('h',false);
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor intake(11);
lemlib::Drivetrain divetain(&leftweels,
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
lemlib::OdomSensors sents(&track1,&track2,nullptr,nullptr,nullptr);
lemlib::ControllerSettings latissimus_dorsi(17, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              127 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings abdominals(6.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                                 1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
lemlib::Chassis chassis(divetain, // drivetrain settings
                        latissimus_dorsi, // lateral PID settings
                        abdominals, // angular PID settings
                        sents // odometry sensors
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
void autonomous() {
	chassis.setPose(0,0,0);
	chassis.moveToPoint(0,48, 5000);
	
	
	

	

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
	pros::Motor convey(6);
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
	
	while (true) {
		int race=racistmech.get_hue();
		
		leftweels.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		rightweels.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake.move(-1000);
			convey.move(1000);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			intake.move(1000);
			convey.move(-1000);
		}
		else{
			intake.brake();
			convey.brake();
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && ((clamp2.is_extended()==false)&&(clamp1.is_extended()==false))){
			clamp1.extend();
			clamp2.extend();
			pros::delay(700);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && ((clamp2.is_extended()==true)&&(clamp1.is_extended()==true))){
			clamp1.retract();
			clamp2.retract();
			pros::delay(700);
		}
		//anti blue
		if ((race==190)||(race==191)||(race==192)||(race==193)||(race==194)||(race==195)||(race==196)||(race==197)||(race==198)||(race==199)||(race==200)||(race==201)||(race==202)||(race==189)||(race==188)||(race==203)){
			
			convey.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			pros::delay(80);
			convey.brake();
			convey.move(100);
			pros::delay(100);
			convey.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			babybrown.move(100);
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
