#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/motors.h"
#include "pros/optical.hpp"
#include <string>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "arhun");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
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
void autonomous() {}

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
	pros::MotorGroup rightweels({11,12,18},pros::MotorGearset::blue);
	pros::Motor convey(6);
	pros::adi::Pneumatics clamp1('g',false);
	pros::adi::Pneumatics clamp2('h',false);
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor intake(19);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	convey.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	bool sigma=true;
	pros::Optical racistmech(15);
	
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
		//anti red
		if ((race==7)||(race==8)||(race==9)||(race==10)||(race==6)||(race==5)||(race==4)||(race==3)||(race==2)||(race==1)||(race==355)||(race==356)||(race==357)||(race==358)||(race==359)||(race==360)){
			
			convey.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			pros::delay(200);
			convey.brake();
			convey.move(100);
			pros::delay(100);
			convey.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		//anti blue
		if ((race==190)||(race==191)||(race==192)||(race==193)||(race==194)||(race==195)||(race==196)||(race==197)||(race==198)||(race==199)||(race==200)||(race==201)||(race==202)||(race==189)||(race==188)||(race==203)){
			
			convey.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			pros::delay(200);
			convey.brake();
			convey.move(100);
			pros::delay(100);
			convey.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}


	pros::delay(20);
}
}
