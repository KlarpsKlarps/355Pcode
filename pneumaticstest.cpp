//comp code

#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

pros::MotorGroup left_wheels ({-1,-9});
pros::MotorGroup right_wheels ({2,10});
pros::Motor mogo(5,pros::v5::MotorGears::red) ;

/**
 * A callback function for LLEMU's center button.
 *            `
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
	pros::lcd::set_text(1, "im in the thick of it everybody knows");
	pros::lcd::set_text(2, "They know me where it snows, I skid in and they froze");
	
	std::cout << "Battery Level:" << pros::battery::get_capacity();
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
void autonomous() {
  pros::adi::DigitalOut mogyPISTY (DIGITAL_A);
  pros::MotorGroup left_wheels ({-18,-17});
  pros::MotorGroup right_wheels ({12,10});
  pros::Controller master (CONTROLLER_MASTER);
  pros::Controller controller (pros::E_CONTROLLER_MASTER);
  pros::Motor upperintake (20);
  pros::Motor lowerintake(19);
  pros::Motor mogo(1);
  //start autonomous
  mogo.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  left_wheels.move(-50);
  right_wheels.move(-50);
  pros::delay(1000);//wait .7 secs

  left_wheels.brake();//brake time
  right_wheels.brake();
  mogo.move(100);
  pros::delay(1000);
  mogo.brake();

  left_wheels.move(50);//turn left
  pros::delay(300);
  left_wheels.brake();
  //intake random ring
  left_wheels.move(100);
  right_wheels.move(100);
  upperintake.move(60);
  lowerintake.move(-70);
  pros::delay(1000);
  left_wheels.brake();
  right_wheels.brake();



  
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
 // Be sure to reverse this!

void opcontrol() {/*basically, this code under just conficures the motors for driver control.*/
  pros::adi::DigitalOut mogyPISTY (DIGITAL_A);
  pros::MotorGroup left_wheels ({-18,-17});
  pros::MotorGroup right_wheels ({12,10});
  pros::Controller master (CONTROLLER_MASTER);
  pros::Controller controller (pros::E_CONTROLLER_MASTER);
  pros::Motor upperintake (20);
  pros::Motor lowerintake(19);
  pros::Motor mogo(1);
  left_wheels.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  right_wheels.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  mogo.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  

  while (true) {
	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
		mogo.set_zero_position(0);
	}


	
    mogo.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); 
    left_wheels.move(master.get_analog(ANALOG_LEFT_Y));//tanks drives left
    right_wheels.move(master.get_analog(ANALOG_RIGHT_Y));//tank drive righty


	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){ //RUNS EPIC INTAKE CODE FORWARDS
		upperintake.move(-100);
		lowerintake.move(100);
  	}
	else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){  //RUNS IT BAKWEdS
		lowerintake.move(-100);
		upperintake.move(100);
	
	}
	else{
		upperintake.brake();
		lowerintake.brake();
	}
	
	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
		mogyPISTY.set_value(true);
	}
	else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
		mogyPISTY.set_value(false);
	}else{
		mogo.brake();
	}

	
    pros::delay(20); //i think we ned this
}}
	
