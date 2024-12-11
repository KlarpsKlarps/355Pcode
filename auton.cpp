//comp code

#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

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
	pros::lcd::set_text(1, "auto eeet");
	std::cout << "Battery Level:" << pros::battery::get_capacity();
	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
 
void disabled() {

  pros::Controller rumby (CONTROLLER_MASTER);
  pros::Controller controller (pros::E_CONTROLLER_MASTER);

  rumby.rumble("- - - - - - - - -");
}


void competition_initialize() {}


void autonomous() {
  pros::adi::Pneumatics mogyPISTY('A',false);
  pros::adi::Pneumatics doinker('B',false);
  pros::Motor pish(3);
  pros::MotorGroup left_wheels ({-1,-10});
  pros::MotorGroup right_wheels ({2,11});
  pros::MotorGroup drivetrain({-1,-10,2,11});
  pros::Controller master (CONTROLLER_MASTER);
  pros::Controller controller (pros::E_CONTROLLER_MASTER);
  pros::Motor upperintake (12);
  pros::Motor lowerintake(9);
  pros::MotorGroup intake({12,9});
  bool moko=false;
  left_wheels.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  right_wheels.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  drivetrain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);


  drivetrain.move_relative(-1400, 60);
  pros::delay(1700);
  right_wheels.move_relative(450,60)&& left_wheels.move_relative(-450,60);
  pros::delay(1700);
  drivetrain.move_relative(-1000,40);
  pros::delay(1700);
  upperintake.move_relative(1000,100);


  
}
void opcontrol() {/*basically, this code under just conficures the motors for driver control.*/
  pros::adi::Pneumatics mogyPISTY('A',false);
  pros::adi::Pneumatics doinker('B',false);
  pros::Motor pish(3);
  pros::MotorGroup left_wheels ({-1,-10});
  pros::MotorGroup right_wheels ({2,11});
  pros::Controller master (CONTROLLER_MASTER);
  pros::Controller controller (pros::E_CONTROLLER_MASTER);
  pros::Motor upperintake (12);
  pros::Motor lowerintake(9);
  pros::MotorGroup intake({12,9});
  bool moko=false;
  left_wheels.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  right_wheels.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  
  
  

  while (true) {
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

if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
    mogyPISTY.extend();
  }
else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
    mogyPISTY.retract();
  }

	
  if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
    doinker.extend();
    master.clear();
    master.print(0,0,"doinker in");
  }
  else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
    doinker.retract();
    master.clear();
    master.print(0,0,"DOINKER OUT");
  }

  if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
    pish.move(60);
  }
  else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
    pish.move(-60);

  }
  else{
    pish.brake();
  }
	
    pros::delay(20); //i think we ned this
}}
	
