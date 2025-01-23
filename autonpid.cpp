#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
pros::Motor babybrown(5);
pros::Rotation babymove(20);
int primemodewall=30;
pros::MotorGroup leftweels({-1,	-2,-3},pros::MotorGearset::blue);
pros::MotorGroup rightweels({13,12,18},pros::MotorGearset::blue);

pros::Motor convey(14);
pros::adi::Pneumatics clamp1('a',true);
pros::adi::Pneumatics clamp2('b',true);
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor intake(11);
pros::Rotation leftrot(8);
pros::Rotation rightrot(-9);
lemlib::TrackingWheel v1(&leftrot, lemlib::Omniwheel::NEW_275, -7.5);
lemlib::TrackingWheel v2(&rightrot, lemlib::Omniwheel::NEW_275, 7.5);
lemlib::Drivetrain drivetrain(&leftweels,
								&rightweels,
								13.75,
								lemlib::Omniwheel::NEW_275,
								450,
								2




);
lemlib::OdomSensors sensors(&v1, // vertical tracking wheel 1, set to null
                            &v2, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr // inertial sensor
);

lemlib::ControllerSettings lateral_movement(20, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              1000000 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_movement(2.2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              0, // anti windup
                                                 0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              1000000 // maximum acceleration (slew)
);
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_movement, // lateral PID settings
                        angular_movement, // angular PID settings
                        sensors // odometry sensors
);


void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
	leftweels.tare_position();
	rightweels.tare_position();
	pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
    	
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


void epicredalliancelefty(){
	chassis.setPose(0,0,0);
	chassis.moveToPose(0, -19,0, 2000,{.forwards = false});
	chassis.turnToHeading(-90,1000);
	chassis.moveToPose(-10, -19,-90, 2000,{.forwards = true});
	pros::delay(1000);
	babybrown.move(-70);
	pros::delay(500);
	babybrown.brake();
	pros::delay(700);
	babybrown.move(70);
	pros::delay(200);
	babybrown.brake();
	pros::delay(1000);
	chassis.moveToPose(6, -19, -90, 4000);
}
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
	 



}

void autonomous(){
	chassis.setPose(0,0,0);

	chassis.moveToPose(0, -15,0, 2000,{.forwards = false});
	chassis.turnToHeading(-90,1000);
	chassis.moveToPose(-6, -19,-90, 2000,{.forwards = true});
	pros::delay(1000);
	babybrown.move(-70);
	pros::delay(500);
	babybrown.brake();
	pros::delay(700);
	babybrown.move(70);
	pros::delay(200);
	babybrown.brake();
}

/*{.direction=AngularDirection::CCW_COUNTERCLOCKWISE,.minSpeed=100}*/

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
	pros::Motor babybrown(5);
	int primemodewall=30;
	pros::MotorGroup leftweels({-1,	-2,-3},pros::MotorGearset::blue);
	pros::MotorGroup rightweels({13,12,18},pros::MotorGearset::blue);
	pros::Motor convey(14);
	pros::adi::Pneumatics clamp1('a',true);
	pros::adi::Pneumatics clamp2('b',true);
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor intake(11);
	babybrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	
	
	while (true) {
	 	 // print robot location to the brain screen
		
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
			pros::delay(250);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && ((clamp2.is_extended()==true)&&(clamp1.is_extended()==true))){
			clamp1.retract();
			clamp2.retract();
			pros::delay(250);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			babybrown.move(-100);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			babybrown.move(100);
		}
		else{
			babybrown.brake();
		}


		}
		





	pros::delay(20);
}


