#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
pros::MotorGroup left({-1,-2,-3});
pros::MotorGroup right({13,12,18});
pros::adi::Pneumatics clamp1('a',false);
pros::adi::Pneumatics clamp2('b',false);
pros::Motor babybrown(15);
pros::Motor intake(20);
pros::Motor convey(14);
pros::Imu imu(21);
pros::MotorGroup score({20,-14});
pros::Rotation verticale(-16);
pros::adi::Pneumatics doiniga('c',false);
lemlib::Drivetrain drivetrain(&left, // left motor group
                              &right, // right motor group
                              13.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              480, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)                        
);
lemlib::TrackingWheel vert(&verticale, lemlib::Omniwheel::NEW_2, -1.5);
lemlib::OdomSensors sensors(&vert, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              50, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              00, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              00, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);
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
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate();
    left.tare_position();
    right.tare_position(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(1, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(2, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(3, "Theta: %f", chassis.getPose().theta);
             // heading
            // delay to save resources
            pros::delay(20);
        }
    });
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
void mainleft(){
    chassis.setPose(0,0,0);
    chassis.moveToPose(0, -17, 0, 1000,{.forwards=false},false);
    chassis.moveToPose(0,-22,0,1000,{.forwards=false},false);
    clamp1.extend();
    clamp2.extend();
    pros::delay(1000);
    intake.move(600);
    convey.move(-600);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPose(15, -22, 90,2000,{.forwards=true},false);
    chassis.moveToPose(-50,-17, 90, 1000,{.forwards=true},false);
    intake.move(100);
    convey.move(-100);
}
void mainCode(){
    chassis.setPose(0,0,0);
    chassis.moveToPose(0, -17, 0, 1000,{.forwards=false},false);
    chassis.moveToPose(0,-22,0,1000,{.forwards=false},false);
    clamp1.extend();
    clamp2.extend();
    pros::delay(1000);
    intake.move(600);
    convey.move(-600);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPose(15, -22, 90,2000,{.forwards=true},false);
    chassis.moveToPose(-30,-17, 90, 10000,{.forwards=true},false);
    intake.move(100);
    convey.move(-100);




}
void ultimacyskills(){
    imu.tare_heading();
    verticale.reset_position();
    convey.move(-600);
    intake.move(100);
    pros::delay(2000);
    convey.brake();
    convey.move(100);
    pros::delay(500);
    convey.brake();
    chassis.setPose(0,0,0);
    chassis.moveToPose(0, 15.5, 0, 2000,{.minSpeed=100});
    chassis.moveToPose(22, 15.5, -90, 2000,{.forwards=false,.minSpeed=60},false);
    clamp1.extend();
    clamp2.extend();
    chassis.moveToPose(22, 15.5, -90, 2000,{.forwards=false,.minSpeed=100},false);
    pros::delay(1000);
    chassis.moveToPose(22, 15.5, 0, 1000,{.forwards=true,.maxSpeed=60},false);
    score.move(500);
    chassis.moveToPose(23, 15.5, 0, 500,{.forwards=false,.maxSpeed=60},false);
    chassis.moveToPose(23, 37, 0, 2000,{.maxSpeed=100},false);
    chassis.moveToPose(43, 37, 90,2000,{.maxSpeed=60});
    chassis.moveToPose(43, 37, 0,2000,{.maxSpeed=60});
    

    chassis.moveToPose(54, 60, 0,3000);
    chassis.turnToHeading(90, 1000);

    chassis.turnToHeading(180, 2000);
    chassis.moveToPose(57, 5, 180,3000,{.maxSpeed=60});
    chassis.moveToPose(57, 30, 180,2000,{.forwards=false,.maxSpeed=60,});
    chassis.turnToHeading(-140, 3000);
    chassis.moveToPose(30, -10, -10, 4000,{.forwards=true,.maxSpeed=40});
    chassis.turnToHeading(180, 1000);
    chassis.turnToHeading(-90, 1000);
    chassis.moveToPose(60,0,-90, 2000,{.forwards=false},false);
    clamp1.retract();
    clamp2.retract();
    

    
    
    /*chassis.moveToPose(52, 15, 0, 2000,{.forwards=true,.maxSpeed=100});
    chassis.moveToPose(52, 15, -90, 2000,{.forwards=true,.maxSpeed=100});
    chassis.moveToPose(60, 15, -90, 2000,{.forwards=false},false);
    clamp1.retract();
    clamp2.retract();
    chassis.moveToPose(0, 0, 0, 5000);
    //part two
    chassis.setPose(0,0,0);
    chassis.moveToPose(0, 15.5, 0, 3000,{.maxSpeed=60});
    chassis.moveToPose(-25, 15.5, 90, 3000,{.forwards=false,.maxSpeed=60},false);
    clamp1.extend();
    clamp2.extend();
    chassis.moveToPose(-25, 15.5, 0, 3000,{.forwards=true,.maxSpeed=60},false);
    score.move(500);
    chassis.moveToPose(-25, 41, 0, 2000,{.maxSpeed=60},false);
    chassis.moveToPose(-50, 41, -90,2000,{.maxSpeed=60});
    chassis.moveToPose(-50, 65, 0,2000,{.maxSpeed=60});
    chassis.moveToPose(-50, 65, -90,2000);
    chassis.moveToPose(-60, 10, -180,4000,{.maxSpeed=60});
    chassis.moveToPose(-60, 35, -180,2000,{.forwards=false,.minSpeed=100,});
    chassis.moveToPose(-60, 35, 130,1000,{.maxSpeed=60});
    chassis.moveToPose(-52, 1.5, -180, 3000,{.forwards=true,.maxSpeed=20});
    chassis.moveToPose(-52, 0, -180, 5000,{.forwards=true,.maxSpeed=100});
    chassis.turnToHeading(180, 1000);
    chassis.moveToPose(52, 15, 0, 2000,{.forwards=true,.maxSpeed=100});
    chassis.moveToPose(52, 15, 90, 2000,{.forwards=true,.maxSpeed=100});
    chassis.moveToPose(60, 15, 90, 2000,{.forwards=false},false);
    clamp1.retract();
    clamp2.retract();*/

    
    

 
    

  
    
    

    
    
}
void rightthingy(){
    pros::MotorGroup dive({-1,-2,-3,13,12,18});
    chassis.setPose(0,0,0);
    chassis.moveToPose(0, -31,0 ,2000,{.forwards=false},false);
    chassis.moveToPose(5, -31, -90, 3000,{.forwards=false},false);
    convey.move(-40);
    pros::delay(2000);
    convey.brake();
    dive.move(30);
    pros::delay(700);
    dive.brake();
    chassis.turnToHeading(-225, 2000,{},false);
    pros::delay(500);
    dive.move(-70);
    pros::delay(1100);
    dive.brake();
    clamp1.extend();
    clamp2.extend();
    chassis.turnToHeading(0, 2000,{},false);
    intake.move(100);
    convey.move(-100);
    dive.move(100);
    pros::delay(500);
    dive.brake();
    intake.brake();
}
void autonomous() {
    ultimacyskills();
    
}
void epicness(){
    chassis.setPose(0,0,0);
    chassis.moveToPose(0, -17, 0, 1000,{.forwards=false},false);
    chassis.moveToPose(0,-22,0,1000,{.forwards=false},false);
    clamp1.extend();
    clamp2.extend();
    pros::delay(1000);
    intake.move(600);
    convey.move(-600);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPose(15, -22, -90,2000,{.forwards=true},false);
    chassis.moveToPose(-30,-17, -90, 3000,{.forwards=true},false);
    intake.move(100);
    convey.move(-100);
    
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
    babybrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


	while (true) {  
		left.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		right.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
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
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			babybrown.move(-50);
	
		}
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            babybrown.move(50);
        }
        else{
            babybrown.brake();
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
        if (babybrown.get_position()<-1400 && master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            babybrown.move(5);
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
            babybrown.set_zero_position(0);
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)&& doiniga.is_extended()==true){
            doiniga.retract();
            pros::delay(250);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)&& doiniga.is_extended()==false){
            doiniga.extend();
            pros::delay(250);
        }
    

		
        pros::delay(20);
	}

    
    
}
