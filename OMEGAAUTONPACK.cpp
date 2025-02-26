
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
pros::Rotation hoarizon(10);

pros::adi::Pneumatics doiniga('c',false);
lemlib::Drivetrain drivetrain(&left, // left motor group
                              &right, // right motor group
                              13.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              480, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)                        
);
lemlib::TrackingWheel vert(&verticale, lemlib::Omniwheel::NEW_2, -1.5);
lemlib::TrackingWheel hori(&hoarizon, lemlib::Omniwheel::NEW_2, -5.5);
lemlib::OdomSensors sensors(&vert, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &hori, // horizontal tracking wheel 1
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
                                              60 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              50, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              00, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              00, // large error range timeout, in milliseconds
                                              50 // maximum acceleration (slew)
);
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate();
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
   // calibrate sensors
    // print position to brain screen
    
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




/*
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
    chassis.moveToPose(0, 15, 0, 1000,{.minSpeed=100});
    chassis.turnToHeading(-90, 1000);
    chassis.moveToPoint(23, 15.5, 2000,{.forwards=false,.maxSpeed=60},false);
    clamp1.extend();
    clamp2.extend();
    chassis.moveToPose(20, 15, -90, 2000,{.forwards=false,.minSpeed=100},false);
    pros::delay(1000);
    chassis.moveToPose(20, 15, 0, 1000,{.forwards=true,.maxSpeed=60},false);
    score.move(500);
    chassis.moveToPose(21, 33, 0, 2000,{.maxSpeed=100},false);
    chassis.moveToPose(44, 37, 90,3000,{.maxSpeed=60});
    chassis.turnToHeading(180, 2000);
    chassis.moveToPose(46, 10, 180, 4000,{.maxSpeed=40});
    chassis.turnToHeading(180, 1000);
    chassis.moveToPose(46, 15, 180, 1000,{.forwards=false},false);
    chassis.moveToPose(60, 15, 90, 2000);
    chassis.turnToHeading(0, 2000,{},false);
    chassis.moveToPose(62, 0, 0, 2000,{.forwards=false},false);
    chassis.turnToHeading(-45, 1000);
    clamp1.retract();
    clamp2.retract();
    score.move(-100);                
    pros::delay(300);
    score.brake();
    //start of quadrant 2
    chassis.moveToPose(62, 20, 0, 1000);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPose(-25, 20, 90, 4000,{.forwards=false,.maxSpeed=80});


   
    

    
    


   
    

    
    
}
void rightthingy(){
    score.move(600);
    pros::delay(2000);
    chassis.setPose(0,0,0);
    imu.tare();
    chassis.moveToPoint(0, 15, 1000);
    chassis.turnToHeading(-90, 1000);
    score.brake();
    chassis.moveToPose(24,15,-90,2000,{.forwards=false},false);
    clamp1.extend();
    clamp2.extend();
    pros::delay(1000);
    chassis.turnToHeading(0, 1000,{},false);
    imu.tare();
    score.move(66600);
    chassis.moveToPoint(22, 39, 2000);
    chassis.turnToPoint(57,66.5,1000);
   
    chassis.moveToPose(57, 66.5,60, 3000,{.minSpeed=100});
    chassis.turnToHeading(90, 1000);
    chassis.turnToHeading(180, 1000);
    chassis.moveToPoint(51, 50, 2000,{.minSpeed=127});
    chassis.moveToPoint(51, 5, 3000,{.maxSpeed=60});
    pros::delay(500);
    chassis.moveToPoint(50, 34,1000,{.forwards=false});
    chassis.turnToPoint(60,10,2000);
    chassis.moveToPose(60, 10, 180,2000,{.minSpeed=127});
    chassis.moveToPose(60, 10, 180,2000,{.minSpeed=127});

    chassis.turnToHeading(90, 1000);
    chassis.turnToHeading(-35, 1000);
    chassis.moveToPoint(70, 0, 2000,{.forwards=false,.maxSpeed=60});
    clamp1.retract();
    clamp2.retract();
    pros::delay(1000);
    score.move(-100);
    pros::delay(200);
    score.brake();
    pros::delay(1000);
    left.move(100);
    right.move(100);
    pros::delay(500);
    left.brake();
    right.brake();

    
    chassis.turnToHeading(90, 1000);
    chassis.moveToPose(-15,10,100,6000,{.forwards=false},false);
    clamp1.extend();
    clamp2.extend();
    chassis.setPose(-24,15,90);

    //start of quad tew
    pros::delay(1000);
    chassis.turnToHeading(0, 1000,{},false);
    score.move(66600);
    chassis.moveToPoint(-22, 39, 2000);
    chassis.turnToPoint(-55,66.5,1000);
   
    chassis.moveToPose(-55, 66.5,-60, 3000,{.minSpeed=100});
    chassis.turnToHeading(-90, 1000);
    chassis.turnToHeading(180, 1000);
    chassis.moveToPoint(-48, 50, 2000,{.minSpeed=127});
    chassis.moveToPoint(-48, 5, 3000,{.maxSpeed=60});
    pros::delay(500);
    chassis.moveToPoint(-50, 34,1000,{.forwards=false});
    chassis.turnToPoint(-60,10,2000);
    chassis.moveToPose(-60, 10, 180,2000,{.minSpeed=127});
    chassis.moveToPose(-60, 10, 180,2000,{.minSpeed=127});
    
    chassis.turnToHeading(35, 1000);
    chassis.moveToPoint(-70, 0, 2000,{.forwards=false,.maxSpeed=60});
    clamp1.retract();
    clamp2.retract();
    pros::delay(1000);
    score.move(-100);
    pros::delay(200);
    score.brake();

}
    */
void tworing(){
    chassis.moveToPoint(0,-25,1500,{.forwards=false,.maxSpeed=60},false);
    chassis.moveToPoint(0,-36,1000,{.forwards=false},false);
    clamp1.extend();
    clamp2.extend();
    chassis.turnToHeading(90, 1000,{},false);
    score.move(127);
    chassis.moveToPoint(24, -36, 1000);
    chassis.moveToPoint(-10, -50, 2000,{},false);
    babybrown.move(-100);
    pros::delay(500);
    babybrown.brake();
}
void fouringred() {
    chassis.moveToPoint(0,-25,1500,{.forwards=false,.maxSpeed=60},false);
    chassis.moveToPoint(0,-36,1000,{.forwards=false},false);
    clamp1.extend();
    clamp2.extend();
    chassis.turnToHeading(90, 1000,{},false);
    score.move(127);
    chassis.moveToPoint(24, -36, 1000);
    chassis.turnToHeading(180, 1000);
    chassis.moveToPoint(18,-60,1400);
    
    chassis.moveToPoint(18,-50, 1000,{.forwards=false});
    chassis.moveToPoint(27, -62, 1000);
    chassis.moveToPoint(-10, -50, 2000,{},false);
    babybrown.move(-100);
    pros::delay(500);
    babybrown.brake();

   
    /* chassis.moveToPoint(34,-48,1000);
    chassis.moveToPoint(34,-60,1000);
    */
 /* chassis.turnToHeading(90, 1000,{},false);
    score.move(1270);
    chassis.moveToPoint(24, -36, 1000,{},false);
    chassis.turnToHeading(180, 1000,{},false);
    chassis.moveToPoint(20, -60, 1000,{});
    chassis.moveToPoint(20,-53,1000,{.forwards=false},false);
    chassis.moveToPoint(30, -60, 1000,{});
    chassis.moveToPoint(30, -64, 1000,{});
    chassis.turnToHeading(180, 1000);*/

    






    




    
}
void fouringblue() {

    chassis.moveToPoint(-0,-25,1500,{.forwards=false,.maxSpeed=60},false);
    chassis.moveToPoint(-0,-36,1000,{.forwards=false},false);
    clamp1.extend();
    clamp2.extend();
    chassis.turnToHeading(-90, 1000,{},false);
    score.move(127);
    chassis.moveToPoint(-30, -36, 2000);
    chassis.turnToHeading(-180, 1000);
    chassis.moveToPoint(-24,-60,1400);
    
    chassis.moveToPoint(-24,-50, 1000,{.forwards=false});
    chassis.moveToPoint(-32, -62, 1000);
    chassis.moveToPoint(7, -50, 2000,{},false);
    babybrown.move(-100);
    pros::delay(500);
    babybrown.brake();
    
}




    


    
    

void goalrush(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    intake.move(1270);
    chassis.moveToPoint(0, 41, 1000,{.minSpeed=1270},false);
    doiniga.extend();
    pros::delay(500);
    chassis.moveToPoint(-1, 22, 1000,{.forwards=false},false);
    doiniga.retract();
    pros::delay(500);
    chassis.turnToHeading(180, 1000,{},false);
    chassis.moveToPoint(7,38,2000,{.forwards=false,.maxSpeed=80},false);
    clamp1.extend();
    clamp2.extend();
    pros::delay(500);
    score.move(127);
    pros::delay(250);
    score.brake();
    intake.move(127);
    chassis.moveToPoint(-7,5,1000,{.forwards=false},false);
    clamp1.retract();
    clamp2.retract();
    chassis.moveToPoint(-7,34,1000,{},false);
    chassis.moveToPoint(-26,34,2000,{.forwards=false,.maxSpeed=60},false);
    clamp1.extend();
    clamp2.extend();
    pros::delay(500);
    score.move(127);
    chassis.moveToPose(-40,38,325,2000,{.minSpeed=10000},false);
    chassis.turnToHeading(325, 1000,{},false);
    babybrown.move(-100);
    pros::delay(1000);
    babybrown.brake();


    


}
void skillsauto(){
    score.move(100);
    pros::delay(500);
    score.brake();
    chassis.moveToPoint(0, 15, 1000);
    chassis.moveToPoint(18, 15, 1000,{.forwards=false,.maxSpeed=80},false);
    clamp1.extend();
    clamp2.extend();
    pros::delay(250);
    score.move(100);
    chassis.moveToPoint(25.5, 43.5, 1000);
    chassis.moveToPoint(62.5,68.5,2000);
    chassis.moveToPoint(52, 38, 1000);
    chassis.moveToPoint(57.5, 3, 3000,{.maxSpeed=60});
    chassis.moveToPoint(57.5, 28.5, 1000,{.forwards=false});
    chassis.moveToPoint(67.7,18, 1000);
    chassis.turnToHeading(0, 1000);
    chassis.moveToPose(69, 5, -45,2000,{.forwards=false},false);



}
void autonomous(){
    skillsauto();
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
        if (babybrown.get_position()<-1100 && master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
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
