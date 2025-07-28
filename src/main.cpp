#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include <cmath>
#include "robot_config.cpp"
#include "basic_functions.h"
#include "PID.h"
#include "main.h"
#include "odom.h"

#pragma region


void initialize() {


    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    vertical_encoder.reset();
    vertical_encoder.set_reversed(true);
    // print position to brain screen


    /*front_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    middle_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    back_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    front_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    middle_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    back_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);*/


    //pros::c::screen_print(pros::E_TEXT_MEDIUM, 4, "encoder unit: %d", front_left_motor.get_encoder_units());
    pros::Task Intake(bkintake);
    pros::Task BKintake(ftintake);
    pros::Task GPS(GPStracking);
    
    //pros::lcd::clear();
    //pros::lcd::print(5, "IUESHIUSEFIHUWER");

    master.clear();
    pros::delay(100);
    master.print(5, 5, "RED_SAWP");


#if 0
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
#endif

    //master.clear();
    master.print(5, 5, "ha;sdf;jiasdf");


    //disabled();
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */

void disabled() {

}


void competition_initialize() {

}

/**

 */
void autonomous() {
    //chassis.turnToHeading(90,5000);

    //pros::lcd::print(1, "encoder unit: %d", front_left_motor.get_encoder_units());
     //pros::c::screen_print(pros::E_TEXT_MEDIUM, 2, "encoder unit: %d", front_left_motor.get_encoder_units());
    //master.print(1, "encoder unit: %d", front_left_motor.get_encoder_units());
    //moveForMsec(20, true, 6000);
    //moveForSec(20, true, 2);
    
    //pros::c::screen_print(pros::E_TEXT_MEDIUM, 2, "encoder unit: %d", vertical_encoder.get_position());

    pros::c::screen_print(pros::E_TEXT_MEDIUM, 2, "encoder unit");
    
 
    pidMoveold(11, 1, 200);
    pidTurn(-20, 1, 200);
    setftintakespd(-100);
    moveForSec(50, true, 0.4);
    pidTurn(-45, 1, 200);
    pidMoveold(14, 1, 200);
    pidTurn(40, 1, 200);
    moveForSec(30, true, 1.2);
    pros::delay(600);
    pidMoveold(-30, 1, 200);
    setftintakespd(0);
    blockertoggle = !blockertoggle;
    blocker.set_value(blockertoggle);
    pidTurn(-47, 1, 200);
    pidMoveold(22, 1, 200);
    pidTurn(-81, 1, 200);
    moveForSec(30, false, 2);
    setftintakespd(-100);






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
    // loop forever
    bool clamptoggle = false;
    bool doinkertoggle = false;
    bool clampsignal = false;
    
    front_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    middle_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    front_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    middle_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    while (true) {
        // get left y and right x positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);
        
        
        //clamp
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            blockertoggle = !blockertoggle;
            blocker.set_value(blockertoggle);
        }

        /*if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            wallie.move(13000);
            pros::c::delay(145);
            wallie.brake();
        }*/

            
        

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) setbkintakespd(-0.55);

        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) setbkintakespd(100);

        else {
            setbkintakespd(0);
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            setftintakespd(-100);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            setftintakespd(100);
        }
        else {

            setftintakespd(0);
        }
        pros::c::delay(25);
        
    }   
}


	