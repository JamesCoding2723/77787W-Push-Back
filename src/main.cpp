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
#include "robot_config.h"
#include "basic_functions.h"
#include "PID.h"
#include "main.h"
#include "odom.h"

#pragma region

void initialize()
{
    front_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    middle_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    back_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    front_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    middle_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    back_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate();     // calibrate sensors
    vertical_encoder.reset();
    vertical_encoder.set_reversed(true);
    // print position to brain screen

    /*front_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    middle_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    back_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    front_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    middle_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    back_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);*/

    // pros::c::screen_print(pros::E_TEXT_MEDIUM, 4, "encoder unit: %d", front_left_motor.get_encoder_units());
    pros::Task Intake(intake);
    pros::Task Intake2(intake2);
    //pros::Task GPS(GPStracking);

    // pros::lcd::clear();
    // pros::lcd::print(5, "IUESHIUSEFIHUWER");

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

    // master.clear();
    master.print(5, 5, "ha;sdf;jiasdf");

    // disabled();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */

void disabled()
{
}

void competition_initialize()
{
}

/**

 */
void autonomous()
{
    //WTV
    // imu.set_heading(0);
    // pidTurnRel(90, 1, 400);

    //new robot auton
    pidMoveold(15, 0.5, 200, 100);
    setintakespddiff(-100, -25);
    pidMoveold(4.3, 0.5, 200, 70);
    pros::delay(300);
    pidMoveold(9.3, 0.5, 200, 40);
    setintakespd(0);
    pros::delay(100);

    pidTurnRel(-54, 2, 400);
    pros::delay(100);
    pidMoveold(15.5, 0.5, 200, 60);

    jemintaket();
    setintakespd(75); //outtake
    pros::delay(200);
    pidMoveold(1, 0.5, 200, 30);
    pros::delay(1500);
    jemintaket();
    pros::delay(200);
    setintakespd(0);

    pidMoveold(-55.2, 0.5, 200, 100);
    pros::delay(100);
    pidTurnRel(-130.5, 2, 200);

    jeminloadert();
    pros::delay(400);
    moveForSec(55, true, 0.75);
    pros::delay(100);
    setintakespddiff(-100, -25);
    moveForSec(10, false, 0.2);
    pros::delay(600);
    moveForSec(10, true, 0.1);
    pros::delay(300);

    moveForSec(55, false, 0.8);
    pidTurnRel(0.4, 0.5, 200);
    moveForSec(55, false, 0.2);
    setintakespd(-100);
    setintake2spd(-100);
    moveForSec(10, false, 0.3);
    jeminloadert();

    //LEFT 7BALL CODE
    /*setintakespd(-100);
    pidMoveold(13.5, 0.5, 200, 50);
    pidTurnRel(-30, 2, 200);
    pidMoveold(14.5, 0.5, 200, 15); // go into the 3 blocks
    setintakespd(0);
    setintakerspd(0);
    //pros::delay(2000);

    pidTurnRel(-96, 1, 50);
    pros::delay(200);
    jeminrooft();
    moveForSec(30, false, 1.1); // go to middle goal
    setintakerspd(-100); //scoring
    pros::delay(2000);
    setintakerspd(0);

    setintakerspd(20);
    pidMoveold(49, 0.5, 200, 60); //going to mtach load
    setintakerspd(0);
    pidTurnRel(-48, 2, 50);
    moveForSec(30, false, 0.1);
    pros::delay(500);
    jeminmecht();
    jeminrooft();
    pros::delay(100);
    setintakespd(-100);
    moveForSec(40, true, 0.7); //collecting match load
    pros::delay(200);
    moveForSec(30, false, 0.2);
    pros::delay(200);
    moveForSec(30, true, 0.4);
    pros::delay(400);

    setintakespd(0);
    jeminrooft();
    // moveForSec(60, false, 0.3);
    // pidTurnRel(-1.4, 0.5, 100);
    moveForSec(50, false, 1.4);
    setintakespd(-100);*/


    //RIGHT 7BALL CODE
    /*setintakespd(-70);
    pidMoveold(14, 0.5, 200, 50);
    pidTurnRel(39, 2, 200);
    pidMoveold(16, 0.5, 200, 15); // go into the 3 blocks
    setintakespd(0);
    setintakerspd(0);
    //pros::delay(2000);

    pidTurnRel(-92, 1, 50);
    jeminrooft();
    jeminmecht();
    pros::delay(500);
    moveForSec(35, true, 0.5); // go to middle goal
    setintakerspd(100);
    pros::delay(1500);
    moveForSec(35, false, 0.2);
    setintakerspd(0);

    jeminmecht();
    pidTurnRel(150, 1, 200);
    pidMoveold(35, 0.5, 200, 60); //going to mtach load
    setintakerspd(0);
    pidTurnRel(43, 2, 50);
    jeminmecht();
    jeminrooft();
    pros::delay(500);
    setintakespd(-100);
    moveForSec(80, true, 0.6); //collecting match load
    pros::delay(100);
    moveForSec(30, false, 0.2);
    pros::delay(100);
    moveForSec(50, true, 0.3);
    pros::delay(1000);

    setintakespd(0);
    moveForSec(55, false, 0.6);
    pidTurnRel(3, 0.5, 200);
    moveForSec(55, false, 1);
    jeminrooft();
    setintakespd(-100);*/



   
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
void opcontrol()
{
    front_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    middle_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    front_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    middle_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    while (true)
    {
        // get left y and right x positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) //dble park
        {
            jeminmechtoggle = !jeminmechtoggle;
            jeminmech.set_value(jeminmechtoggle);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) { //chopstick
            jeminloadertoggle = !jeminloadertoggle;
            jeminloader.set_value(jeminloadertoggle);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            jemintaketoggle = !jemintaketoggle;
            jemintake.set_value(jemintaketoggle);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            jeminwingtoggle = !jeminwingtoggle;
            jeminwing.set_value(jeminwingtoggle);
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            setintake2spd(-100);
            setintakespd(-100);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            if (jemintaketoggle == true){
                setintakespd(80);
                setintake2spd(80);
            }
            else {
                setintakespd(100);
                setintake2spd(100);
            }
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            setintakespd(-100);
            setintake2spd(50);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            setintakespd(-100);
            //setintake2spd(-100);
        }
        else 
        {
            setintakespd(0);
            setintake2spd(0);
        }
        pros::c::delay(25);
    }
}
