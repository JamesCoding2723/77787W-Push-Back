#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/distance.h"
#include "pros/distance.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/optical.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include <cmath>
#include "robot_config.h"
#include "basic_functions.h"
#include "PID.h"
#include "main.h"
#include "odom.h"
#include <iostream>

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
    imu.tare();     // calibrate sensors
    imu.set_data_rate(pros::E_MOTOR_ENCODER_DEGREES);
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
    pros::Task Walltask(imu_display_task);
    pros::Task matchload(jeminloaderd);
    //pros::Task GPS(GPStracking);

    // pros::lcd::clear();
    // pros::lcd::print(5, "IUESHIUSEFIHUWER");

    //master.clear();
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
    //PARK CLEAR TEST
    imu.set_heading(0);
    jemintaket();
    setintakespd(-100);
    setintake2spd(-100);
    pidMoveold(19, 1, 2 ,70);
    pros::delay(100);
    //jeminmecht();
    pidTurnRel(-35, 1, 2);
    pros::delay(100);
    //setintakespd(100);
    pidGyro(35, 285, 2, 70, 1, 25, 15, 10000);
    pros::delay(1000);
    moveForSec(70, true, 1.0);
    jeminloadert();
    moveForSec(70, true, 0.7);
    //pros::delay(1000);
    moveForSec(30, false, 0.8);
    pidGyro(38, 225, 3, 70, 1, 25, 15, 100);
    pidTurnAbs(0, 10, 1);
    //pros::delay(1000);
    pidGyro(-25, 0, 2.4);
    jeminmecht();
    setintakespd(-100);
    setintake2spd(-100);




    //AUTO SKILLS
    /*imu.set_heading(180);
    
    jemintaket();
    jeminwingt(); //FIRST HALF

    setintakespd(-100);
    setintake2spd(-100);
    pros::delay(1200);
    moveForSec(60, true, 0.5);
    moveForSec(60, false, 0.1);
    pros::delay(300);
    moveForSec(60, true, 0.1);
    pros::delay(300);
    moveForSec(10, true, 0.5);
    setintakespd(0);
    setintake2spd(0);
    moveForSec(40, false, 0.2);
    moveForSec(60, false, 0.8);
    pros::delay(300);
    moveForSec(25, true, 0.6);
    imu.set_heading(180);

    //pros::delay(200); //
    //jemintaket();

    pidMoveold(-26, 1, 1);
    pros::delay(200);
    pidTurnAbs(90, 1, 1);
    //pros::delay(300);
    setintakespddiff(-100, 0);
    pidMoveold(30, 0.5, 3);
    pros::delay(200);
    
    //setintakespd(00);
    //setintake2spd(00);
    pidTurnAbs(315, 1, 1);
    //pros::delay(300);


    pidMoveold(26, 0.5, 2);
    //moveForSec(30, false, 0.1);

    jemintaket();
    setintakespd(90);
    setintake2spd(0);
    pros::delay(400);
    setintake2spd(54);
    setintakespd(54);
    pros::delay(3800);
    //jeminwingt();
    
    //pidMoveold(-30, 1, 3);
    //pidTurnRel(-10, 1, 1);
    pidwallGyro(24, 310, 90, 3);
    //pros::delay(200);

    jeminloadert();
    jemintaket();
    pidTurnAbs(180, 1, 1.2);



    //pros::delay(2000);

    setintakespd(-100);
    setintake2spd(-100);
    pidMoveold(9, 1, 1);
    moveForSec(55, true, 0.8);
    moveForSec(10, true, 0.9);

    //DONE MATCHLOAD

    pidMoveold(-3, 0.5, 2, 50);

    jeminloadert();
    pidTurnRel(-122, 2, 3, 60);
    //pros::delay(200);
    setintakespddiff(-100, 0);
    setintake2spd(0);

    //pidTurnRel(-42, 1, 2);
    //pros::delay(400);
    pidMoveold(96, 1, 7, 80);
    //pros::delay(200);
    
    pidTurnRel(-70, 2, 2, 70);
    pidWallMove(22.5, 0.5, 90, 3);
    pidTurnAbs(10, 1, 1);
    //pros::delay(1000); //ALIGNED

    // pidTurnRel(-40, 1, 2);
    // pidMoveold(14.5, 0.5, 2);
    // pidTurnRel(43, 1, 2);
    // pros::delay(300);

    
    moveForSec(60, false, 0.5);
    setintakespd(-100);
    setintake2spd(-100);
    jeminmecht();
    jeminloadert();

   //jeminmecht();
    pros::delay(1500); //FIRST SCORE DONE
    imu.set_heading(0);

    pidGyro(34, -4, 2);
    jeminmecht();
    moveForSec(30, true, 1.4);
    setintake2spd(0);
    setintakespd(0);
    pidGyro(-40, 8, 2);
    setintakespd(-100);
    setintake2spd(-100);
    jeminmecht(); 


    pros::delay(1500); //SECOND SCORE DONE

    jeminloadert();
    pidMoveold(7, 0.5, 2, 70); 
    pidTurnAbs(270, 5, 0.6);
    //moveForSec(70, false, 0.9);
    jeminmecht();



    pidGyro(96, 270, 5, 75);
    //pros::delay(200);

    jeminloadert();
    pros::delay(300);
    pidTurnAbs(0, 1, 1);
    setintakespd(-100);
    setintake2spd(-100);
    //pidTurnRel(3, 1, 1);
    pidMoveold(16, 1, 1, 70);
    //jeminmecht();
    moveForSec(60, true, 0.5);
    moveForSec(6.7, true, 0.5);

    //pros::delay(1600); //3 MATCHLOAD DONE



    pidMoveold(-3.5, 0.5, 2, 50);
    jeminloadert();
    pidTurnRel(-120, 2, 3, 60);
 
    setintakespddiff(-100, 0);
    setintake2spd(0);


    pidMoveold(97, 1, 4, 80);
    //pros::delay(200);
    
    pidTurnRel(-70, 1, 2);
    pidWallMove(23, 0.5, 270, 3);
    pidTurnAbs(188, 1, 1);
    //pros::delay(200); //ALIGNED

    setintakespd(-100);
    setintake2spd(-100);
    moveForSec(50, false, 0.8);
    //setintakespd(100);
    //setintake2spd(100);
    //pros::delay(200);


    jeminloadert();
    jeminmecht();
    pros::delay(1500); //THIRD SCORE DONE
    imu.set_heading(180);


    //pidMoveold(1.5, 0.5, 1);
   //pidswingRel(-4, 1, 1, false);
    //pros::delay(500);

    //pidTurnRel(-3, 1, 1);
    pidGyro(34, 172, 2);
    jeminmecht();
    moveForSec(30, true, 1.4);
    setintake2spd(0);
    setintakespd(0);
    pidGyro(-40, 183, 2);

    setintakespd(-100);
    setintake2spd(-100);
    jeminmecht(); 
    pros::delay(1500);
    //jeminmecht(); //FOURTH SCORE DONE
    jeminloadert();

    setintakespd(100);
    setintake2spd(100);
    pidMoveold(20, 1, 2 ,70);
    jeminmecht();
    pros::delay(100);
    pidTurnRel(-35, 1, 2);
    pros::delay(100);
    //setintakespd(100);
    pidGyro(30, 110, 2, 70, 1, 25, 15, 10000);
    moveForSec(80, true, 0.8);
    //*/



    //Right 7 Ball (lower middle goal)
    /*
    imu.set_heading(8.5);

    jemintaket();
    jeminwingt();

    pidMoveold(15, 0.5, 200, 100);
    setintakespddiff(-100, -25);
    pidMoveold(4.3, 0.5, 200, 70);
    pros::delay(300);
    pidMoveold(9.7, 0.5, 200, 40);
    //setintakespd(0);
    pros::delay(100);

    pidTurnRel(-56, 2, 400);
    pros::delay(100);
    pidMoveold(17.5, 0.5, 1, 60);

    jemintaket();
    setintakespd(80); //outtake MIDDLE LOWER
    setintake2spd(80);
    pros::delay(200);
    //pidMoveold(1, 0.5, 1, 30);
    pros::delay(1400);
    jemintaket();

    pidwallGyro(24, 310, 90, 3);
    setintakespd(-10);

    jeminloadert();
    //jemintaket();
    pidTurnAbs(180, 1, 0.8);



    //pros::delay(200);

    setintakespd(-100);
    setintake2spd(-100);
    pidMoveold(9, 1, 1);
    moveForSec(60, true, 0.6);
   //moveForSec(20, true, 0.6);

    moveForSec(55, false, 0.3);
    pidTurnAbs(186, 1, 0.5);
    moveForSec(55, false, 0.9);
    setintakespd(-100);
    setintake2spd(-100);
    jeminmecht(); 
    jeminloadert();
    pros::delay(1100);


    setintakespd(0);
    setintake2spd(0);
    pidTurnRel(-45, 1, 2, 100);
    pidMoveold(10, 0.5, 1, 60);
    jeminwingt();
    pidTurnRel(35, 1, 1);
    pidMoveold(-24, 1, 3, 100);
    */

    //*/

    //Right 4 Ball 
    /*
    imu.set_heading(8.5);

    jemintaket();
    jeminwingt();

    setintakespddiff(-100, -25);
    pidMoveold(19.3, 0.5, 200, 100);
    jeminloadert();
    pidMoveold(9.7, 0.5, 200, 100);
    //setintakespd(0);
    // pros::delay(100);

    pidGyro(32, 115, 3, 100);
    std::cout << "gheigh" << std::endl;

    // pidTurnAbs(180, 1, 0.3);
    pidGyro(0, 180, 1, 100);
    moveForSec(75, false, 0.4);
    setintakespd(-100);
    setintake2spd(-100);
    jeminmecht(); 
    jeminloadert();
    pros::delay(1400);


    setintakespd(0);
    setintake2spd(0);
    pidTurnRel(-45, 1, 2, 100);
    pidMoveold(10, 0.5, 1, 60);
    jeminwingt();
    pidTurnRel(35, 1, 1);
    pidMoveold(-24, 1, 3, 100);
    //*/

    //Left 7 Ball (upper middle goal)
    /*
    imu.set_heading(351.5);

    jemintaket();
    jeminwingt();

    pidMoveold(15, 0.5, 200, 100);
    setintakespddiff(-100, -25);
    pidMoveold(6.3, 0.5, 200, 70);
    // pros::delay(100);
    pidMoveold(10, 0.5, 200, 40);
    //setintakespd(0);
    pros::delay(100);

    pidTurnAbs(225, 1, 1);
    //pros::delay(100);
    pidMoveold(-16, 0.5, 0.7, 60);

    setintakespd(30);
    setintake2spd(30);
    pros::delay(200);
    setintakespd(-90);
    setintake2spd(60); //outtake
    pros::delay(650);


    //jeminwingt();
    setintakespd(0);
    setintakespd(0);
    pidMoveold(52, 0.5, 3, 80);
    pros::delay(300);
    jeminloadert();
    pidTurnAbs(180, 1, 1);

    //jeminloadert();
    setintakespd(-100);
    setintake2spd(-100);
    pidMoveold(13, 1, 1);
    moveForSec(60, true, 0.5);

    moveForSec(55, false, 0.3);
    pidTurnAbs(186, 1, 0.5);
    moveForSec(55, false, 0.9);
    setintakespd(-100);
    setintake2spd(-100);
    jeminmecht(); 
    jeminloadert();
    pros::delay(1100);


    setintakespd(0);
    setintake2spd(0);
    pidTurnRel(-45, 1, 2, 100);
    pidMoveold(10, 0.5, 1, 60);
    jeminwingt();
    pidTurnRel(35, 1, 1);
    pidMoveold(-24, 1, 3, 100);
    //*/

    //moveForSec(50, true, 0.1);

    //Left 4 Ball 
    /*
    imu.set_heading(351.5);

    jemintaket();
    jeminwingt();

    setintakespddiff(-100, -25);
    pidMoveold(19.3, 0.5, 200, 100);
    jeminloadert();
    pidMoveold(9.7, 0.5, 200, 100);
    //setintakespd(0);
    // pros::delay(100);

    pidGyro(35, -120, 3, 100);
    std::cout << "gheigh" << std::endl;

    // pidTurnAbs(180, 1, 0.3);
    pidGyro(0, 180, 1, 100);
    moveForSec(75, false, 0.4);
    setintakespd(-100);
    setintake2spd(-100);
    jeminmecht(); 
    jeminloadert();
    pros::delay(1400);


    setintakespd(0);
    setintake2spd(0);
    pidTurnRel(-45, 1, 2, 100);
    pidMoveold(10, 0.5, 1, 60);
    jeminwingt();
    pidTurnRel(35, 1, 1);
    pidMoveold(-24, 1, 3, 100);
    //*/

    //BETTER SOLO AWP
    /*imu.set_heading(270);
    jemintaket();
    jeminwingt();

    setintakespd(-100);
    setintake2spd(-100);
    moveForSec(30, true, 0.2);
    pidWallMove(27.5, 0.5, 90, 2);
    jeminloadert();
    pidTurnAbs(180, 1, 0.6);
    //pros::delay(200);

    setintakespd(-100);
    setintake2spd(-100);
    pidMoveold(10, 1, 1, 70);
    moveForSec(65, true, 0.9);
    
    moveForSec(100, false, 0.4);
    pidTurnRel(2.5, 1, 0.2, 70);
    // pidTurnAbs(190, 1, 0.5);
    moveForSec(100, false, 0.3);
    jeminmecht();
    //moveForSec(40, false, 0.1);
    setintakespd(-100);
    setintake2spd(-100);
    //moveForSec(10, false, 0.3);
    jeminloadert();
    moveForSec(40, false, 0.3);
    pros::delay(650);
    //jeminmecht();

    
    setintake2spd(-100);
    setintakespd(-100);

    pidMoveold(7, 1, 1, 100);
    // pidTurnRel(96, 1, 1.2, 100);
    pidGyro(23, 294, 2, 100, 10);
    //jeminloadert();
    jeminmecht();

    //pros::delay(1000);
    pidGyro(57.5, 272, 4, 100, 1, 25, 3);
    //jeminloadert();
    //pros::delay(1000);
    //pidMoveold(6, 2, 1,80);

    //pidGyro(5, 272, 3, 70, 1, 25);
    pidTurnAbs(220, 1, 0.5);
    //jeminloadert();
    pidMoveold(-20.3, 0.5, 1, 100);


    setintakespd(100);
    setintake2spd(100);
    pros::delay(100);
    setintakespd(-100);
    setintake2spd(70); //outtake MIDDLE UPPER
    pros::delay(500);
    setintakespd(-100);
    setintake2spd(-100);
    // jeminloadert();

    
    //jeminwingt();

    pidMoveold(49.4, 0.5, 3, 100);
    // pros::delay(100);
    //jeminloadert();
    pidTurnAbs(178, 1, 0.6);

    //jeminloadert();
    //setintake2spd(-100);
    //setintakespd(-100);
    //pidMoveold(10, 1, 1, 100);
    //moveForSec(60, true, 0.2);

    moveForSec(100, false, 0.5);
    jeminmecht();
    // setintakespd(-100);
    // setintake2spd(-100);
    // jeminmecht();
    moveForSec(40, false, 0.3); 
    //*/

    //moveForSec(30, 1, 0.3);'




    //RIGHT 3+4
    /*imu.set_heading(270);
    jemintaket();
    jeminwingt();

    setintakespd(-100);
    setintake2spd(-100);
    pidWallMove(26.7, 0.5, 90, 2);
    jeminloadert();
    pidTurnAbs(180, 1, 0.6);
    //pros::delay(200);

    setintakespd(-100);
    setintake2spd(-100);
    pidMoveold(10, 1, 1, 70);
    moveForSec(65, true, 0.9);
    
    moveForSec(100, false, 0.4);
    //pidTurnRel(2.5, 1, 0.2, 70);
    // pidTurnAbs(190, 1, 0.5);
    moveForSec(100, false, 0.3);
    jeminmecht();
    //moveForSec(40, false, 0.1);
    setintakespd(-100);
    setintake2spd(-100);
    //moveForSec(10, false, 0.3);
    jeminloadert();
    moveForSec(40, false, 0.3);
    pros::delay(650);
    jeminmecht();

    
    setintake2spd(-100);
    setintakespddiff(-100, 0);

    pidMoveold(7, 1, 1, 100);
    // pidTurnRel(96, 1, 1.2, 100);
    pidGyro(29, 294, 3, 80, 1, 25, 10);
    pidGyro(22, 315, 2, 80, 1, 25, 10);
    setintake2spd(0);


    jemintaket();
    setintakespd(100);
    pros::delay(700);
    setintakespd(0);
    jemintaket();


    pidGyro(-26.8, 270, 3, 70, 1, 25, 5, 1000);
    pidGyro(-4, 13, 1.5, 70, 1, 25, 2, 100);

    jeminwingt();
    pros::delay(200);
    moveForSec(70, true, 0.25);
    pidTurnRel(-40, 1, 1, 100);
    //*/


    //LEFT 3+4
    /*imu.set_heading(90);
    jemintaket();
    jeminwingt();

    setintakespd(-100);
    setintake2spd(-100);
    pidWallMove(27.5, 0.5, 270, 2);
    jeminloadert();
    pidTurnAbs(180, 1, 0.6);
    //pros::delay(200);

    setintakespd(-100);
    setintake2spd(-100);
    pidMoveold(9.2, 1, 1, 70);
    moveForSec(50, true, 0.10);
    
    moveForSec(100, false, 0.4);
    pidTurnRel(1, 0.5, 0.2, 70);
    // pidTurnAbs(190, 1, 0.5);
    moveForSec(100, false, 0.3);
    jeminmecht();
    //moveForSec(40, false, 0.1);
    setintakespd(-100);
    setintake2spd(-100);
    //moveForSec(10, false, 0.3);
    jeminloadert();
    moveForSec(40, false, 0.3);
    pros::delay(530);
    jeminmecht();

    
    setintake2spd(-100);
    setintakespddiff(-100, -40);

    pidMoveold(9, 0.5, 1, 100);
    // pidTurnRel(96, 1, 1.2, 100);
    pidGyro(42, 60, 5, 50, 1, 25, 10);
    pidTurnAbs(215, 1, 1, 80);
    moveForSec(30, false, 0.7);
    //setintake2spd(0);

    setintakespd(-100);
    setintake2spd(70);
    pros::delay(800);
    setintakespd(0);


    pidGyro(28, 270, 2, 80, 1, 25, 5, 1000);
    pidGyro(4, 167, 1.5, 70, 1, 25, 1, 100);

    jeminwingt();
    pros::delay(200);
    moveForSec(70, false, 0.25);
    pidTurnRel(40, 1, 1, 100);
    //*/


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

    const float Joystick_LowerDeadzone = 7;

    while (true)
    {
        // get left y and right x positions
        int rightX = abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < Joystick_LowerDeadzone ? 0: master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int leftY = abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) < Joystick_LowerDeadzone ? 0: master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        // move the robot
        moveleft(leftY + 1 * rightX);
        moveright(leftY - 1 * rightX);

        /*int Ch1 = abbs(C1) < Joystick_LowerDeadzone ? 0 : C1;
        int Ch3 = abbs(C3) < Joystick_LowerDeadzone ? 0 : C3;


        moveLeft(Ch3 + 1 * Ch1);
        moveRight(Ch3 - 1 * Ch1);*/

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) //hood
        {
            jeminmechtoggle = !jeminmechtoggle;
            jeminmech.set_value(jeminmechtoggle);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) { //MATHCLOAD
            jeminloadertoggle = !jeminloadertoggle;
            jeminloader.set_value(jeminloadertoggle);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            jemintaketoggle = !jemintaketoggle;
            jemintake.set_value(jemintaketoggle);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            jeminwingtoggle = !jeminwingtoggle;
            jeminwing.set_value(jeminwingtoggle);
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            jeminchoptoggle = !jeminchoptoggle;
            jeminchop.set_value(jeminchoptoggle);
        }

        /*if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            jeminparktoggle = !jeminparktoggle;
            jeminpark.set_value(jeminparktoggle);
        }*/
        

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            setintakespd(-100);
            setintake2spd(-100);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            if (jemintaketoggle == false){
                setintakespd(55);
                setintake2spd(55);
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
            setintakespd(100);
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
