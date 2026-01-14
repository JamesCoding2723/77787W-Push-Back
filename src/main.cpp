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
    imu.set_heading(0);

    pidTurnAbs(90, 1, 1);



    //AUTO SKILLS
    
    /*jemintaket();
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

    pros::delay(200); //
    //jemintaket();

    pidMoveold(-21, 1, 1);
    pros::delay(400);
    pidTurnRel(-82, 2, 1);
    pros::delay(300);
    setintakespd(-100);
    setintake2spd(-100);
    pidMoveold(32, 0.5, 3);
    pros::delay(300);
    
    //setintakespd(00);
    //setintake2spd(00);
    pidTurnRel(-132, 2, 2);
    pros::delay(300);


    pidMoveold(27.25, 0.5, 1.8);

    jemintaket();
    setintakespd(75);
    setintake2spd(0);
    pros::delay(50);
    setintake2spd(60);
    setintakespd(60);
    pros::delay(3200);
    //jeminwingt();
    

    pidMoveold(-55, 0.5, 200, 80);
    pros::delay(200);

    jeminloadert();
    jemintaket();
    pidTurnRel(-126, 1.5, 2);



    pros::delay(200);

    setintakespd(-100);
    setintake2spd(-100);
    pidMoveold(9, 1, 1);
    moveForSec(55, true, 0.8);
    moveForSec(20, true, 0.8);

    //DONE MATCHLOAD

    pidMoveold(-5, 0.5, 2, 50);

    jeminloadert();
    pidTurnRel(-143, 2, 3, 50);
    pros::delay(200);
    setintakespddiff(0, 0);
    setintake2spd(0);

    //pidTurnRel(-42, 1, 2);
    //pros::delay(400);
    pidMoveold(96, 1, 7, 80);
    pros::delay(200);
    
    pidTurnRel(-79, 2, 2, 70);
    moveForSec(80, false, 0.5);
    pidMoveold(15, 0.5, 2);
    pidTurnRel(83, 2, 2);
    pros::delay(200); //ALIGNED

    // pidTurnRel(-40, 1, 2);
    // pidMoveold(14.5, 0.5, 2);
    // pidTurnRel(43, 1, 2);
    // pros::delay(300);

    setintakespd(-100);
    setintake2spd(-100);
    moveForSec(60, false, 0.5);
    //setintakespd(100);
    //setintake2spd(100);
    jeminmecht();
    jeminloadert();

   //jeminmecht();
    pros::delay(1400); //FIRST SCORE DONE

    //jeminloadert();
    //pidMoveold(1.5, 0.5, 1);
   //pidswingRel(-4, 1, 1, false);
    //jeminmecht();

    //jeminloadert();
    //jemintaket();
    //setintake2spd(-100);
    //setintakespd(-100);


    pidMoveold(18, 1, 1, 70);

    

    jeminmecht();
    moveForSec(60, true, 0.9);
    moveForSec(10, true, 1.4); //second mathclaod


    //pidMoveold(-3, 0.5, 1);
    //pidTurnRel(2.5, 1, 1);    
    moveForSec(70, false, 0.4);
    pidTurnRel(1, 0.5, 1);
    moveForSec(70, false, 0.5);    
    jeminmecht(); 
    //setintakespd(100);
    //setintake2spd(100);
    //pros::delay(200);
    setintakespd(-100);
    setintake2spd(-100);
    pros::delay(1400); //SECOND SCORE DONE

    //jemintaket();
    //jeminmecht();

    setintakespd(0);
    setintake2spd(0);
    jeminloadert();
    pidMoveold(7, 0.5, 2, 70); 
    pidTurnRel(-75, 2, 1);
    moveForSec(70, false, 0.9);



    pidMoveold(115, 1, 5, 75);
    pros::delay(200);

    jeminloadert();
    pidTurnRel(67, 1, 2, 50);
    setintakespd(-100);
    setintake2spd(-100);
    //pidTurnRel(3, 1, 1);
    pidMoveold(15, 1, 1, 70);
    jeminmecht();
    moveForSec(60, true, 0.5);
    moveForSec(30, true, 0.4);
    //pros::delay(300);
    //moveForSec(10, false, 0.2);
    //pros::delay(200);
    //moveForSec(10, true, 0.1);
    //pros::delay(1600); //3 MATCHLOAD DONE



    pidMoveold(-6, 0.5, 2, 50);
    //setintake2spd(-75);
    //setintakespd(-75);
    jeminloadert();
    pidTurnRel(-146, 1, 2, 50);
    pros::delay(200);
    //pidMoveold(13.3, 0.5, 1, 50);

    //pidTurnRel(-38.5, 1, 1);
    //pros::delay(200);
    pidMoveold(97, 1, 4, 80);
    pros::delay(200);
    
    pidTurnRel(-85, 1, 2);
    moveForSec(75, false, 0.7);
    pidMoveold(15, 0.5, 2);
    pidTurnRel(84, 1, 2);
    pros::delay(200); //ALIGNED

    setintakespd(-100);
    setintake2spd(-100);
    moveForSec(50, false, 0.8);
    //setintakespd(100);
    //setintake2spd(100);
    //pros::delay(200);


    jeminloadert();
    jeminmecht();
    pros::delay(1400); //THIRD SCORE DONE


    //pidMoveold(1.5, 0.5, 1);
   //pidswingRel(-4, 1, 1, false);
    //pros::delay(500);
    //pidTurnRel(-3, 1, 1);
    pidMoveold(26, 1, 1, 70);
    jeminmecht();
    moveForSec(60, true, 0.5);
    moveForSec(10, true, 1.4);

    //pros::delay(100);
    //setintakespd(-100);
    //setintake2spd(-100);
    //moveForSec(10, false, 0.2);
    //pros::delay(300);
    //moveForSec(10, true, 0.1);
    //pros::delay(1600); //MATCHLOAD DONE
    setintakespd(0);
    setintake2spd(0);

    pidMoveold(-2, 0.5, 1); 
    //pidTurnRel(3, 1, 1);
    //pidswingRel(-4, 1, 1, true);
    moveForSec(70, false, 0.25);
    pidTurnRel(1, 0.5, 1);
    moveForSec(70, false, 0.65); 
    jeminmecht(); 
    //setintakespd(100);
    //pros::delay(200);
    setintakespd(-100);
    setintake2spd(-100);
    pros::delay(1400);
    //jeminmecht(); //FOURTH SCORE DONE
    jeminloadert();

    pidMoveold(7, 1, 2 ,70);
    jeminmecht();
    pros::delay(100);
    pidTurnRel(-35, 1, 2);
    pros::delay(100);
    setintakespd(100);
    pidMoveold(34, 1, 2, 70);
    pidTurnRel(-21, 1, 3, 70);
    moveForSec(90, true, 0.84);
    //*/



    //Right 7 Ball (lower middle goal)
    /*jemintaket();
    jeminwingt();

    pidMoveold(15, 0.5, 200, 100);
    setintakespddiff(-100, -25);
    pidMoveold(4.3, 0.5, 200, 70);
    pros::delay(300);
    pidMoveold(11, 0.5, 200, 40);
    //setintakespd(0);
    pros::delay(100);

    pidTurnRel(-51, 2, 400);
    pros::delay(100);
    pidMoveold(14, 0.5, 1, 60);

    jemintaket();
    setintakespd(80); //outtake MIDDLE LOWER
    setintake2spd(80);
    pros::delay(200);
    //pidMoveold(1, 0.5, 1, 30);
    pros::delay(1000);
    jemintaket();
    setintakespd(-10);

    pidMoveold(-52, 0.5, 2, 80);
    pros::delay(200);

    jeminloadert();
    //jemintaket();
    pidTurnRel(-126, 2, 200);



    pros::delay(200);

    setintakespd(-100);
    setintake2spd(-100);
    pidMoveold(9, 1, 1);
    moveForSec(60, true, 0.7);
   //moveForSec(20, true, 0.6);

    moveForSec(55, false, 1.2);
    setintakespd(-100);
    setintake2spd(-100);
    jeminmecht(); 
    jeminloadert();
    pros::delay(1200);


    setintakespd(0);
    setintake2spd(0);
    pidTurnRel(-45, 1, 2, 100);
    pidMoveold(10, 0.5, 1, 60);
    jeminwingt();
    pidTurnRel(32, 1, 1);
    pidMoveold(-24, 1, 3, 100);

    //*/



    //Left 7 Ball (upper middle goal)
    /*jemintaket();
    jeminwingt();

    pidMoveold(15, 0.5, 200, 100);
    setintakespddiff(-100, -25);
    pidMoveold(6.3, 0.5, 200, 70);
    pros::delay(100);
    pidMoveold(10.5, 0.5, 200, 40);
    //setintakespd(0);
    pros::delay(100);

    pidTurnRel(-120, 2, 400);
    pros::delay(100);
    pidMoveold(-16, 0.5, 0.7, 60);

    setintakespd(-90);
    setintake2spd(60); //outtake
    pros::delay(650);
    setintakespd(60);
    setintake2spd(60);

    //jeminwingt();

    pidMoveold(55, 0.5, 3, 100);
    pros::delay(300);
    jeminloadert();
    pidTurnRel(-39, 2, 1);

    //jeminloadert();
    setintakespd(-100);
    setintake2spd(-100);
    pidMoveold(9, 1, 1);
    moveForSec(60, true, 0.58);

    moveForSec(75, false, 0.4);
    //pidTurnRel(1, 0.5, 1, 70);
    moveForSec(60, false, 0.5);
    jeminmecht();
    pidTurnRel(-0.4, 0.5, 200);
    moveForSec(40, false, 0.1);
    setintakespd(-100);
    setintake2spd(-100);
    moveForSec(10, false, 0.3);
    jeminloadert();
    pros::delay(1100);

    setintakespd(0);
    setintake2spd(0);
    pidTurnRel(-45, 1, 2, 100);
    pidMoveold(10, 0.5, 1, 60);
    jeminwingt();
    pidTurnRel(33, 1, 1);
    pidMoveold(-25.5, 1, 3, 90);

    //*/

    //moveForSec(50, true, 0.1);


    //WEIRD SOLO AWP
    /*jemintaket();
    jeminwingt();

    pidMoveold(15, 0.5, 200, 100);
    setintakespddiff(-100, -25);
    pidMoveold(4.3, 0.5, 200, 70);
    pros::delay(300);
    pidMoveold(11, 0.5, 200, 40);
    setintakespd(0);
    pros::delay(100);

    pidTurnRel(-51, 2, 400);
    pros::delay(100);
    pidMoveold(15.5, 0.5, 1, 60);

    jemintaket();
    setintakespd(80); //outtake MIDDLE LOWER
    pros::delay(200);
    //pidMoveold(1, 0.5, 1, 30);
    pros::delay(800);
    jemintaket();
    setintakespd(0);

    pidMoveold(-16, 0.5, 200, 80);
    pros::delay(200);
    pidTurnRel(-39, 1, 2);
    pros::delay(300);
    setintakespd(-100);
    setintake2spd(10);
    pidMoveold(34, 2, 2, 80);
    jeminloadert();
    pidMoveold(10, 1, 1);

    pidTurnRel(-39, 2, 400);
    pros::delay(100);
    pidMoveold(-13, 0.5, 1.1, 60);

    setintakespd(-100);
    setintake2spd(70); //outtake MIDDLE UPPER
    pros::delay(800);
    setintakespd(0);
    setintake2spd(0);
    jeminwingt();

    pidMoveold(52, 0.5, 200, 100);
    pros::delay(100);
    pidTurnRel(-45, 1, 200);

    //jeminloadert();
    pros::delay(200);
    setintakespddiff(-100, -100);
    moveForSec(40, true, 1.1);
    pros::delay(100);
    moveForSec(10, false, 0.1);
    pros::delay(200);
    moveForSec(10, true, 0.1);
    pros::delay(100);

    moveForSec(90, false, 0.8);
    jeminmecht();
    setintakespd(-100);
    setintake2spd(-100);
    //moveForSec(10, false, 0.3);   */



    //BETTER SOLO AWP
    /*jemintaket();
    jeminwingt();

    setintakespd(-100);
    setintake2spd(-100);
    pidMoveold(3, 1, 1, 100);
    pidMoveold(-45, 0.5, 2, 70);
    jeminloadert();
    pidTurnRel(-82, 2, 2, 50);
    pros::delay(200);

    setintakespd(-100);
    setintake2spd(-100);
    pidMoveold(10, 1, 1, 70);
    moveForSec(60, true, 0.7);

    moveForSec(75, false, 0.4);
    //pidTurnRel(1, 0.5, 1, 70);
    moveForSec(60, false, 0.5);
    jeminmecht();
    //pidTurnRel(3, 1, 1, 70);
    //moveForSec(40, false, 0.1);
    setintakespd(-100);
    setintake2spd(-100);
    //moveForSec(10, false, 0.3);
    jeminloadert();
    pros::delay(1000);
    //jeminmecht();


    setintake2spd(-100);
    setintakespd(-100);

    moveForSec(20, true, 0.2);
    pidTurnRel(93, 1, 1.2, 80);
    pidMoveold(10, 1, 1, 80);
    jeminmecht();


    //pidTurnRel(-4, 1, 1);
    pidMoveold(43, 1, 1, 80);
    jeminloadert();
    pidMoveold(10, 1, 1,80);

    pidTurnRel(-45, 2, 400);
    pros::delay(100);
    pidMoveold(-16.6, 0.5, 1, 60);


    setintakespd(100);
    setintake2spd(100);
    pros::delay(200);
    setintakespd(-100);
    setintake2spd(70); //outtake MIDDLE UPPER
    pros::delay(430);
    setintakespd(20);
    setintake2spd(0);
    //jeminwingt();

    pidMoveold(52, 0.5, 200, 100);
    pros::delay(100);
    //jeminloadert();
    pidTurnRel(-41, 1, 200);

    //jeminloadert();
    //setintake2spd(-100);
    //setintakespd(-100);
    //pidMoveold(10, 1, 1, 100);
    //moveForSec(60, true, 0.2);

    moveForSec(100, false, 0.7);
    //jeminmecht();
    setintakespd(-100);
    setintake2spd(-100);
    jeminmecht();
    //moveForSec(10, false, 0.3); */

    //moveForSec(30, 1, 0.3);


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
                setintakespd(60);
                setintake2spd(60);
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
