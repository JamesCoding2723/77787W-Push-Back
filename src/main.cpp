#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include <cmath>
#include "pros/rtos.hpp"
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
    
    leftintakem.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

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
    pros::Task ColorSort(color_sort);
    pros::Task Store(store);
    pros::Task Score(score);
    top_color_sensor.set_led_pwm(100);
    //pros::Task GPS(GPStracking);

    // pros::lcd::clear();
    // pros::lcd::print(5, "IUESHIUSEFIHUWER");

    //master.clear();
  
   

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

    setintake2spd(0);
    setintakespd(0);
    sort_on = false;

    //AUTO SKILLS
    /*imu.set_heading(180);
    
    jemintaket();//FIRST HALF
    jeminmecht();
    jeminwingt();


    setintakespd(-100);
    setintake2spd(0);
    moveForSec(50, true, 0.05);
    moveForSec(50, false, 0.1);
    pros::delay(1000);
    moveForSec(60, true, 0.5);
    moveForSec(60, false, 0.1);
    pros::delay(300);
    moveForSec(60, true, 0.1);
    pros::delay(300);
    moveForSec(10, true, 0.3);
    moveForSec(60, false, 0.65);
    setintakespd(0);
    setintake2spd(0);


    pidFrontWallGyro(24, 180, 0, 0.8, 60);
    pidTurnAbs(47, 10, 1);
    setintakespddiff(-100, 0);
    pidGyro(22, 47, 2, 100);


    pidGyro(22, 315, 1.5, 70, 1, 25, 1, 800);

    moveForSec(30, true, 0.4);
    jemintaket();
    setintakespddiff(80, 70);
    setintake2spd(-5);
    pros::delay(600);
    setintake2spd(-10);
    setintakespddiff(50, 40);
    pros::delay(400);
    setintakespddiff(40, 30);
    pros::delay(700);
    setintakespddiff(-30, -30);
    pros::delay(100);
    setintakespddiff(40, 30);
    pros::delay(800);

    //jeminmecht();
    pidwallGyro(24.5, 310, 90, 3);

    jeminloadert();
    jemintaket();
    pidTurnAbs(182, 1, 2);


    setintakespd(-100);
    setintake2spd(0);
    pidGyro(19, 180, 1);
    moveForSec(55, true, 0.5);
    moveForSec(10, true, 1.3);
    //DONE MATCHLOAD

    moveForSec(60, false, 0.13);
    jeminloadert();
    pidTurnRel(50, 2, 3, 60);
    setintakespddiff(-100, 0);
    setintake2spd(0);

    pidGyro(-99, 180, 5.3, 90, 30, 100, 2, 100);
    pidwallGyro(37, 180, 180, 1.5, 100);
    
    pidTurnAbs(270, 10, 0.8);
    pidWallMove(24.5, 0.5, 90, 2);
    pidTurnAbs(0, 2, 0.7);
   
    moveForSec(60, false, 0.6);

    score_on = true;
    moveForSec(50, false, 1.8); //FIRST SCORE DONE
    score_on = false;
    imu.set_heading(0);
    jeminloadert();

    setintakespd(-100);
    setintake2spd(-0);
    pidGyro(33, -4, 2, 80, 3);
    moveForSec(55, true, 1.8);
    pidGyro(-40, 6, 2);


    score_on = true;
    moveForSec(30, false, 1.8); //SECOND SCORE DONE
    score_on = false;
    imu.set_heading(0);
    jeminloadert();
    setintakespd(0);
    setintake2spd(0);


    
    //TO CLEAR PARK
    //pidMoveold(7, 0.5, 2, 70); 
    //pidTurnAbs(270, 5, 0.6);
    //pidGyro(96, 270, 5, 75); //91 PATH

    pidmove(19, 1, 2, 100);
    pidTurnAbs(-35, 5, 1);
    pidGyro(40, 295, 2, 100, 11, 25, 2, 100);
    setintakespd(-100);
    setintake2spd(-0);
    moveForSec(60, true, 1.95);


    pidFrontWallGyro(20, 230, 270, 1.8, 80);
    pidTurnAbs(2, 3, 1);

    pidGyro(-27, 2, 1.4, 90);
    score_on = true;
    imu.set_heading(0);
    moveForSec(30, false, 1.2); //PARK SCORE DONE
    score_on = false;
    jeminloadert();

    setintake2spd(0);
    setintakespd(-100);
    pidGyro(34, 5, 2);
    moveForSec(55, true, 1.7);
    //3 MATCHLOAD DONE



    moveForSec(60, false, 0.13);
    jeminloadert();
    pidTurnRel(50, 2, 3, 60);
 
    setintakespddiff(-100, 0);
    setintake2spd(0);


    pidGyro(-99, 5, 6, 90, 30, 100, 5, 100);
    pidwallGyro(36, 0, 0, 1.5, 100);
    
    pidTurnAbs(90, 10, 1);
    pidWallMove(25, 0.5, 270, 2);
    pidTurnAbs(178, 2, 0.7);

    
    moveForSec(50, false, 0.8);
    

    jeminloadert();
    score_on = true;
    moveForSec(50, false, 1.8); //THIRD SCORE DONE
    score_on = false;
    imu.set_heading(180);

    setintakespd(-100);
    setintake2spd(0);
    pidGyro(34, 176, 2);
    moveForSec(55, true, 1.7);
    pidGyro(-40, 186, 2);


    score_on = true;
    moveForSec(30, false, 1.8); //FOURTH SCORE DONE
    score_on = false;
    jeminloadert();

    setintakespd(-100);
    setintake2spd(0);
    pidmove(19, 1, 2, 100);
    pidTurnAbs(145, 5, 1);
    pidGyro(40, 110, 2, 100, 11, 25, 2, 100);
    setintakespd(-100);
    setintake2spd(-0);
    moveForSec(70, true, 0.75);
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
    imu.set_heading(270);
    sort_on = false;
    storing = false;
    jemintaket();
    jeminwingt();

    setintakespd(-100);
    setintake2spd(0);
    // storing = true;
    storing = true;
    moveForSec(30, true, 0.2);
    pidWallMove(29.3, 0.5, 90, 2);
    jeminloadert();
    pidTurnAbs(180, 10, 0.8);
    //pros::delay(200);

    // setintakespd(-100);
    // setintake2spd(-100);
    pidGyro(9, 180, 0.8, 100);
    moveForSec(70, true, 0.7);
    
    moveForSec(100, false, 0.4);
    // pidTurnRel(2.5, 1, 0.2, 70);
    // pidTurnAbs(190, 1, 0.5);
    
    // sort_on = true;
    // moveForSec(70, false, 0.2);
    
    pidGyro(-17, 190, 0.8, 100);
    // moveForSec(70, false, 0.1);
    jeminmech.set_value(true);
    storing = false;
    // setintake2spd(-100);
    score_on = true;
    // moveForSec(30, false, 0.3);
    moveForSec(30, false, 0.3);
    jeminloadert();
    // moveForSec(30, false, 0.3);
    pros::delay(900);
    // jeminmecht();
    score_on = false;
    setintakespd(0);
    setintake2spd(0);

    
    // setintake2spd(-100);
    // setintakespd(-100);

    pidmove(7, 1, 0.5, 100);
    jeminmech.set_value(false);
    setintakespd(-100);
    setintake2spd(0);
    storing = true;
    // pidTurnRel(96, 1, 1.2, 100);
    pidGyro(23, 294, 2, 100, 10);
    

    //pros::delay(1000);
    pidGyro(54.5, 272, 2, 100, 1, 20, 3);
    jeminloadert();



    //pidGyro(5, 272, 3, 70, 1, 25);
    pidTurnAbs(227, 1, 0.5);
    //jeminloadert();
    // pidmove(-20.3, 0.5, 0.3, 100);
    
    moveForSec(100, false, 0.3);
    storing = false;
    setintakespd(-100);
    setintake2spd(80);
    jeminmech.set_value(true);
    moveForSec(30, false, 0.4);

    
    // setintakespd(100);
    // setintake2spd(100);
    // pros::delay(100);
    
    pros::delay(400);
    jeminmecht();
    storing = true;
    setintakespd(-100);
    setintake2spd(0);
    jeminloadert();

    
    //jeminwingt();

    // pidmove(48, 0.5, 3, 100);
    pidGyro(47, 238, 3, 70, 1, 25);
    // pros::delay(100);
    //jeminloadert();
    pidTurnAbs(180, 1, 0.6);

    //jeminloadert();
    //setintake2spd(-100);
    //setintakespd(-100);
    //pidMoveold(10, 1, 1, 100);
    //moveForSec(60, true, 0.2);

    // jeminmecht();
    storing = false;
    jeminmech.set_value(true);
    moveForSec(100, false, 0.5);
    score_on = true; 
    moveForSec(40, false, 0.3); 
    //*/

    //moveForSec(30, 1, 0.3);'




    //RIGHT 3+4
    /*imu.set_heading(270);
    jemintaket();
    jeminwingt();

    setintakespd(-100);
    setintake2spd(-100);
    // storing = true;
    pidWallMove(25.5, 0.5, 90, 2);
    jeminloadert();
    pidTurnAbs(180, 1, 0.8);
    //pros::delay(200);

    setintakespd(-100);
    setintake2spd(-100);
    pidMoveold(10, 1, 1, 70);
    moveForSec(65, true, 0.9);
    
    moveForSec(70, false, 0.5);
    // pidTurnRel(2, 1, 0.2, 70);
    // pidTurnAbs(190, 1, 0.5);
    moveForSec(70, false, 0.4);
    // jeminmecht();
    //moveForSec(40, false, 0.1);
    // setintakespd(-100);
    // setintake2spd(-100);
    sort_on = true;
    score();
    //moveForSec(10, false, 0.3);
    jeminloadert();
    moveForSec(40, false, 0.3);
    pros::delay(650);
    jeminmecht();

    
    setintake2spd(-100);
    setintakespddiff(-100, 0);

    pidMoveold(10, 1, 1, 100);
    // pidTurnRel(96, 1, 1.2, 100);
    pidGyro(30, 294, 3, 80, 1, 25, 10);
    pidGyro(22, 319, 2, 80, 1, 25, 10);
    setintake2spd(0);

    jemintaket();
    setintakespd(100);
    pros::delay(700);
    setintakespd(0);
    jemintaket();

    jeminwingt();

    pidGyro(-27, 270, 3, 70, 1, 25, 5, 1000);
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
    pidMoveold(20, 0.5, 1, 70);
    //moveForSec(50, true, 0.5);
    
    moveForSec(70, false, 0.5);
    pidTurnRel(-1, 0.5, 0.2, 70);
    // pidTurnAbs(190, 1, 0.5);
    moveForSec(70, false, 0.4);
    jeminmecht();
    //moveForSec(40, false, 0.1);
    setintakespd(-100);
    setintake2spd(-100);
    //moveForSec(10, false, 0.3);
    jeminloadert();
    moveForSec(40, false, 0.4);
    pros::delay(530);
    jeminmecht();

    
    setintake2spd(-100);
    setintakespddiff(-100, -40);

    pidMoveold(9, 0.5, 1, 100);
    // pidTurnRel(96, 1, 1.2, 100);
    pidGyro(40, 57, 5, 50, 1, 25, 10);
    pidTurnAbs(220, 1, 1, 80);
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
    setintakespd(0);
    setintake2spd(0);

    sort_on = false;
    side = false; //false is red, true is blue
    


    front_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    middle_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    front_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    middle_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    const float Joystick_LowerDeadzone = 7;

    storing = false;


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
            side = !side;

        }

        /*if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            jeminparktoggle = !jeminparktoggle;
            jeminpark.set_value(jeminparktoggle);
        }*/
        

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && is_sorting == false)
        {
            // setintakespd(-100);
            // setintake2spd(-100);
            sort_on = true;
            score_on = true;
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && is_sorting == false)
        {
            // if (jemintaketoggle == false){
            //     setintakespd(55);
            //     setintake2spd(55);
            // }
            // else {
            //     setintakespd(100);
            //     setintake2spd(100);
            // }
            sort_on = false;
            storing = true;
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && is_sorting == false)
        {
            sort_on = false;
            midscore();
        }

        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            sort_on = false;
            lowscore();
            // storing = true;
        }
        else 
        {
            if (!is_sorting) {
                setintakespd(0);
                setintake2spd(0);
                jeminmech.set_value(false);
                //jemintake.set_value(true);
            }
            sort_on = false;
            storing = false;
            score_on = false;
        }

        pros::c::delay(25);
    }
}
