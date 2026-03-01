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
#include "auton.h"


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
    pros::Task mmidscore(midscore);

    //pros::Task Lowgoal(lowscore);
    top_color_sensor.set_led_pwm(100);
    //pros::Task GPS(GPStracking);

    // pros::lcd::clear();
    // pros::lcd::print(5, "IUESHIUSEFIHUWER");

    //master.clear();

    //jeminmech.set_value(true);
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
    jeminwing.set_value(true);

    //auto_skills();

    //Right_4ball();

    //Left_4ball();

    //Right_3_4();

    //Left_3_4();

    SAWP();

    //mid_SAWP();

    //right_7();

    //left_7();


    //AUTO SKILLS
    /*imu.set_heading(180);
    jemintaket();//FIRST HALF
    jeminmecht();
    jeminwingt();


    setintakespd(-100);
    setintake2spd(0);
    pros::delay(500);
    moveForSec(70, true, 0.1);
    moveForSec(60, false, 0.1);
    moveForSec(70, true, 0.1);
    moveForSec(60, false, 0.1);
    pros::delay(700);
    moveleft(70);
    moveright(65);
    pros::delay(600);
    //moveForSec(70, true, 0.6);
    moveForSec(60, false, 0.1);
    moveForSec(70, true, 0.1);
    moveForSec(60, false, 0.1);
    moveForSec(70, true, 0.1);
    pros::delay(400);
    moveForSec(30, true, 0.3);
    moveForSec(60, false, 0.8);
    setintakespd(0);
    setintake2spd(-5);


    pidFrontWallGyro(25, 180, 0, 0.7, 60);
    pidTurnAbs(47, 10, 1);
    setintakespddiff(-100, 0);
    pidGyro(22, 47, 2, 100);


    pidGyro(22, 315, 1.5, 70, 1, 25, 1, 400);

    moveForSec(30, true, 0.35);
    jemintaket();
    setintakespddiff(90, 80);
    setintake2spd(25);
    pros::delay(500);
    setintakespddiff(70, 56);
    pros::delay(500);
    setintakespddiff(-65, -65);
    pros::delay(100);
    setintakespddiff(70, 56);
    pros::delay(200);
    setintakespddiff(50, 50);
    pros::delay(1800);

    //jeminmecht();
    pidwallGyro(24.5, 310, 90, 3);

    jeminloadert();
    jemintaket();
    pidTurnAbs(182, 1, 1);


    setintakespd(-100);
    setintake2spd(-5);
    pidGyro(19, 180, 1);
    moveForSec(55, true, 0.5);
    moveForSec(10, true, 1.5);
    //DONE MATCHLOAD

    moveForSec(60, false, 0.1);
    jeminloadert();
    pidTurnRel(50, 2, 3, 60);
    setintakespddiff(-100, 0);
    setintake2spd(0);

    pidGyro(-99, 180, 5.3, 90, 30, 100, 2, 300);
    pidwallGyro(37, 180, 180, 1, 100, 3);
    
    pidTurnAbs(270, 10, 0.8);
    pidWallMove(24.5, 0.5, 90, 2);
    pidTurnAbs(0, 2, 0.7);
   
    moveForSec(60, false, 0.6);

    skillscore();
    jeminloadert();
    moveForSec(60, false, 1.6); //FIRST SCORE DONE
    imu.set_heading(0);

    setintakespd(-100);
    setintake2spd(-5);
    pidGyro(33, -4, 2, 80, 3);
    moveForSec(55, true, 1.8);
    pidGyro(-40, 6, 2);

    skillscore();
    moveForSec(30, false, 1.6); //SECOND SCORE DONE
    imu.set_heading(0);
    jeminloadert();
    setintakespd(0);
    setintake2spd(0);


    
    //TO CLEAR PARK
    //pidMoveold(7, 0.5, 2, 70); 
    //pidTurnAbs(270, 5, 0.6);
    //pidGyro(96, 270, 5, 75); //91 PATH

    //imu.set_heading(0);

    pidmove(19, 1, 2, 100);
    pidTurnAbs(-35, 5, 1);
    pidGyro(40, 295, 2, 100, 11, 25, 2, 100);
    setintakespd(-100);
    setintake2spd(-5);
    moveForSec(55, true, 2.1);  // 60, 1.95


    pidFrontWallGyro(20, 235, 270, 1.8, 80);
    pidTurnAbs(2, 3, 1);

    pidGyro(-27, 2, 1.4, 90);
    skillscore();
    imu.set_heading(0);
    jeminloadert();
    moveForSec(30, false, 0.5); //PARK SCORE DONE

    setintake2spd(0);
    setintakespd(-100);
    pidGyro(34, 5, 2);
    moveForSec(45, true, 2.2);
    //3 MATCHLOAD DONE



    moveForSec(60, false, 0.13);
    jeminloadert();
    pidTurnRel(50, 2, 3, 60);
 
    setintakespddiff(-100, 0);
    setintake2spd(0);


    pidGyro(-99, 5, 6, 90, 30, 100, 5, 100);
    pidwallGyro(37, 0, 0, 0.5, 100, 3);
    
    pidTurnAbs(90, 10, 1);
    pidWallMove(24.5, 0.5, 270, 2);
    pidTurnAbs(178, 2, 0.7);

    
    moveForSec(50, false, 0.8);
    

    jeminloadert();
    skillscore();
    moveForSec(50, false, 1.8); //THIRD SCORE DONE
    imu.set_heading(180);

    setintakespd(-100);
    setintake2spd(-5);
    pidGyro(34, 176, 2);
    moveForSec(55, true, 1.7);
    pidGyro(-40, 186, 2);


    skillscore();
    jeminloadert();
    moveForSec(30, false, 1.8); //FOURTH SCORE DONE

    setintakespd(-100);
    setintake2spd(0);
    pidmove(19, 1, 2, 100);
    pidTurnAbs(145, 5, 1);
    pidGyro(40, 110, 2, 100, 11, 25, 2, 100);
    setintakespd(-100);
    setintake2spd(-5);
    moveForSec(70, true, 0.75);
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



    //RIGHT 3+4
    /*imu.set_heading(270);
    sort_on = false;
    storing = false;
    jemintaket();

    setintakespd(-100);
    setintake2spd(0);
    pidWallMove(27.8, 0.5, 90, 2.5);
    jeminloadert();
    pidTurnAbs(180, 10, 0.8);

    pidGyro(10, 182, 0.8, 100);
    moveForSec(50, true, 0.9);
    
    //moveForSec(100, false, 0.35);

    pidGyro(-34, 185, 1.5, 100);
    jeminmech.set_value(true);
    skillscore();

    moveForSec(50, false, 1.3);
    jeminloadert();
    setintakespd(-100);
    setintake2spd(0);

    
    setintake2spd(-100);
    setintakespddiff(-100, 0);

    pidMoveold(15, 1, 1, 100); //13
    // pidTurnRel(96, 1, 1.2, 100);
    pidGyro(30, 310, 3, 70, 1, 25, 5);
    pidGyro(22, 310, 2, 80, 1, 25, 10);
    setintake2spd(0);

    jemintaket();
    setintakespd(100);
    moveForSec(30, true, 0.7);
    setintakespd(0);
    //jemintaket();

    jeminwingt();

    pidGyro(-22, 270, 3, 70, 1, 25, 5, 1000);
    pidGyro(-13, 10, 1.5, 70, 1, 25, 2, 500);

    jeminwingt();
    jemintaket();
    pros::delay(200);
    moveForSec(70, true, 0.25);
    pidTurnRel(-40, 1, 1, 100);
    //*/


    //LEFT 3+4
    /*imu.set_heading(90);
    sort_on = false;
    storing = false;
    jemintaket();

    setintakespd(-100);
    setintake2spd(0);
    pidWallMove(27.8, 0.5, 270, 2.5);
    jeminloadert();
    pidTurnAbs(180, 10, 0.8);

    pidGyro(10, 178, 0.8, 100);
    moveForSec(50, true, 0.9);
    
    //moveForSec(100, false, 0.35);

    pidGyro(-34, 175, 1.5, 100);
    jeminmech.set_value(true);
    skillscore();

    moveForSec(50, false, 1.3);
    jeminloadert();
    setintakespd(-100);
    setintake2spd(0);

    
    setintake2spd(-100);
    setintakespddiff(-100, 0);

    pidMoveold(15, 1, 1, 100);
    pidGyro(41, 52, 2, 80, 1, 25, 15);
    jeminloadert();
    moveForSec(30, true, 0.4);
    pidTurnAbs(221, 1, 0.8, 80);
    moveForSec(60, false, 0.75);

    setintakespd(100);
    setintake2spd(100);
    pros::delay(50);
    setintakespd(-100);
    setintake2spd(40);
    pros::delay(1000);
    setintake2spd(0);
    setintakespd(0);
    jeminloadert();
    moveForSec(50, true, 0.2);


    pidGyro(29, 200, 2, 80, 1, 25, 5, 1000);
    pidGyro(4, 170, 1.5, 70, 1, 25, 1, 100);
    pros::delay(200);

    jeminwingt();
    pros::delay(200);
    moveForSec(70, false, 0.5);
    pidTurnRel(40, 1, 1, 100);
    //*/


    //BETTER SOLO AWP
    /*imu.set_heading(270);
    sort_on = false;
    storing = false;
    jemintaket();
    jeminwingt();

    setintakespd(-100);
    setintake2spd(-5);

    // storing = true;
    storing = true;
    moveForSec(30, true, 0.2);
    pidwallGyro(27, 270, 90, 2.5, 80);
    jeminloadert();
    pidTurnAbs(180, 10, 0.8);
    //pros::delay(200);

    // setintakespd(-100);
    // setintake2spd(-100);
    pidGyro(10, 180, 0.8, 100);
    moveForSec(60, true, 0.71);

    
    pidGyro(-38, 188, 2, 100);
    // moveForSec(70, false, 0.1);
    jeminmech.set_value(true);
    storing = false;
    // score_on = true;
    setintake2spd(-100);
    setintakespd(-100);

    moveForSec(50, false, 0.8);
    jeminloadert();
    //moveForSec(50, false, 0.7);
    score_on = false;
    setintakespd(-100);
    setintake2spd(0);



    pidGyro(33, 310, 2, 100, 1, 25, 5, 200);
    //pidmove(11, 1, 0.5, 100);

    storing = true;
    jeminmech.set_value(false);
    

    //pros::delay(1000);
    pidGyro(72, 270, 2, 100, 1, 20, 3);
    jeminloadert();

    // pidmove(48, 0.5, 3, 100);
    pidGyro(40, 230, 2, 70, 5, 25, 1);
    // pros::delay(1);

    pidFrontWallGyro(21.5, 240, 270, 0.8, 100);


    pidTurnAbs(180, 3, 0.6);


    pidGyro(-20, 180, 1, 100);
    jeminmech.set_value(true);
    setintake2spd(-100);
    setintakespd(-100);
    moveForSec(60, false, 0.8);
    setintake2spd(0);

    pidGyro(34, 176, 1, 100);
    moveForSec(60, true, 1);
    jeminloadert();
    setintakespd(-100);
    setintake2spd(0);

    pidmove(-17, 1, 0.5, 100);

    storing = true;
    jeminmech.set_value(false);
    pidGyro(-47, 232, 2, 100, 10);
    jeminmech.set_value(true);
    moveForSec(40, false, 0.3);
    storing = false;
    setintakespddiff(-100, -40);
    setintake2spd(60);
    moveForSec(30, false, 0.5);

    // storing = false;
    // jeminmech.set_value(true);
    // moveForSec(100, false, 0.5);
    // score_on = true; 
    // moveForSec(40, false, 0.3); 
    //*/



    //MIDDLE SOLO AWP
    /*imu.set_heading(270);
    sort_on = false;
    storing = false;
    jemintaket();

    setintakespd(-100);
    setintake2spd(0);
    pidWallMove(27.8, 0.5, 90, 2.5);
    jeminloadert();
    pidTurnAbs(180, 10, 0.8);

    pidGyro(10, 182, 0.8, 100);
    moveForSec(50, true, 0.9);
    
    //moveForSec(100, false, 0.35);

    pidGyro(-34, 185, 1.5, 100);
    jeminmech.set_value(true);
    setintakespd(-100);
    setintake2spd(-100);

    moveForSec(50, false, 1.6);
    jeminloadert();
    setintakespd(-100);
    setintake2spd(0);

    
    setintake2spd(-100);
    setintakespddiff(-100, 0);

    pidMoveold(13, 1, 1, 100);
    // pidTurnRel(96, 1, 1.2, 100);
    pidGyro(30, 310, 3, 70, 1, 25, 5);
    pidGyro(22, 310, 2, 80, 1, 25, 10);
    setintake2spd(0);

    jemintaket();
    setintakespd(70);
    moveForSec(30, true, 1.2);
    jemintaket();

    setintakespd(-100);
    pidmove(-23, 1, 2);
    pidGyro(46, 267, 3, 70, 5, 25, 1, 15);
    jeminloadert();
    moveForSec(30, true, 0.2);
    pidGyro(-23, 225, 2);

    //setintakespd(-100);
    setintake2spd(50);
    moveForSec(30, false, 0.3);
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
    bool mid_scoring = false;

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

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && is_sorting == false)
        {
            // setintakespd(-100);
            // setintake2spd(-100);
            sort_on = true;
            score_on = true;
            mid_scoring = false;
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && is_sorting == false)
        {
            sort_on = false;
            storing = true;
            mid_scoring = false;
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && is_sorting == false)
        {
            sort_on = false;
            //skills
            /*mid_scoring = true;
            jeminmech.set_value(true);
            setintakespd(75);
            setintake2spd(75);
            pros::delay(75);
            if (middistance.get() > 100) {
                setintakespd(-50);
                setintake2spd(35);
            }
            else{
                setintake2spd(35);
                pros::delay(250);
                setintakespd(-35);
            }//*/
            midscore();
        }

        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            sort_on = false;
            lowscore();
            mid_scoring = false;
        }
        else 
        {
            if (!is_sorting && !mid_scoring) {
                setintakespd(0);
                setintake2spd(0);
                jeminmech.set_value(false);
                //jemintake.set_value(true);
            }
            sort_on = false;
            storing = false;
            score_on = false;
            lowgoal_on = false;
        }

        pros::c::delay(25);
    }
}
