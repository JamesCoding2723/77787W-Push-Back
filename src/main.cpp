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
    //pros::Task mmidscore(midscore);

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

    // Right_3_4();

    Left_3_4();

    // right_7();

    // left_7();

    //Right_4ball();

    //Left_4ball();

    //SAWP();

    //mid_SAWP();

    //one inch
    /*jemintaket();
    moveForSec(30, true, 0.5);*/

    
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

    sort_on = true;
    Msort_on = false;
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
     static bool antijam = false;

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

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            side = !side;
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            Msort_on = !Msort_on;
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
            if (!antijam){
                setintakespd(90);
                setintake2spd(100);
                pros::delay(50);
                midscore();
                antijam = true;
            }
            midscore();
            //mid_scoring = false;
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
            }
            sort_on = false;
            storing = false;
            antijam = false;
            score_on = false;
            lowgoal_on = false;
        }

        pros::c::delay(25);
    }
}
