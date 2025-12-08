#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cmath>
#include "pros/vision.h"
#include "robot_config.h"

int sign(float _input)
{
    if (_input > 0)
        return 1;
    else if (_input < 0)
        return -1;
    else
        return 0;
}

int intakespd1 = 0;
int intakespd2 = 0;

int intake2spd = 0;

void setintakespd(float spd)
{
    intakespd1 = spd;
    intakespd2 = spd;
}

void setintakespddiff(float spd1, float spd2)
{
    intakespd1 = spd1;
    intakespd2 = spd2;
}

void setintake2spd(float spd) 
{
    intake2spd = spd;
}

void intake()
{
    while (true)
    {
        if (intakespd1 == 0)
        {
            rightintakem1.brake();  
        } 
        else 
        {
            rightintakem1.move((int)1.27 * intakespd1);
        }
        if (intakespd2 == 0)
        {
            rightintakem2.brake();  
        }
        else
        {
            rightintakem2.move((int)1.27 * intakespd2);
        }
    }
}

void intake2() {
    while (true)
    {
        if (intake2spd == 0)
        {
            leftintakem.brake();
        }
        else
        {
            leftintakem.move((int)1.27 * intake2spd);

        }
    }
}

void moveleft(float spd)
{
    if (fabs(spd) > 100)
        spd = sign(spd) * 100;
    left_motor_group.move(spd * (int)(127 / 100));
}

void moveright(float spd)
{
    if (fabs(spd) > 100)
        spd = sign(spd) * 100;
    right_motor_group.move(spd * (int)(127 / 100));
}

void move(float spd)
{
    if (fabs(spd) > 100)
        spd = sign(spd) * 100;
    moveright(spd);
    moveleft(spd);
}

void turn(float spd)
{
    if (fabs(spd) > 100)
        spd = sign(spd) * 100;
    moveright(-spd); 
    moveleft(spd);
}

void stop()
{
    back_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    middle_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    front_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    front_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    middle_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    back_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    front_left_motor.brake();
    middle_left_motor.brake();
    back_left_motor.brake();
    front_right_motor.brake();
    middle_right_motor.brake();
    back_right_motor.brake();
}

double motorpos(){
    return ((front_left_motor.get_position() + front_right_motor.get_position()) / 2);
}

void resetmotorpos(){
    front_left_motor.set_value(0);
    middle_left_motor.set_zero_position(0);
    back_left_motor.set_zero_position(0);
    front_right_motor.set_zero_position(0);
    middle_right_motor.set_zero_position(0);
    back_right_motor.set_zero_position(0);
}

void moveDis(float spd, float dis)
{
    if (fabs(spd) > 100)
        spd = sign(spd) * 100;
    while (std::abs(vertical_encoder.get_position()) < dis)
    {
        move(sign(spd) * spd);
    }
    stop();
}

void moveForSec(float spd, bool dir, float secs)
{
    if (fabs(spd) > 100)
        spd = sign(spd) * 100;
    if (dir == true)
    {
        while (true)
        {
            move(spd);
            pros::delay(secs * 1000);
            stop();
            break;
        }
    }
    else if (dir == false)
    {
        while (true)
        {
            move(-spd);
            pros::delay(secs * 1000);
            stop();
            break;
        }
    }
}

bool jeminmechtoggle = false;

void jeminmecht()
{
    // bool clamptoggle = false;
    jeminmechtoggle = !jeminmechtoggle;
    jeminmech.set_value(jeminmechtoggle);
}

bool jeminloadertoggle = false;

void jeminloadert()
{
    // bool clamptoggle = false;
    jeminloadertoggle = !jeminloadertoggle;
    jeminloader.set_value(jeminloadertoggle);
}

bool jemintaketoggle = false;

void jemintaket()
{
    // bool clamptoggle = false;
    jemintaketoggle = !jemintaketoggle;
    jemintake.set_value(jemintaketoggle);
}

bool jeminwingtoggle = false;

void jeminwingt()
{
    // bool clamptoggle = false;
    jeminwingtoggle = !jeminwingtoggle;
    jeminwing.set_value(jeminwingtoggle);
}


<<<<<<< HEAD

=======
void moveforward(float dis, bool dir, float tm)
{
    chassis.setPose(0, 0, 0);
    if (dir == false)
        chassis.moveToPoint(0, -dis, tm, {.forwards = false});
    else if (dir == true)
        chassis.moveToPoint(0, dis, tm);
}
>>>>>>> parent of 58a9a06 (AUTO SKILLS PLUS COMP)

void wait(float time)
{
    pros::c::delay(time);
}

float deg2rad(float _input)
{
    return M_PI / 180 * _input;
}

float rad2deg(float _input)
{
    return _input * 180 / M_PI;
}