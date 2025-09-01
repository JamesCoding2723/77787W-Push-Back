#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cmath>
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

int intakespd = 0;
int intakerspd = 0;

void setintakespd(float spd)
{
    intakespd = spd;
}

void setintakerspd(float spd) 
{
    intakerspd = spd;
}

void intake()
{
    while (true)
    {
        if (intakespd == 0 && intakerspd == 0)
        {
            rightintakem.brake();
            leftintakem2.brake();
            leftintakem3.brake();
            leftintakem1.brake();
            
        }
        else if (intakerspd != 0)
        {
            rightintakem.move((int)130 * intakerspd);
            leftintakem1.move(int(130) * intakerspd);
            leftintakem2.move((int)-130 * intakerspd);
            leftintakem3.move((int)130 * intakerspd);
        }
        else
        {
            rightintakem.move((int)130 * intakespd);
            leftintakem1.move(int(130) * intakespd);
            leftintakem2.move((int)-130 * intakespd);
            leftintakem3.move((int)-130 * intakespd);
        }
    }
}

/*void intaker() {
    if (intakerspd == 0)
    {
        rightintakem.brake();
        leftintakem2.brake();
        leftintakem3.brake();
        leftintakem1.brake();
    }
    else
    {
        rightintakem.move((int)130 * intakespd);
        leftintakem1.move(int(130) * intakespd);
        leftintakem2.move((int)-130 * intakespd);
        leftintakem3.move((int)-130 * intakespd);
    }
}*/

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
    front_left_motor.brake();
    middle_right_motor.brake();
    back_right_motor.brake();
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

bool jeminrooftoggle = false;

void jeminrooft()
{
    // bool clamptoggle = false;
    jeminrooftoggle = !jeminrooftoggle;
    jeminroof.set_value(jeminrooftoggle);
}

void moveforward(float dis, bool dir, float tm)
{
    chassis.setPose(0, 0, 0);
    if (dir == false)
        chassis.moveToPoint(0, -dis, tm, {.forwards = false});
    else if (dir == true)
        chassis.moveToPoint(0, dis, tm);
}

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