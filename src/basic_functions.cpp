
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/optical.h"
#include "pros/rtos.hpp"
#include <cmath>
#include "robot_config.h"
#include "basic_functions.h"
#include "pros/screen.h"

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

bool midgoal;

void intake()
{
    while (true)
    {
        if (intakespd1 == 0 && is_sorting == false)
        {
            rightintakem1.brake();  
        } 
        else 
        {
            rightintakem1.move((int)1.27 * intakespd1);
        }
        if (intakespd2 == 0 && is_sorting == false)
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
        if (intake2spd == 0 && is_sorting == false)
        {
            leftintakem.brake();
        }
        else
        {

            if (jemintaketoggle == false) {
                pros::delay(400);
                leftintakem.move((int)1.27 * 50);
                pros::delay(100);
               leftintakem.move((int)1.27 * intake2spd);
            }

            else{
                leftintakem.move((int)1.27 * intake2spd);
            }


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
    front_left_motor.set_zero_position(0);
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

void setjeminparallel(float _time, bool _active)
{
    float delay_time = _time;
    bool delay_active = _active;
}

float delay_time;
bool delay_active;

void jeminloaderd()
{
    while(delay_active == true)
    {
        pros::delay(delay_time);
        jeminloadertoggle = !jeminloadertoggle;
        jeminloader.set_value(jeminloadertoggle);
    }
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

bool jeminchoptoggle = false;

void jeminchopt()
{
    // bool clamptoggle = false;
    jeminchoptoggle = !jeminchoptoggle;
    jeminchop.set_value(jeminchoptoggle);
}

bool jeminparktoggle = false;

void jeminparkt()
{
    // bool clamptoggle = false;
    jeminparktoggle = !jeminparktoggle;
    jeminpark.set_value(jeminparktoggle);
}

float wall_heading;

void setwall_heading(float _input)
{
    wall_heading = _input;
}

float d0 = 6.5;
double wallpos;

double getwallpos(float wall_heading)
{
        double theta = imu.get_heading();
        theta = fabs(theta-360);

        theta -= wall_heading;

        if (theta > 180) theta -= 360;
        if (theta < -180) theta += 360;

        //if (fabs(theta) > 5) theta = fabs(theta) - 5;
        //else theta = 0;

        return wallpos = ((distance_sensor.get())/25.5 + d0) * cos(deg2rad(theta));
}

float df0 = 3.5;
double frontwallpos;

double getfrontwallpos(float wall_heading)
{
    while(true)
    {
        double theta = imu.get_heading();
        theta = fabs(theta-360);

        theta -= wall_heading;

        theta += 180;

        if (theta > 180) theta -= 360;
        if (theta < -180) theta += 360;

        //if (fabs(theta) > 5) theta = fabs(theta) - 5;
        //else theta = 0;

        return frontwallpos = ((frontdistance.get())/25.5 + df0) * cos(deg2rad(theta));
    }
}

float deg2rad(float _input)
{
    return M_PI / 180 * _input;
}

float rad2deg(float _input)
{
    return _input * 180 / M_PI;
}

void imu_display_task(void*) {
  while (true) {
        pros::c::screen_print(pros::E_TEXT_MEDIUM, 1, "wallpos: %f, %f, %f", wallpos, frontwallpos, imu.get_heading());
        pros::c::screen_print(pros::E_TEXT_MEDIUM, 2, "color sort: %f, %f, %f", top_color_sensor.get_rgb().red, top_color_sensor.get_rgb().green, top_color_sensor.get_rgb().blue);
        pros::c::screen_print(pros::E_TEXT_MEDIUM, 3, "scoring val: %d, %d", middistance.get(), top_color_sensor.get_proximity());

    pros::delay(100);   // update ~10 times per second
  }
}

bool side = false; //false = red out; true = blue out
bool sort_on = false;
bool is_sorting = false;
bool lastsort = false;

void color_sort() {
    while (true) {
        if (lastsort) {
            lastsort = false;
            pros::delay(150);
            continue;
        }
        if (sort_on) {
            pros::c::optical_rgb_s_t rgb = top_color_sensor.get_rgb();
            if (!side && (rgb.red > 200 && rgb.red > rgb.green && rgb.red > rgb.blue)) {
                jeminmech.set_value(true);
                is_sorting = true;
                setintakespddiff(-100, -100);
                setintake2spd(100);
                pros::delay(150);
                setintakespddiff(intakespd1, intakespd2);
                setintake2spd(intake2spd);
                is_sorting = false;
                lastsort = true;
            } 
            else if (side && (rgb.blue > 200 && rgb.blue > rgb.red && rgb.blue > rgb.green)) {
                jeminmech.set_value(true);
                is_sorting = true;
                setintakespddiff(-100, -100);
                setintake2spd(100); 
                pros::delay(150);
                setintakespddiff(intakespd1, intakespd2);
                setintake2spd(intake2spd);
                is_sorting = false;
                lastsort = true;
            }
        }
        pros::delay(10);
    }
}

bool storing = true;
bool prev = false;
void store() {
    while (true) {
        if (storing && !prev) {
            jeminmech.set_value(false);
            setintakespd(-100);
            // if (mid_color_sensor.get_proximity() > 150) { 
            setintake2spd(0);// hold block
            // }
            prev = true;
        } else {
            // leftintakem.move_voltage(0);
            if (!storing) prev = false;
        }
        pros::delay(20);
    }
}

bool lastscore = false;
bool score_on = false;

void score() {
    while(true) {
        if (lastscore) {
            lastscore = false;
            pros::delay(120);
            continue;
        }
        if (score_on && !is_sorting) {
            storing = false;
            jeminmech.set_value(true);
            if (middistance.get() < 125) {
                setintakespd(10);
                setintake2spd(-100);
                pros::delay(120);
                lastscore = true;
            }
            else {
                setintakespd(-100);
                setintake2spd(-100);
                lastscore = true;
            }
        }
    }
}

void midscore() {
    storing = false;
    jeminmech.set_value(true);
    setintakespd(-100);
    setintake2spd(40);
}

void lowscore() {
    storing = false;
    jeminmech.set_value(true);

    if (jemintaketoggle == false) {
        setintakespd(60);
        setintake2spd(-10);
    }
    else {
        setintakespd(100);
        setintake2spd(-10);
    }
}

void skillscore() {
    storing = false;
    setintakespd(10);
    setintake2spd(-100);
    moveForSec(50, false, 0.3);
    setintakespd(-100);
    setintake2spd(-100);
}