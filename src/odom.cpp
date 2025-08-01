#include <cmath>
#include <iostream>
#include "pros/screen.h"
#include "robot_config.h"
#include "basic_functions.h"
#include "PID.h"

using namespace pros;

constexpr double start_heading = 90;

double x = 0;
double y = 0;
double gpsHeading;
static float lastForwardPos = 0;
static float lastHeading = imu.get_heading();
static float lastPitch = imu.get_pitch();
static float currForwardPos, currHeading;

void GPStracking()
{
    int line_number = 1;
    vertical_encoder.set_position(0);

    while (true)
    {
        // double heading = std::fmod(360 - imu.get_heading() + start_heading, 360);
        // double dis_travY = vertical_encoder.get_position() * 0.0002399827721;
        // // double dis_travX = horizontal_encoder.get_position() * 0.0002399827721;
        // double deltaposY = dis_travY - prevdisY;
        // // double deltaposX = dis_travX - prevdisX;

        // // x += deltaposX * std::cos(heading * (M_PI / 180)) - deltaposY * std::sin(heading * (M_PI / 180));
        // // y += deltaposX * std::sin(heading * (M_PI / 180)) + deltaposY * std::cos(heading * (M_PI / 180));

        // x += deltaposY * std::sin(heading * (M_PI / 180));
        // y += deltaposY * std::cos(heading * (M_PI / 180));

        currForwardPos = vertical_encoder.get_position() * 0.0002399827721;
        currHeading = imu.get_heading();
        lastPitch = imu.get_pitch();

        float factor = cos(deg2rad(lastPitch));
        x += factor*(currForwardPos - lastForwardPos) * sin(deg2rad(currHeading));
        y += factor*(currForwardPos - lastForwardPos) * cos(deg2rad(currHeading));
        gpsHeading = currHeading;
        lastForwardPos = currForwardPos;

        c::screen_print(E_TEXT_MEDIUM, line_number++, "Xerror: %f, %f", x, y);
        std::cout << x << " " << y << std::endl;
        delay(10);
    }
}

double getX() 
{
    return x;
}

double getY()
{
    return y;
}

void setX(double X)
{
    x = X;
}

void setY(double Y)
{
    y = Y;
}

void GPSmove(float desx, float desy)
{
    float dx = desx - x;
    float dy = desy - y;
    float movedis = std::sqrt(dx * dx + dy * dy);
    float turndis = rad2deg(std::atan2(dy, dx));

    pidTurnRel(turndis, 1, 2000);
    pidMoveold(movedis, 1, 1500);
}