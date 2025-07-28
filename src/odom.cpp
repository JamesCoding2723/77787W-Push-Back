#include <cmath>
#include "robot_config.cpp"
#include "basic_functions.h"
#include "PID.h"


constexpr double start_heading = 90;

double x = 0;
double y = 0;

double tracking(){
    vertical_encoder.set_position(0);   

    double prevdis = 0;

    while (true) {
        double heading = std::fmod(360 - imu.get_heading() + start_heading, 360);

        double dis_trav = vertical_encoder.get_position() * 0.0002399827721;
        double deltapos = dis_trav - prevdis;

        



    }
}