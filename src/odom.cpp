#include <cmath>
#include "robot_config.cpp"
#include "basic_functions.h"
#include "PID.h"


constexpr double start_heading = 90;

double x = 0;
double y = 0;

double tracking(){
    vertical_encoder.set_position(0);   

    double prevdisX = 0;
    double prevdisY = 0;

    while (true) {
        double heading = std::fmod(360 - imu.get_heading() + start_heading, 360);
        double dis_travY = vertical_encoder.get_position() * 0.0002399827721;
        //double dis_travX = horizontal_encoder.get_position() * 0.0002399827721;
        double deltaposY = dis_travY - prevdisY;
        //double deltaposX = dis_travX - prevdisX;

        // x += deltaposX * std::cos(heading * (M_PI / 180)) - deltaposY * std::sin(heading * (M_PI / 180));
        // y += deltaposX * std::sin(heading * (M_PI / 180)) + deltaposY * std::cos(heading * (M_PI / 180));
        
        x += deltaposY * std::cos(heading * (M_PI / 180));
        y += deltaposY * std::sin(heading * (M_PI / 180));
        
        return x;
        return y;

        vex::this_thread::sleep_for(10);
    }
}