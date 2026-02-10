#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.hpp"
#include "pros/optical.hpp"
#include <cmath>

extern pros::Motor rightintakem1;
extern pros::Motor rightintakem2;
extern pros::Motor leftintakem;

extern pros::ADIDigitalOut jeminmech;
extern pros::ADIDigitalOut jeminloader;
extern pros::ADIDigitalOut jemintake;
extern pros::ADIDigitalOut jeminwing;
extern pros::ADIDigitalOut jeminchop;
extern pros::ADIDigitalOut jeminpark;


extern pros::Controller master;

extern pros::Motor front_left_motor;   // front left motor -13
extern pros::Motor middle_left_motor;  // middle left motorv -12
extern pros::Motor back_left_motor;    // back left motor -11
extern pros::Motor front_right_motor;  // front right motor 15
extern pros::Motor middle_right_motor; // middle right motor 17
extern pros::Motor back_right_motor;   // back right motor 16

extern pros::Rotation vertical_encoder;

extern pros::Optical top_color_sensor;

// left group
extern pros::MotorGroup left_motor_group;
// right group
extern pros::MotorGroup right_motor_group;

extern lemlib::Drivetrain drivetrain;

// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);
// lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0.75);

extern pros::Imu imu;
extern pros::Distance distance_sensor;
extern pros::Distance frontdistance;
extern pros::Distance middistance;

/*pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
pros::ADIEncoder vertical_encoder('C', 'D', true);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel*/
// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
extern lemlib::OdomSensors sensors;

// lateral PID controller
//extern lemlib::ControllerSettings lateral_controller;

// angular PID controller
//extern lemlib::ControllerSettings angular_controller;

extern lemlib::Chassis chassis;
