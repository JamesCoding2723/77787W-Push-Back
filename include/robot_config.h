
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include <cmath>

extern pros::adi::Motor rightintakem1;
extern pros::adi::Motor rightintakem2;
extern pros::adi::Motor leftintakem;

extern pros::adi::DigitalOut jeminmech;
extern pros::adi::DigitalOut jeminloader;
extern pros::adi::DigitalOut jemintake;
extern pros::adi::DigitalOut jeminwing;
extern pros::adi::DigitalOut jeminchop;
extern pros::adi::DigitalOut jeminpark;


extern pros::Controller master;

extern pros::adi::Motor front_left_motor;   // front left motor -13
extern pros::adi::Motor middle_left_motor;  // middle left motorv -12
extern pros::Motor back_left_motor;    // back left motor -11
extern pros::Motor front_right_motor;  // front right motor 15
extern pros::Motor middle_right_motor; // middle right motor 17
extern pros::Motor back_right_motor;   // back right motor 16

extern pros::Rotation vertical_encoder;

// left group


// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);
// lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0.75);

extern pros::Imu imu;

/*pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
pros::ADIEncoder vertical_encoder('C', 'D', true);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel*/
// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
/*extern lemlib::OdomSensors sensors;

// lateral PID controller
extern lemlib::ControllerSettings lateral_controller;

// angular PID controller
extern lemlib::ControllerSettings angular_controller;

extern lemlib::Chassis chassis;*/
