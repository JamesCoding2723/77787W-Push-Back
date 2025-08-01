#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include <cmath>

using namespace pros;

extern Motor backintakem;
extern Motor intakem;

extern ADIDigitalOut blocker; 	
                    
extern Controller master;

extern Motor front_left_motor; // front left motor -13
extern Motor middle_left_motor; // middle left motorv -12
extern Motor back_left_motor; // back left motor -11
extern Motor front_right_motor; // front right motor 15
extern Motor middle_right_motor; // middle right motor 17
extern Motor back_right_motor; // back right motor 16

extern Rotation vertical_encoder;

// left group
extern MotorGroup left_motor_group;
// right group
extern MotorGroup right_motor_group;

extern lemlib::Drivetrain drivetrain;

//lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);
//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0.75);

extern Imu imu;

/*pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
pros::ADIEncoder vertical_encoder('C', 'D', true);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel*/
//lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
extern lemlib::OdomSensors sensors;

// lateral PID controller
extern lemlib::ControllerSettings lateral_controller;

// angular PID controller
extern lemlib::ControllerSettings angular_controller;

extern lemlib::Chassis chassis;

