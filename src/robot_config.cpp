#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include <cmath>
#include "robot_config.h"

// ROBOT CONFIG  //  PID CONFIG
#pragma region

pros::Motor backintakem(21, pros::E_MOTOR_GEAR_BLUE, 1);
pros::Motor intakem(17, pros::E_MOTOR_GEAR_BLUE, 1);

pros::ADIDigitalOut blocker('A', false);

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor front_left_motor(-3, pros::E_MOTOR_GEAR_BLUE);   // front left motor -13
pros::Motor middle_left_motor(-2, pros::E_MOTOR_GEAR_BLUE); // middle left motorv -12
pros::Motor back_left_motor(-1, pros::E_MOTOR_GEAR_BLUE);    // back left motor -11
pros::Motor front_right_motor(8, pros::E_MOTOR_GEAR_BLUE); // front right motor 15
pros::Motor middle_right_motor(9, pros::E_MOTOR_GEAR_BLUE); // middle right motor 17
pros::Motor back_right_motor(10, pros::E_MOTOR_GEAR_BLUE); // back right motor 16

pros::Rotation vertical_encoder(6);

// left group
pros::MotorGroup left_motor_group({front_left_motor, middle_left_motor, back_left_motor});
// right group
pros::MotorGroup right_motor_group({front_right_motor, middle_right_motor, back_right_motor});

lemlib::Drivetrain drivetrain(&left_motor_group,          // left group
                              &right_motor_group,         // right group
                              13,                         // track width
                              lemlib::Omniwheel::OLD_325, // using old_2.75
                              450,
                              2 // horizontal drift is 2 (for now)
);

// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);
// lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0.75);

pros::Imu imu(9);

/*pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
pros::ADIEncoder vertical_encoder('C', 'D', true);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel*/
// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr,
                            nullptr, // horizontal tracking wheel
                            nullptr,
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10,  // proportional gain (kP) //10
                                              0,   // integral gain (kI)
                                              3,   // derivative gain (kD)
                                              0,   // anti windup
                                              1,   // small error range, in inches
                                              300, // small error range timeout, in milliseconds
                                              3,   // large error range, in inches
                                              700, // large error range timeout, in milliseconds
                                              0    // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3,    // proportional gain (kP)
                                              0,    // integral gain (kI)
                                              8,    // derivative gain (kD)
                                              0,    // anti windup
                                              1,    // small error range, in degrees
                                              500,  // small error range timeout, in milliseconds
                                              3,    // large error range, in degrees
                                              1300, // large error range timeout, in milliseconds
                                              0     // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors

);

#pragma endregion