#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/screen.h"
#include <cmath>
#include "main.h"


//ROBOT CONFIG  //  PID CONFIG
#pragma region

    pros::Motor intakem(21, pros::E_MOTOR_GEAR_BLUE, 1);
    pros::Motor hookm (7, pros::E_MOTOR_GEAR_BLUE, 1);
    pros::Motor wallie (8, pros::E_MOTOR_GEAR_GREEN, 1);

    pros::ADIDigitalOut clamp('A', false); 	
    pros::ADIDigitalOut doinker('C', false); 
    //pros::ADIDigitalOut raise('C', false);
					 
    pros::Controller master (pros::E_CONTROLLER_MASTER);

    pros::Motor front_left_motor(-13, pros::E_MOTOR_GEAR_BLUE); // front left motor -13
    pros::Motor middle_left_motor(-12, pros::E_MOTOR_GEAR_BLUE); // middle left motorv -12
    pros::Motor back_left_motor(-11, pros::E_MOTOR_GEAR_BLUE); // back left motor -11
    pros::Motor front_right_motor(15, pros::E_MOTOR_GEAR_BLUE); // front right motor 15
pros::Motor middle_right_motor(17, pros::E_MOTOR_GEAR_BLUE); // middle right motor 17
pros::Motor back_right_motor(16, pros::E_MOTOR_GEAR_BLUE); // back right motor 16

pros::Rotation horizontal_encoder(-1);
pros::Rotation vertical_encoder(-2);




// left group
pros::MotorGroup left_motor_group({ front_left_motor, middle_left_motor, back_left_motor });
// right group
pros::MotorGroup right_motor_group({ front_right_motor, middle_right_motor, back_right_motor });

lemlib::Drivetrain drivetrain(&left_motor_group, // left group
                              &right_motor_group, // right group
                              13, //track width
                              lemlib::Omniwheel::OLD_275, // using old_2.75
                              450, 
                              2 // horizontal drift is 2 (for now)
);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0.75);

pros::Imu imu(9);

/*pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
pros::ADIEncoder vertical_encoder('C', 'D', true);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel*/
//lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel  
							nullptr,
                            &horizontal_tracking_wheel, // horizontal tracking wheel
							nullptr,
                            &imu // inertial sensor
);



// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP) //10
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              300, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              700, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3,// proportional gain (kP)
                                              0, // integral gain (kI)
                                              8, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in degrees
                                              500, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              1300, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors

);



#pragma endregion

#pragma region
//basic random functions



int sign(float _input) {
    if (_input > 0) return 1;
    else if (_input < 0) return -1;
    else return 0;
}

int hookspd = 0;

void sethookspd(float spd) {
    hookspd = spd;
}

void hook() {
    while(true){
        if (hookspd == 0) hookm.brake();
        else hookm.move((int)130 * hookspd);
    }
}

int intakespd = 0;

void setintakespd(float spd) {
    intakespd = spd;
}

void intake() {
    while(true){
        if (intakespd == 0) intakem.brake();
        else intakem.move((int)130 * intakespd);
    }
}

void moveleft(float spd) {
     if (fabs (spd) > 100) spd = sign(spd) * 100;
    left_motor_group.move(spd*(int)(127/100));
}

void moveright(float spd) {
    if (fabs (spd) > 100) spd = sign(spd) * 100;
    right_motor_group.move(spd*(int)(127/100));
}

void move(float spd) {
    if (fabs (spd) > 100) spd = sign(spd) * 100;
    moveright(spd);
    moveleft(spd);
}

void turn(float spd) {
    if (fabs (spd) > 100) spd = sign(spd) * 100;
    moveright(-spd);
    moveleft(spd);
}



void stop() {
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

/*void moveForMsec(float spd, bool dir, float msecs) {
    int counter = msecs/20;
    int n = 0;
    std::vector<double> left, right;
    double left_position, right_position, diff;
    float left_spd, right_spd;
    double ratio = 0.5;
    left_spd = right_spd = spd;

    if (dir == false) {
        ratio = ratio * (-1);
        left_spd = right_spd = spd * (-1);
    }
    
    while ( (n++) < counter) {
            middle_left_motor.set_zero_position(0);
            back_left_motor.set_zero_position(0);
            middle_right_motor.set_zero_position(0);
            back_right_motor.set_zero_position(0);

            moveleft(left_spd);
            moveright(right_spd);
            pros::delay(10); 
            left = left_motor_group.get_positions();
            right = right_motor_group.get_positions();
            left_position = (left. at(1) + left.at(2))/2;
            right_position = (right. at(1) + right.at(2))/2;
            diff = fabs(left_position - right_position);
            if (right_position > left_position) {
                left_spd = left_spd + diff *1/3 * ratio;
                right_spd = right_spd - diff *2/3 * ratio;
            } else {
                left_spd = left_spd - diff *1/3 * ratio;
                right_spd = right_spd + diff *2/3 * ratio;
            }   
            if (n > 10)
                pros::c::screen_print(pros::E_TEXT_MEDIUM, n-10, "n= %d left: %4f, right: %4f", n, left_spd, right_spd);   
    }
    //stop();
}*/

void moveDis(float spd, float dis) {
    if (fabs (spd) > 100) spd = sign(spd) * 100;
    while(std::abs(vertical_encoder.get_position()) < dis){
        move(sign(spd) * spd);
    }
    stop();
}


void moveForSec(float spd, bool dir, float secs) {
    if (fabs (spd) > 100) spd = sign(spd) * 100;
    if (dir == true) {
        while(true){
            move(spd);
            pros::delay(secs*1000);      
            stop();
            break;
        }
    }
    else if (dir == false) {
        while(true){
            move(-spd);
            pros::delay(secs*1000);
            stop();
            break;
        }
    }
}

bool clamptoggle = false;

void clampmech() {
    //bool clamptoggle = false;
    clamptoggle = !clamptoggle;
    clamp.set_value(clamptoggle);
}

bool doinkertoggle = false;

void doinkermech() {
    //bool doinkertoggle = false;
    doinkertoggle = !doinkertoggle;
    doinker.set_value(doinkertoggle);
}
/*
bool raisetoggle = false; 

void raisemech() {
    raisetoggle = !raisetoggle; 
    raise.set_value(raisetoggle);
}*/



void moveforward(float dis, bool dir, float tm) {
    chassis.setPose(0, 0, 0);
    if (dir == false) chassis.moveToPoint(0, -dis, tm, {.forwards = false});
    else if (dir == true) chassis.moveToPoint(0, dis, tm); 
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}*/

float InchToEncoderunit(float distance){
    return (distance * 360 / 2.75 / 3.1415926) * (4/3); // gear ratio is 4:3
}

/*void pidMoveNew(float target_inch, float move_tolocal_inch){ // MOVE MOVE MOVE
  
    float target = InchToEncoderunit(target_inch);
    float move_tolocal = InchToEncoderunit(move_tolocal_inch);
    float pTol = move_tolocal;
    float dTol = move_tolocal;
    float lastError;
    float kp = 0.9; // for new robot
    float kd = 0.25; // for new robot
    float ki = 0.1; // for new roobot
    float s_error = 0;
    int line_number = 1;
    float speed_ratio = 0.01;

    int n = 0;
    std::vector<double> left, right;
    double left_position, right_position, diff;
    float left_spd, right_spd, spd;
    double diff_ratio = 0.5;

    right_position = left_position = 0;
    left_spd = right_spd = 0;
    front_left_motor.set_zero_position(0);
    front_right_motor.set_zero_position(0);
    
    while(true){
      float error = target - (front_left_motor.get_position() + front_right_motor.get_position())/2 ;
      float P = error * kp;
      float D = (error - lastError) * kd;
      s_error += error;
      s_error = fmin(s_error,100);
      s_error = fmax(s_error,-100);
        // Brain.Screen.printAt(10,20,"error=%f",error);
      if (error * lastError < 0) s_error = 0;
  
      if(fabs(error)<pTol){
        //move_turn(0);
        stop();
        break;
      }
  
      float I = ki * s_error;
      lastError = error;

      left_spd = (P+D+I)*speed_ratio + (left_spd - right_spd);
      right_spd = (P+D+I)*speed_ratio;
      if (right_position > left_position) {
        left_spd = left_spd + diff *1/3 * diff_ratio;
        right_spd = right_spd - diff *2/3 * diff_ratio;
      } else {
        left_spd = left_spd - diff *1/3 * diff_ratio;
        right_spd = right_spd + diff *2/3 * diff_ratio;
      }
      middle_left_motor.set_zero_position(0);
      back_left_motor.set_zero_position(0);
      middle_right_motor.set_zero_position(0);
      back_right_motor.set_zero_position(0);

      moveleft(left_spd);
      moveright(right_spd);
      pros::delay(20); 
      left = left_motor_group.get_positions();
      right = right_motor_group.get_positions();
      left_position = (left. at(1) + left.at(2))/2;
      right_position = (right. at(1) + right.at(2))/2;
      diff = fabs(left_position - right_position);
      
      //pros::c::screen_print(pros::E_TEXT_MEDIUM, n-10, "n= %d left: %4f, right: %4f", n, left_spd, right_spd);   

      //pros::c::screen_print(pros::E_TEXT_MEDIUM, line_number++, "error: %f", error);
    }
}*/

void pidMoveold(float target_inch, float move_tolocal_inch){ // MOVE MOVE MOVE
  
  float target = InchToEncoderunit(target_inch);
  float move_tolocal = InchToEncoderunit(move_tolocal_inch);
  float pTol = move_tolocal;
  float dTol = move_tolocal;
  float lastError;
  float kp = 0.7; // for new robot
  float kd = 0.2; // for new robot
  float ki = 0.0; // for new roobot
  float s_error = 0;
  int line_number = 1;
  float speed_ratio = 0.3, spd;
  front_left_motor.set_zero_position(0);
  front_right_motor.set_zero_position(0);
  int repeat_limit = 2000;
  int repeat = 0;

  while((repeat++) < repeat_limit){
    float error = target - (front_left_motor.get_position() + front_right_motor.get_position())/2 ;
    float P = error * kp;
    float D = (error - lastError) * kd;
    s_error += error;
    s_error = fmin(s_error,100);
    s_error = fmax(s_error,-100);
      // Brain.Screen.printAt(10,20,"error=%f",error);
    if (error * lastError < 0) s_error = 0;

    if(fabs(error)<pTol){
      //move_turn(0);
      stop();
      break;
    }

    float I = ki * s_error;
    lastError = error;
    spd = (P+D+I)*speed_ratio;
    if (std::abs(spd) < 12) spd = sign(spd) *12;
    move(spd);
    pros::c::delay(20);
    //pros::c::screen_print(pros::E_TEXT_MEDIUM, line_number++, "error: %f", error);
  }
}


void pidTurn(float target, float rotate_tolocal, float timeout) { // ROTATE with tolerate variable
  float pTol = rotate_tolocal;
  float dTol = rotate_tolocal;
  float lastError;
  float kp = 0.9;  // for new robot
  float kd = 0.4; // for new robot
  float ki = 0.05; // for new roobot
  float spd_ratio = 0.7; //0.5
  float s_error = 0;
  int n = 0;
  int repeat = 0;
  imu.set_rotation(0);
  while(true){
 

    float error = target - imu.get_rotation();
    float P = error * kp;
    float D = (error - lastError) * kd;
    s_error += error; // 1
    s_error = fmin(s_error,100); // 2
    s_error = fmax(s_error,-100); // 2
    //Brain.Screen.printAt(10,10,"inertial=%f,error=%f,P+D+I=%f",sensor.rotation(deg),error,P+D);//1031
    if (error * lastError < 0) s_error = 0; // 3
    float I = ki * s_error;
    // if (error != 0) break; //delete later



    if(fabs(error)<pTol){
      stop();
      break;
    }

    if(repeat > timeout){
        stop();
        break;
      }

    float pidspd = (P+D+I);

    if (std::abs(pidspd) < 15) pidspd = sign(pidspd) * 15;

    turn(pidspd);
    pros::c::screen_print(pros::E_TEXT_MEDIUM, n++, "pid: %f, %f, %f", (P+D+I), imu.get_rotation(), error);
    repeat++;

    if (n>150){
        pros::c::screen_erase();
        n = 0;
    }

    
    lastError = error;
    pros::c::delay(20);
  }

}

int autonselector = 0;

 
        

void initialize() {


    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen


    /*front_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    middle_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    back_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    front_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    middle_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    back_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);*/


    //pros::c::screen_print(pros::E_TEXT_MEDIUM, 4, "encoder unit: %d", front_left_motor.get_encoder_units());
    pros::Task Intake(intake);
    pros::Task Hook(hook);
    
    //pros::lcd::clear();
    //pros::lcd::print(5, "IUESHIUSEFIHUWER");

    master.clear();
    pros::delay(100);
    master.print(5, 5, "RED_SAWP");


#if 0
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
#endif

    //master.clear();
    master.print(5, 5, "ha;sdf;jiasdf");


    //disabled();
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */

void disabled() {


/*while (true) {

            master.clear();

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
        autonselector = 1;
        master.clear();
        master.print(0, 0, "RED_SAWP:");
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
        autonselector = 2;
         master.clear();
        master.print(0, 0, "RED_RUSH");
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
        autonselector = 3;
         master.clear();
        master.print(0, 0, "RED_RING");
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        autonselector = 4;
        master.print(0, 0, "BLUE_SAWP");
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        autonselector = 5;
        master.print(0, 0, "BLUE_RUSH");
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        autonselector = 6;
        master.print(0, 0, "BLUE_RING");
    }
    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
        autonselector = 7;
        master.print(0, 0, "SKILL");
    }
    pros::delay(100);
    }*/
}


void competition_initialize() {

}

/**

 */
void autonomous() {
    //chassis.turnToHeading(90,5000);

    //pros::lcd::print(1, "encoder unit: %d", front_left_motor.get_encoder_units());
     //pros::c::screen_print(pros::E_TEXT_MEDIUM, 2, "encoder unit: %d", front_left_motor.get_encoder_units());
    //master.print(1, "encoder unit: %d", front_left_motor.get_encoder_units());
    //moveForMsec(20, true, 6000);
    //moveForSec(20, true, 2);
    
    //pros::c::screen_print(pros::E_TEXT_MEDIUM, 2, "encoder unit: %d", vertical_encoder.get_position());

    pros::c::screen_print(pros::E_TEXT_MEDIUM, 2, "encoder unit");
    

    //SOLO AWP BLUE
    /*chassis.setPose(0,0,222);
    moveForSec(40, true, 0.46);
    wallie.move(13000);
    pros::c::delay(800);
    wallie.move(-12000);
    moveForSec(40, false, 1.1);
    wallie.brake();
    pidTurn(-45, 2, 200);
    pidMoveold(-14, 2);
    moveForSec(30, false, 0.4);
    clampmech();
    pros::c::delay(500);
    pidTurn(-129, 2, 200);
    setintakespd(100);
    sethookspd(-100);
    pidMoveold(10, 2);
    pros::c::delay(1000);
    pidMoveold(-12, 2);
    pidTurn(48, 2, 200);

    pidMoveold(14, 2);
    pros::c::delay(800);
    pidTurn(-160, 2, 200);
    wallie.move(3200);
    moveForSec(90, true, 0.65);*/

    //SOLO AWP RED
    /*chassis.setPose(0,0,222);
    moveForSec(40, true, 0.46);
    wallie.move(13000);
    pros::c::delay(800);
    wallie.move(-12000);
    moveForSec(40, false, 1.1);
    wallie.brake();
    pidTurn(45, 2, 200);
    pidMoveold(-14, 2);
    moveForSec(30, false, 0.4);
    clampmech();
    pros::c::delay(500);
    pidTurn(129, 2, 200);
    setintakespd(100);
    sethookspd(-100);
    pidMoveold(10, 2);
    pros::c::delay(1000);
    pidMoveold(-12, 2);
    pidTurn(-48, 2, 200);
    pidMoveold(14, 2);
    pros::c::delay(800);
    pidTurn(160, 2, 200);
    wallie.move(3200);
    moveForSec(90, true, 0.65);*/


    //ELIM BLUE
    /*chassis.setPose(0,0,222);
    wallie.brake();
    pidMoveold(-14, 2);
    moveForSec(25, false, 0.57);
    clampmech();
    pros::c::delay(750);
    pidTurn(-125, 2, 100);
    setintakespd(100);
    sethookspd(-100);
    pidMoveold(13, 2); //move to stack
    pidTurn(40, 2, 100);
    moveForSec(50, true, 0.8);
    pros::c::delay(500);
    moveForSec(50, false, 0.4);
    pidTurn(-30, 2, 100);
    pidMoveold(-14.5, 2); //move out stack
    pidTurn(43, 2, 100);
    pidMoveold(13.5, 2);
    pros::c::delay(1000);
    setintakespd(0);
    //setintakespd(-100);
    //pidMoveold(1.5, 2);
    pros::c::delay(100);
    pidTurn(-147, 2, 100);
    moveForSec(60, true, 0.9);
    wallie.move(1000);
    moveForSec(60, true, 0.6);
    wallie.brake();
    /*wallie.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wallie.move(13000);
    pros::c::delay(140);
    wallie.brake();*/
    //setintakespd(100);
    /*pidTurn(-6, 2, 40);
    moveForSec(40, true, 0.4);
    pros::c::delay(1200);
    sethookspd(25);
    pros::c::delay(50);
    sethookspd(0);
    wallie.move(13000);
    pros::c::delay(800);
    wallie.brake();
    right_motor_group.move(4000);
    pros::c::delay(800);
    right_motor_group.brake();*/



    //ELIM RED
    /*chassis.setPose(0,0,222);
    wallie.brake();
    pidMoveold(-12, 2);
    moveForSec(25, false, 0.7);
    clampmech();
    pros::c::delay(750);
    pidTurn(120, 2, 100);
    setintakespd(100);
    sethookspd(-100);
    pidMoveold(10, 2); //move to stack
    pidTurn(-35, 2, 100);
    moveForSec(50, true, 0.8);
    pros::c::delay(500);
    moveForSec(50, false, 0.4);
    pidTurn(25, 2, 100);
    pidMoveold(-14.5, 2); //move out stack
    pidTurn(-43, 2, 100);
    pidMoveold(14.5, 2);
    pros::c::delay(500);
    setintakespd(-100);
    pidMoveold(1.5, 2);
    pros::c::delay(100);
    pidTurn(-133, 2, 100);
    wallie.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wallie.move(13000);
    pros::c::delay(140);
    wallie.brake();
    setintakespd(100);
    pidMoveold(28.7, 2);
    pidTurn(6, 2, 40);
    moveForSec(40, true, 0.4);
    pros::c::delay(1200);
    sethookspd(25);
    pros::c::delay(50);
    sethookspd(0);
    wallie.move(13000);
    pros::c::delay(800);
    wallie.brake();
    left_motor_group.move(4000);
    pros::c::delay(800);
    left_motor_group.brake();*/






    //pros::delay(10000); AUTON SKILLLLS
    /*chassis.setPose(0,0,135);
    wallie.move(13000);
    pros::c::delay(800);
    wallie.brake();
    pros::c::delay(1000);
    moveForSec(40, false, 1);
    moveForSec(20, false, 0.8);
    clampmech();
    pros::c::delay(500);
    chassis.turnToHeading(0, 3000);
    sethookspd(50);
    setintakespd(100);
    pros::c::delay(500);
    sethookspd(-90);
    pidMoveold(13, 2); //13
    pros::c::delay(500);
    chassis.turnToHeading(270, 3000);
    pidMoveold(8, 2);
    chassis.turnToHeading(180, 3000);
    pidMoveold(20, 2);
    chassis.turnToHeading(315, 2000);
    pidMoveold(5, 1);
    chassis.turnToHeading(30, 1000);
    pidMoveold(-8, 1);*/

    // RED RUSH 
    /*wallie.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wallie.move(52);
    pidMoveold(25, 2);
    wallie.brake();
    pidTurn(80, 2, 50); //50, 50
    pros::c::delay(300);
    pidTurn(-40, 2, 50);
    wallie.move(100);
    pros::c::delay(300);
    wallie.move(-13000);
    //pros::c::delay(000);
    moveForSec(30, false, 1.5);
    wallie.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    wallie.brake();
    clampmech();
    pros::c::delay(400);
    pidTurn(30, 2, 100);
    setintakespd(100);
    //sethookspd(-100);
    pidMoveold(18, 2);
    sethookspd(-100);
    pros::c::delay(1400);
    pidMoveold(-10, 2);
    clampmech();
    pidMoveold(5, 2);
    pidTurn(80, 2, 100);
    pidMoveold(-4, 2);*/




    // BLUE RUSH 
    /*wallie.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wallie.move(52);
    pidMoveold(25, 2);
    wallie.brake();
    pidTurn(-80, 2, 50); //50, 50
    pros::c::delay(300);
    pidTurn(40, 2, 50);
    wallie.move(100);
    pros::c::delay(300);
    wallie.move(-13000);
    //pros::c::delay(000);
    moveForSec(30, false, 1.5);
    wallie.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    wallie.brake();
    clampmech();
    pros::c::delay(400);
    pidTurn(-30, 2, 100);
    setintakespd(100);
    pidMoveold(16, 2);
    sethookspd(-100);
    pros::c::delay(1400);
    pidMoveold(-10, 2);
    clampmech();
    pidMoveold(5, 2);
    pidTurn(-80, 2, 100);
    pidMoveold(-4, 2);*/




    //RED RUSH 2
    /*wallie.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    setintakespd(100);
    wallie.move(55);
    pidMoveold(26.5, 2);
    wallie.brake();
    pros::c::delay(600);
    //pidTurn(80, 2, 50); //50, 50
    //pros::c::delay(300);
    pidTurn(42, 2, 50);
    //wallie.move(100);
    //pros::c::delay(300);
    //wallie.move(-13000);
    //pros::c::delay(000);
    moveForSec(30, false, 2.1);
    wallie.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    wallie.brake();
    clampmech();
    pros::c::delay(400);
    pidTurn(30, 2, 100);
    sethookspd(-100);
    pidMoveold(18, 2);
    pros::c::delay(1000);
    pidMoveold(-10, 2);
    clampmech();
    pidMoveold(5, 2);
    pidTurn(80, 2, 100);
    pidMoveold(-4, 2);*/



    // BLUE RUSH 2
    /*wallie.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    setintakespd(100);
    wallie.move(55);
    pidMoveold(26.5, 2);
    wallie.brake();
    pros::c::delay(600);
    //pidTurn(80, 2, 50); //50, 50
    //pros::c::delay(300);
    pidTurn(42, 2, 50);
    //wallie.move(100);
    //pros::c::delay(300);
    //wallie.move(-13000);
    //pros::c::delay(000);
    moveForSec(30, false, 2.1);
    wallie.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    wallie.brake();
    clampmech();
    pros::c::delay(400);
    pidTurn(30, 2, 100);
    sethookspd(-100);
    pidMoveold(18, 2);
    pros::c::delay(1000);
    pidMoveold(-10, 2);
    clampmech();
    pidMoveold(5, 2);
    pidTurn(80, 2, 100);
    pidMoveold(-4, 2);*/\



    // BLUE QUAL STAKE
    /*wallie.brake();
    pidMoveold(-14, 2);
    moveForSec(25, false, 0.7);
    clampmech();
    pros::c::delay(750);
    pidTurn(90, 2, 100);
    setintakespd(100);
    sethookspd(-100);
    pidMoveold(14.5, 2);
    pros::c::delay(500);
    //setintakespd(-100);
    //pidMoveold(1.5, 2);
    pros::c::delay(100);
    pidTurn(126, 2, 200);
    wallie.move(800);
    moveForSec(90, true, 0.8);
    wallie.brake();*/

    
    
    
    //RED QUAL STAKE
    /*pidMoveold(-14, 2);
    moveForSec(25, false, 0.7);
    clampmech();
    pros::c::delay(750);
    pidTurn(-90, 2, 100);
    setintakespd(100);
    //sethookspd(-100);
    pidMoveold(15.5, 2);
    pros::c::delay(500);
    setintakespd(0);
    moveForSec(30, false, 0.5);
    //setintakespd(-100);
    //pidMoveold(1.5, 2);
    pros::c::delay(100);
    pidTurn(-80, 2, 150);
    sethookspd(-100);
    setintakespd(100);*/

    /* Backup Auton Skills
    wallie.move(13000);
    pros::c::delay(750);
    wallie.move(-13000);
    pidMoveold(-5, 1);
    pros::c::delay(300);
    pidTurn(-85, 2, 150);
    pros::c::delay(300);
    wallie.move(0);
    pidMoveold(-12, 2);
    moveForSec(40, false, 0.25);
    clampmech();
    pros::c::delay(300);
    pidTurn(-92, 2, 150);
    sethookspd(-100);
    setintakespd(100);
    pros::c::delay(300);
    pidMoveold(12, 2); //intake first ring
    pros::c::delay(300);
    pidTurn(-25, 2, 150);
    pros::c::delay(300);
    //moveForSec(30, true, 0.5);
    pros::c::delay(300);
    sethookspd(0);
    pidMoveold(32, 3);
    pros::c::delay(300);
    wallie.move(-13000);
    pros::c::delay(250);
    wallie.move(13000); //wallstake macro
    pros::c::delay(100);
    wallie.brake();
    wallie.move(10);
    pidMoveold(-4, 2);
    pros::c::delay(300);
    sethookspd(-100);
    pidTurn(-130, 2, 150); //turn for ring in front of wallstake 
    pros::c::delay(300);
    pidMoveold(17, 2); //drive to ring in front of wallstake 
    pros::c::delay(300);
    moveForSec(50, false, 0.5);
    pros::c::delay(300);
    pidTurn(65, 2, 150); //turn to wallstake
    pros::c::delay(300);
    moveForSec(30, true, 1);
    wallie.move(13000);
    pros::c::delay(100);
    sethookspd(5);
    pros::c::delay(50);
    sethookspd(0);
    pros::c::delay(600);
    sethookspd(-100);
    moveForSec(30, false, 0.3);
    moveForSec(100, true, 0.25);
    pros::c::delay(500);
    pidMoveold(-9.5, 2); //move back from wallstake 
    pros::c::delay(500);
    wallie.move(-13000);
    pidTurn(-97, 2, 150);
    pros::c::delay(500);
    moveForSec(60, true, 1.75); //intake the three rings
    moveForSec(40, true, 1);
    wallie.move(0);
    pidMoveold(-10, 2);
    pros::c::delay(300);
    pidTurn(90, 2, 150); //turn to last ring
    pros::c::delay(300);
    moveForSec(40, true, 1);
    pros::c::delay(500);
    pidMoveold(-10, 2);
    pidTurn(135, 2, 150);
    pros::c::delay(300);
    setintakespd(-100);
    moveForSec(30, true, 0.5);
    clampmech(); 
    sethookspd(0);
    setintakespd(0);
    moveForSec(100, false, 1); //put mogo in corner
    pidMoveold(6, 2); */

    //auton skills
    /*wallie.move(13000);
    pros::c::delay(750);
    wallie.move(-13000);
    pidMoveold(-5, 1);
    pros::c::delay(150);
    pidTurn(-85, 2, 150);
    pros::c::delay(150);
    wallie.move(0);
    pidMoveold(-12, 1);
    moveForSec(30, false, 0.35);
    clampmech();
    pros::c::delay(150);
    pidTurn(-92, 1, 150);
    sethookspd(-100);
    setintakespd(100);
    pros::c::delay(150);
    pidMoveold(11, 1); //intake first ring
    pros::c::delay(150);
    pidTurn(-25, 1, 150);
    pros::c::delay(300); //intake far ring 
    //moveForSec(30, true, 0.5);
    pidMoveold(32, 1);
    sethookspd(0);
    moveForSec(20, true, 0.3);
    pros::c::delay(150);
    wallie.move(-13000);
    pros::c::delay(250);
    wallie.move(13000); //wallstake macro
    pros::c::delay(120);
    wallie.brake();
    wallie.move(5);
    pidMoveold(-4, 1);
    pros::c::delay(150);
    sethookspd(-100);
    pidTurn(-130, 1, 150); //turn for ring in front of wallstake 
    pros::c::delay(150);
    pidMoveold(17, 1); //drive to ring in front of wallstake 
    moveForSec(20, true, 0.25);
    pros::c::delay(150);
    moveForSec(35, false, 0.57);
    pros::c::delay(150);
    pidTurn(65, 1, 150); //turn to wallstake
    pros::c::delay(150);
    moveForSec(30, true, 0.5);
    wallie.move(13000);
    pros::c::delay(100);
    sethookspd(5);
    pros::c::delay(50);
    sethookspd(0);
    pros::c::delay(600);
    wallie.brake();
    sethookspd(-100);
    moveForSec(30, false, 0.3);
    moveForSec(60, true, 0.3);
    pidMoveold(-9.5, 2); //move back from wallstake 
    pros::c::delay(300);
    wallie.move(-13000);
    pidTurn(-94, 1, 150);
    pros::c::delay(300);
    moveForSec(70, true, 1.7); //intake the three rings
    moveForSec(30, true, 1);
    wallie.move(0);
    pidMoveold(-10, 2);
    pros::c::delay(150);
    pidTurn(90, 1, 150); //turn to last ring
    pros::c::delay(150);
    moveForSec(40, true, 1);
    pidMoveold(-10, 2);
    pidTurn(135, 1, 150);
    pros::c::delay(150);
    setintakespd(-100);
    //moveForSec(30, true, 0.5);
    clampmech(); 
    sethookspd(0);
    setintakespd(0);
    moveForSec(75, false, 0.75); //put mogo in corner
    pidMoveold(9, 1);
    pidTurn(-135, 1, 150);
    moveForSec(60, true, 1);
    pidMoveold(-46, 2);
    pidTurn(3, 1, 150);
    moveForSec(30, false, 0.6);
    clampmech(); //--------------------------------------------SECOND CORNER------------------------------------------------------------------------
    pros::c::delay(150);
    moveForSec(30, true, 0.25);
    pidTurn(88, 1, 150);
    sethookspd(-100);
    setintakespd(100);
    pros::c::delay(150);
    pidMoveold(11, 1); //intake first ring
    pros::c::delay(150);
    pidTurn(25, 1, 147);
    pros::c::delay(300); //intake far ring 
    //moveForSec(30, true, 0.5);
    pidMoveold(32, 1);
    sethookspd(0);
    moveForSec(20, true, 0.3);
    pros::c::delay(150);
    wallie.move(-13000);
    pros::c::delay(200);
    wallie.move(13000); //wallstake macro
    pros::c::delay(120);
    wallie.brake();
    wallie.move(5);
    pidMoveold(-4, 1);
    pros::c::delay(150);
    sethookspd(-100);
    pidTurn(130, 1, 150); //turn for ring in front of wallstake 
    pros::c::delay(150);
    pidMoveold(17, 1); //drive to ring in front of wallstake 
    moveForSec(20, true, 0.25);
    pros::c::delay(150);
    moveForSec(35, false, 0.7);
    pros::c::delay(150);
    pidTurn(-70, 1, 150); //turn to wallstake
    pros::c::delay(150);
    moveForSec(30, true, 0.5);
    wallie.move(13000);
    pros::c::delay(200);
    sethookspd(5);
    pros::c::delay(50);
    sethookspd(0);
    pros::c::delay(600);
    wallie.brake();
    sethookspd(-100);
    moveForSec(30, false, 0.3);
    moveForSec(60, true, 0.3);
    pidMoveold(-9.5, 2); //move back from wallstake 
    pros::c::delay(300);
    wallie.move(-13000);
    pidTurn(94, 1, 150);
    pros::c::delay(300);
    moveForSec(70, true, 1.7); //intake the three rings
    moveForSec(30, true, 1);
    wallie.move(0);
    pidMoveold(-10, 1);
    pros::c::delay(150);
    pidTurn(-90, 1, 150); //turn to last ring
    pros::c::delay(150);
    moveForSec(40, true, 1);
    pidMoveold(-10, 1);
    pidTurn(-135, 1, 150);
    pros::c::delay(150);
    clampmech(); 
    sethookspd(0);
    setintakespd(0);
    moveForSec(75, false, 0.75); //put mogo in corner
    pidMoveold(6, 1);
    sethookspd(-100);
    setintakespd(100);


    pidTurn(30, 1, 150);
    pidMoveold(30, 1);
    sethookspd(0);
    pidTurn(-45, 1, 150);
    pidMoveold(17, 1);
    pidTurn(180, 1, 200);
    pidMoveold(-17, 1);
    clampmech();
    pros::c::delay(200);
    sethookspd(-100);
    pidTurn(-45, 1, 150);
    pidMoveold(22, 1);
    pidTurn(135, 1, 150);
    clampmech();
    moveForSec(75, false, 0.75);*/


    //SAFE STAKE SIDE 
    pidMoveold(6, 1);
    pros::c::delay(200);
    wallie.move(13000);
    pros::c::delay(1000);
    moveForSec(30, false, 0.5);
    pidMoveold(-8, 1);
    pros::c::delay(200);
    pidTurn(10, 2, 150);
    pidMoveold(-13, 1);
    wallie.move(-13000);
    pros::c::delay(200);
    moveForSec(30, false, 0.5);
    clampmech();
    pros::c::delay(200);
    moveForSec(30, true, 0.25);
    pidTurn(112, 2, 150);
    pros::c::delay(200);
    setintakespd(100);
    pidMoveold(12, 1);
    moveForSec(30, true, 1);
    sethookspd(-100);
    moveForSec(30, false, 0.5);
    pros::c::delay(200);
    pidTurn(-160, 2, 150);
    pros::c::delay(250);
    doinkermech();
    //raise intake here
    sethookspd(0);
    pidMoveold(30, 1);
    pros::c::delay(200);
    doinkermech();
    moveForSec(30, false, 1);
    sethookspd(-100);
    pidTurn(-70, 2, 150); //TOUCH LADDER 
    moveForSec(60, true, 1);
    sethookspd(0);
    setintakespd(0);

    /*moveForSec(30, false, 0.5);
    pidTurn(-30, 2, 150);
    moveForSec(60, false, 1); //GO TO POS CORNER*/
    

    




  /*wallie.move(13000);
  wait(0.5;
  wallie.move(-4000);
  wait(0.05, seconds);
  pidMoveold(-4,1);
  wallie.move(-7000);
  pidTurn(-80, 2, 150);
  wallie.brake();
  moveForSec(30, false, 1.3);
  clampmech();
  wait(0.3, seconds);
  pidTurn(-81, 2, 150);
  setintakespd(-100);
  sethookspd(-100);
  pidMoveold(15,2);
  pidTurn(-16, 2, 150);
  pidMoveold(48,2);
  pidTurn(-146, 2, 150);
  pidMoveold(11,2);
  wallie.spin(forward, 4000, voltageUnits::mV);
  wait(0.2, seconds);
  wallie.stop(hold);
  turnPID(76);
  moveForSec(20, true, 1.2);

  wait(0.4, seconds);
  setcataspd(0);
  wallie.move(13000);
  wait(0.1, seconds);
  wallie.move(-13000);
  wait(0.1, seconds);
  setcataspd(-100);
  wallie.move(13000);
  wait(0.1, seconds);
  wallie.move(-13000);
  wait(0.1, seconds);
  setcataspd(0);
  wallie.move(13000);
  wait(0.1, seconds);
  wallie.move(-13000);
  wait(0.1, seconds);
  setcataspd(0);
  wallie.move(13000);
  wait(0.1, seconds);
  setcataspd(-100);
  wallie.move(13000);
  wait(0.3, seconds);  

  setcataspd(30);
  wallie.move(13000);
  wait(0.8, seconds);
  moveForSec(50);
  wallie.move(-13000);
  wait(0.1, seconds);
  wallie.move(13000);
  wait(0.4, seconds);
  stop();
  setcataspd(0);
  wallie.move(-13000);
  drivePID(-10);
  wallie.stop();

  turnPID(-80);
  setcataspd(-100);
  moveForSec(50, true, 64);
  turnPID(120);
  drivePID(11);
  turnPID(68);
  moveForSec(40, false, 1);
  clampmech();*/


}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    // loop forever
    bool clamptoggle = false;
    bool doinkertoggle = false;
    bool clampsignal = false;
    wallie.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    
    front_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    middle_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    front_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    middle_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    back_right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    while (true) {
        // get left y and right x positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);
        
        
        //clamp
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            clamptoggle = !clamptoggle;
            clamp.set_value(clamptoggle);
        }

        //doinker
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            doinkertoggle = !doinkertoggle;
            doinker.set_value(doinkertoggle);
            clampsignal = !clampsignal;
            master.clear();
            //master.print(std::uint8_t line, std::uint8_t col, const char *fmt, Params args...)
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            wallie.move(13000);
            pros::c::delay(130);
            wallie.brake();
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) wallie.move(13000);

        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) wallie.move(-13000);

        else {
            wallie.brake();
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            setintakespd(100);
            sethookspd(-100);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            setintakespd(-100);
            sethookspd(100);
        }
        else {
            setintakespd(0);
            sethookspd(0);
        }
    pros::c::delay(25);
    }
}    
      

	