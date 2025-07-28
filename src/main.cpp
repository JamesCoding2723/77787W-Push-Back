#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include <cmath>
#include "main.h"


//ROBOT CONFIG  //  PID CONFIG
#pragma region

    pros::Motor backintakem(21, pros::E_MOTOR_GEAR_BLUE, 1);
    pros::Motor intakem (17, pros::E_MOTOR_GEAR_BLUE, 1);

    pros::ADIDigitalOut clamp('A', false); 	

    pros::ADIDigitalOut doinker('B', false); 

    pros::ADIDigitalOut intakep('C', false);
					 
    pros::Controller master (pros::E_CONTROLLER_MASTER);

    pros::Motor front_left_motor(11, pros::E_MOTOR_GEAR_BLUE); // front left motor -13
    pros::Motor middle_left_motor(-12, pros::E_MOTOR_GEAR_BLUE); // middle left motorv -12
    pros::Motor back_left_motor(13, pros::E_MOTOR_GEAR_BLUE); // back left motor -11
    pros::Motor front_right_motor(-14, pros::E_MOTOR_GEAR_BLUE); // front right motor 15
pros::Motor middle_right_motor(15, pros::E_MOTOR_GEAR_BLUE); // middle right motor 17
pros::Motor back_right_motor(-16, pros::E_MOTOR_GEAR_BLUE); // back right motor 16


pros::Rotation vertical_encoder(6);




// left group
pros::MotorGroup left_motor_group({ front_left_motor, middle_left_motor, back_left_motor });
// right group
pros::MotorGroup right_motor_group({ front_right_motor, middle_right_motor, back_right_motor });

lemlib::Drivetrain drivetrain(&left_motor_group, // left group
                              &right_motor_group, // right group
                              13, //track width
                              lemlib::Omniwheel::OLD_325, // using old_2.75
                              450, 
                              2 // horizontal drift is 2 (for now)
);

//lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);
//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0.75);

pros::Imu imu(9);

/*pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
pros::ADIEncoder vertical_encoder('C', 'D', true);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel*/
//lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel  
							nullptr,
                            nullptr, // horizontal tracking wheel
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

int ftintakespd = 0;

void setftintakespd(float spd) {
    ftintakespd = spd;
}

void fntintake() {
    while(true){
        if (ftintakespd == 0) {
            intakem.brake();
            }
        else {
            intakem.move((int)130 * ftintakespd);
        }
    }
}

float backintakespd = 0;

void setbkintakespd(float spd) {
    backintakespd = spd;
}

void bkintake() {
    while(true){
        if (backintakespd == 0) backintakem.brake();
        else backintakem.move((int)130 * backintakespd);
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


bool raisetoggle = false; 

void raisemech() {
    raisetoggle = !raisetoggle; 
    intakep.set_value(raisetoggle);
}



void moveforward(float dis, bool dir, float tm) {
    chassis.setPose(0, 0, 0);
    if (dir == false) chassis.moveToPoint(0, -dis, tm, {.forwards = false});
    else if (dir == true) chassis.moveToPoint(0, dis, tm); 
}

void wait(float time) {
    pros::c::delay(time);
}

constexpr double start_heading = 90;

double x = 0;
double y = 0;

void tracking(){
    vertical_encoder.set_position(0);   

    double prevdis = 0;

    while (true) {
        double heading = std::fmod(360 - imu.get_heading() + start_heading, 360);

        double dis_trav = vertical_encoder.get_position() * 0.0002399827721;
        double deltapos = dis_trav - prevdis;

        



    }
}

float InchToEncoderunit(float distance){
    return ((distance * 360) / 10.21017) * (4/3); // gear ratio is 4:3
}


void pidMoveold(float target_inch, float tolerence_inch, float timeout){ // MOVE MOVE MOVE
  
  //float target = target_inch
  //float tolerence = InchToEncoderunit(tolerence_inch);
  float lastError;
  float kp = 2.1; // for new robot
  float kd = 0.1; // for new robot
  float ki = 0.0; // for new roobot
  float s_error = 0;
  int line_number = 1;
  float speed_ratio = 2.2, spd;
  vertical_encoder.set_position(0);
  int repeat_limit = 2000000;
  int repeat = 0;

  while(true){
    float error = target_inch - (vertical_encoder.get_position() * 0.0002399827721);
    float P = error * kp;
    float D = (error - lastError) * kd;
    s_error += error;
    s_error = fmin(s_error,100);
    s_error = fmax(s_error,-100);
      // Brain.Screen.printAt(10,20,"error=%f",error);
    if (error * lastError < 0) s_error = 0;

    if(fabs(error)<tolerence_inch && spd < 16){
      //move_turn(0);
      stop();
      break;
    }

    if(repeat > 100 * timeout){
        stop();
        break;
    }

    float I = ki * s_error;
    lastError = error;
    spd = (P+D+I)*speed_ratio;
    if (std::abs(spd) < 15) spd = sign(spd) *15;
    move(spd);
    repeat++;
    pros::c::delay(10);
    pros::c::screen_print(pros::E_TEXT_MEDIUM, line_number++, "error: %f", vertical_encoder.get_position());
  }
}


void pidTurn(float target, float rotate_tolocal, float timeout) { // ROTATE with tolerate variable
  float pTol = rotate_tolocal;
  float dTol = rotate_tolocal;
  float lastError;
  float kp = 0.8;  // for new robot
  float kd = 0.4; // for new robot
  float ki = 0.02; // for new roobot
  float spd_ratio = 2; //0.5
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

    if (std::abs(pidspd) < 23) pidspd = sign(pidspd) * 23;

    turn(pidspd);
    pros::c::screen_print(pros::E_TEXT_MEDIUM, n++, "pid: %f, %f, %f", (P+D+I), imu.get_rotation(), error);
    repeat++;

    
    lastError = error;
    pros::c::delay(10);
  }

}

int autonselector = 0;

 
        

void initialize() {


    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    vertical_encoder.reset();
    vertical_encoder.set_reversed(true);
    // print position to brain screen


    /*front_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    middle_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    back_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    front_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    middle_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    back_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);*/


    //pros::c::screen_print(pros::E_TEXT_MEDIUM, 4, "encoder unit: %d", front_left_motor.get_encoder_units());
    pros::Task Intake(bkintake);
    pros::Task Hook(fntintake);
    
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
    moveForSec(30, false, 0.4);*/
 
    pidMoveold(11, 1, 200);
    pidTurn(-20, 1, 200);
    setftintakespd(-100);
    moveForSec(50, true, 0.4);
    pidTurn(-45, 1, 200);
    pidMoveold(14, 1, 200);
    pidTurn(40, 1, 200);
    moveForSec(30, true, 1.2);
    pros::delay(600);
    pidMoveold(-30, 1, 200);
    setftintakespd(0);
    clamptoggle = !clamptoggle;
    clamp.set_value(clamptoggle);
    pidTurn(-47, 1, 200);
    pidMoveold(22, 1, 200);
    pidTurn(-81, 1, 200);
    moveForSec(30, false, 2);
    setftintakespd(-100);






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
            //master.print(std::uint8_t line, std::uint8_t col, const char *fmt, Params args...)
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            raisetoggle = !raisetoggle;
            intakep.set_value(raisetoggle);
            //master.print(std::uint8_t line, std::uint8_t col, const char *fmt, Params args...)
        }

        /*if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            wallie.move(13000);
            pros::c::delay(145);
            wallie.brake();
        }*/

            
        

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) setbkintakespd(-0.55);

        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) setbkintakespd(100);

        else {
            setbkintakespd(0);
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            setftintakespd(-100);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            setftintakespd(100);
        }
        else {

            setftintakespd(0);
        }
    pros::c::delay(25);
      
}   
}


	