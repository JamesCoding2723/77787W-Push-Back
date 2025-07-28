#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include <cmath>
#include "robot_config.cpp"



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

bool blockertoggle = false;

void blockermech() {
    //bool clamptoggle = false;
    blockertoggle = !blockertoggle;
    blocker.set_value(blockertoggle);
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

double tracking(){
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
 