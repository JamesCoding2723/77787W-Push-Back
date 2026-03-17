#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/screen.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include "robot_config.h"
#include "basic_functions.h"

float InchToEncoderunit(float distance)
{
  return ((distance * 360) / 10.21017) * (4 / 3); // gear ratio is 4:3
}



void pidMoveold(float target_inch, float tolerence_inch, float timeout, float max)
{ // MOVE MOVE MOVE

  // float target = target_inch
  // float tolerence = InchToEncoderunit(tolerence_inch);
  float lastError;
  float kp = 5.0; // for new robot
  float kd = 0.2; // for new robot
  float ki = 0.0; // for new roobot
  float s_error = 0;
  int line_number = 1;
  float spd;
  front_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  front_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  front_left_motor.set_zero_position(0);
  front_right_motor.set_zero_position(0);
  
  
  int repeat_limit = 2000000;
  int repeat = 0;

  while (true)
  {
    float error = target_inch - (((front_right_motor.get_position() + front_left_motor.get_position()) / 2) * 0.02127120025);
    float P = error * kp;
    float D = (error - lastError) * kd;
    s_error += error;
    s_error = fmin(s_error, 100);
    s_error = fmax(s_error, -100);
    // Brain.Screen.printAt(10,20,"error=%f",error);
    if (error * lastError < 0)
      s_error = 0;

    if (fabs(error) < tolerence_inch && spd < 16)
    {
      // move_turn(0);
      stop();
      pros::c::screen_print(pros::E_TEXT_MEDIUM, line_number++, "error: %f", front_right_motor.get_position());
      break;
    }

    if (repeat > 100 * timeout)
    {
      stop();
      break;
    }

    float I = ki * s_error;
    lastError = error;
    spd = (P + D + I);
    if (std::abs(spd) < 15)
      spd = sign(spd) * 15;
    move(std::clamp(spd, -max, max));
    repeat++;
    pros::c::delay(10);
  }
}

void pidTurnRel(float target, float rotate_tolocal, float timeout, float max)
{ // ROTATE with tolerate variable
  float pTol = rotate_tolocal;
  float dTol = rotate_tolocal;
  float lastError;
  float kp = 1.45;      // for new robot
  float kd = 7.0;      // for new robot
  float ki = 0.0;     // for new roobot
  //float spd_ratio = 2; // 0.5
  float s_error = 0;
  int n = 0;
  int repeat = 0;
  int line_number = 1;
  //imu.set_rotation(0);
  target += imu.get_rotation();

  while (true)
  {

    float error = target - imu.get_rotation();
    float P = error * kp;
    float D = (error - lastError) * kd;
    s_error += error;              // 1
    s_error = fmin(s_error, 100);  // 2
    s_error = fmax(s_error, -100); // 2
    if (error * lastError < 0)
      s_error = 0; // 3
    float I = ki * s_error;
    // if (error != 0) break; //delete later

    if (fabs(error) < pTol)
    {
      stop();
      pros::c::screen_print(pros::E_TEXT_MEDIUM, line_number++, "error: %f", imu.get_rotation());
      break;
    }

    if (repeat > 100 * timeout)
    {
      stop();
      break;
    }

    float pidspd = (P + D + I);

    if (std::abs(pidspd) < 23)
      pidspd = sign(pidspd) * 23;

    turn(std::clamp(pidspd, -max, max));
    std::cout << pidspd << std::endl;
    pros::c::screen_print(pros::E_TEXT_MEDIUM, n++, "pid: %f, %f, %f", (P+D+I), imu.get_rotation(), error);
    repeat++;

    lastError = error;
    pros::c::delay(10);
  }
}


void pidTurnAbs(float target, float rotate_tolocal, float timeout, float max)
{ // ROTATE with tolerate variable
  float pTol = rotate_tolocal;
  float dTol = rotate_tolocal;
  float lastError;
  float error;
  float kp = 1.7;      // for new robot
  float kd = 7.0;      // for new robot
  float ki = 0.15;     // for new robot
  //float spd_ratio = 2; // 0.5
  float s_error = 0;
  int n = 0;
  int repeat = 0;
  int settletime = 0;
  while (true)
  {
    double currentHeading = imu.get_heading(); 
    double error = target - currentHeading;

    // Handle wraparound
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    float P = error * kp;
    float D = (error - lastError) * kd;
    s_error += error;              // 1
    s_error = fmin(s_error, 100);  // 2
    s_error = fmax(s_error, -100); // 2
    // Brain.Screen.printAt(10,10,"inertial=%f,error=%f,P+D+I=%f",sensor.rotation(deg),error,P+D);//1031
    if (error * lastError < 0)
      s_error = 0; // 3
    float I = ki * s_error;
    // if (error != 0) break; //delete later


    if (repeat > timeout*100)
    {
      stop();
      break;
    }

    float pidspd = (P + D + I);

   // if (std::abs(pidspd) < 15)
   //   pidspd = sign(pidspd) * 15;


    if (fabs(error) < pTol && fabs(pidspd) < 30)
    {
      settletime ++;
    }

    if (settletime > 15)
      {
        stop();
        break;
    }


    turn(std::clamp(pidspd, -max, max));
    pros::c::screen_print(pros::E_TEXT_MEDIUM, 4, "pid: %f, %f, %f", pidspd, settletime, error);
    repeat++;

    lastError = error;
    pros::c::delay(10);
  }
  stop();
}



void pidswingAbs(float target, float rotate_tolocal, float timeout, bool side, float spdmod)
{ // ROTATE with tolerate variable
  float pTol = rotate_tolocal;
  float dTol = rotate_tolocal;
  float lastError;
  float kp = 0.8;      // for new robot
  float kd = 0.4;      // for new robot
  float ki = 0.02;     // for new roobot
  float spd_ratio = 2; // 0.5
  float s_error = 0;
  int n = 0;
  int repeat = 0;
  imu.set_rotation(0);
  int settletime;
  //target = imu.get_rotation() + target;

  while (true)
  {

    double currentHeading = imu.get_heading(); 
    double error = target - currentHeading;

    // Handle wraparound
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    float P = error * kp;
    float D = (error - lastError) * kd;
    s_error += error;              // 1
    s_error = fmin(s_error, 100);  // 2
    s_error = fmax(s_error, -100); // 2
    // Brain.Screen.printAt(10,10,"inertial=%f,error=%f,P+D+I=%f",sensor.rotation(deg),error,P+D);//1031
    if (error * lastError < 0)
      s_error = 0; // 3
    float I = ki * s_error;
    // if (error != 0) break; //delete later

    if (fabs(error) < pTol)
    {
      stop();
      break;
    }

    if (repeat > timeout*100)
    {
      stop();
      break;
    }

    float pidspd = (P + D + I);

    if (std::abs(pidspd) < 23)
      pidspd = sign(pidspd) * 23;

    
    if (side == true) {
      moveright(pidspd);
    }
    else{
      moveleft(pidspd);
    }
    std::cout << pidspd << std::endl;
    pros::c::screen_print(pros::E_TEXT_MEDIUM, n++, "pid: %f, %f, %f", (P+D+I), imu.get_rotation(), error);
    repeat++;

    lastError = error;
    pros::c::delay(10);
  }
}


void pidGyro(double targetInches, double targetHeading, double timeout, double max, double E_TOL, double D_TOL, double _settle, float _turnscale, float spdmod, float _drivescale) {

    pros::c::screen_print(pros::E_TEXT_MEDIUM, 5, "pid: %f, %f, %f, %f", 1,2,3,4);        


    // Reset sensors
    front_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    front_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    front_left_motor.set_zero_position(0);
    front_right_motor.set_zero_position(0);


    //drive PID
    double kP_drive = 5.0;
    double kI_drive = 0.0;
    double kD_drive = 0.2;
    double driveS_error;

    //turn PID
    double kP_turn = 2;
    double kI_turn = 0.0;
    double kD_turn = 7.0;
    double turnS_error;

    double driveError, drivePrevError;
    double turnError, turnPrevError;

    double settleTime;
    int repeat;
    while (true) {
        repeat++;
        //Drive PID
        driveError = targetInches - (((front_right_motor.get_position() + front_left_motor.get_position()) / 2) * 0.02127120025);
        float driveP = driveError * kP_drive;
        float driveD = (driveError - drivePrevError) * kD_drive;
        driveS_error += driveError;              // 1
        driveS_error = fmin(driveS_error, 100);  // 2
        driveS_error = fmax(driveS_error, -100); // 2
        if (driveError * drivePrevError < 0)
          driveS_error = 0; // 3
        float driveI = kI_drive * driveS_error;

        double driveOutput = (driveP + driveI + driveD) * spdmod;

        //Turn PID
        double currentHeading = imu.get_heading(); 
        turnError = targetHeading - currentHeading;

        // Handle wraparound
        if (turnError > 180) turnError -= 360;
        if (turnError < -180) turnError += 360;

        float turnP = turnError * kP_turn;
        float turnD = (turnError - turnPrevError) * kD_turn;
        turnS_error += turnError;              // 1
        turnS_error = fmin(turnS_error, 100);  // 2
        turnS_error = fmax(turnS_error, -100); // 2
        if (turnError * turnPrevError < 0)
          turnS_error = 0; // 3
        float turnI = kI_turn * turnS_error;

        double turnOutput = (turnP + turnI + turnD) * spdmod;

        //outputs-----------------------------------------
        double turnScale = 1.0 - std::min(fabs(turnError) / _turnscale, 1.0);
        double driveScale = (_drivescale <= 0) ? 1.0 : 1.0 - std::min(fabs(driveError) / _drivescale, 1.0);

        driveOutput *= turnScale;
        turnOutput *= driveScale;

        double leftPower = driveOutput + turnOutput;
        double rightPower = driveOutput - turnOutput;


        double maxMag = std::max(fabs(leftPower), fabs(rightPower));
        if (maxMag > 100) {
          double scale = 100 / maxMag;
          leftPower *= scale;
          rightPower *= scale;
        }

        leftPower  = std::clamp(leftPower, -max, max);
        rightPower = std::clamp(rightPower, -max, max);

        moveleft(leftPower);
        moveright(rightPower);

        //exit
        if (fabs(driveError) < D_TOL && fabs(turnError) < 2 && (leftPower + rightPower)/2 < E_TOL)
            settleTime += 1;
        else
            settleTime = 0;

        drivePrevError = driveError;
        turnPrevError = turnError;

        if (repeat > timeout * 50)
        {
          stop();
          break;
        }

        if (settleTime > _settle)
        {
          stop();
          break;
        }

        pros::c::screen_print(pros::E_TEXT_MEDIUM, repeat++, "pid: %f, %f, %f, %f", leftPower, rightPower, driveError, turnError);        

        pros::delay(20);
    }
}




void pidWallMove(float target_inch, float tolerence_inch, float _wall, float timeout, float max)
{ // MOVE MOVE MOVE

  // float target = target_inch
  // float tolerence = InchToEncoderunit(tolerence_inch);
  float lastError;
  float kp = 5.0; // for new robot
  float kd = 0.2; // for new robot
  float ki = 0.0; // for new roobot
  float s_error = 0;
  int line_number = 1;
  float spd;

  
  int repeat_limit = 2000000;
  int repeat = 0;
  float settletime;

  while (true)
  {
    float error = target_inch - getwallpos(_wall);
    float P = error * kp;
    float D = (error - lastError) * kd;
    s_error += error;
    s_error = fmin(s_error, 100);
    s_error = fmax(s_error, -100);
    // Brain.Screen.printAt(10,20,"error=%f",error);
    if (error * lastError < 0)
      s_error = 0;

    if (fabs(error) < tolerence_inch && spd < 16)
    {
      stop();
      break;
    }

    if (repeat > 100 * timeout)
    {
      stop();
      break;
    }


    float I = ki * s_error;
    lastError = error;
    spd = (P + D + I);

    move(std::clamp(spd, -max, max));
    repeat++;
    pros::c::delay(10);
  }
  stop();
}






void pidwallGyro(double targetInches, double targetHeading, float _wall, double timeout, double max, double E_TOL) {

    pros::c::screen_print(pros::E_TEXT_MEDIUM, 5, "pid: %f, %f, %f, %f", 1,2,3,4);        


    // Reset sensors
    front_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    front_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    front_left_motor.set_zero_position(0);
    front_right_motor.set_zero_position(0);


    //drive PID
    double kP_drive = 5.0;
    double kI_drive = 0.0;
    double kD_drive = 0.2;
    double driveS_error;

    //turn PID
    double kP_turn = 2;
    double kI_turn = 0.1;
    double kD_turn = 7.0;
    double turnS_error;

    double driveError, drivePrevError;
    double turnError, turnPrevError;

    double settleTime;
    int repeat;
    while (true) {
        repeat++;
        //Drive PID
        driveError = targetInches - getwallpos(_wall);
        float driveP = driveError * kP_drive;
        float driveD = (driveError - drivePrevError) * kD_drive;
        driveS_error += driveError;              // 1
        driveS_error = fmin(driveS_error, 100);  // 2
        driveS_error = fmax(driveS_error, -100); // 2
        if (driveError * drivePrevError < 0)
          driveS_error = 0; // 3
        float driveI = kI_drive * driveS_error;

        double driveOutput = driveP + driveI + driveD;

        //Turn PID
        double currentHeading = imu.get_heading(); 
        turnError = targetHeading - currentHeading;

        // Handle wraparound
        if (turnError > 180) turnError -= 360;
        if (turnError < -180) turnError += 360;

        float turnP = turnError * kP_turn;
        float turnD = (turnError - turnPrevError) * kD_turn;
        turnS_error += turnError;              // 1
        turnS_error = fmin(turnS_error, 100);  // 2
        turnS_error = fmax(turnS_error, -100); // 2
        if (turnError * turnS_error < 0)
          turnS_error = 0; // 3
        float turnI = kI_turn * turnS_error;

        double turnOutput = turnP + turnI + turnD;

        //outputs-----------------------------------------
        double turnScale = 1.0 - std::min(fabs(turnError) / 20.0, 1.0);

        driveOutput *= turnScale;

        double leftPower = driveOutput + turnOutput;
        double rightPower = driveOutput - turnOutput;


        double maxMag = std::max(fabs(leftPower), fabs(rightPower));
        if (maxMag > 100) {
          double scale = 100 / maxMag;
          leftPower *= scale;
          rightPower *= scale;
        }

        leftPower  = std::clamp(leftPower, -max, max);
        rightPower = std::clamp(rightPower, -max, max);

        moveleft(leftPower);
        moveright(rightPower);

        //exit
        if (fabs(driveError) < E_TOL && fabs(turnError) < 2 && (leftPower + rightPower)/2 < 25)
            settleTime += 1;
        else
            settleTime = 0;

        drivePrevError = driveError;
        turnPrevError = turnError;

        if (repeat > timeout * 50)
        {
          stop();
          break;
        }

        if (settleTime > 10)
        {
          stop();
          break;
        }

        pros::c::screen_print(pros::E_TEXT_MEDIUM, repeat++, "pid: %f, %f, %f, %f", leftPower, rightPower, driveError, turnError);        

        pros::delay(20);
    }
}




void pidFrontWallGyro(double targetInches, double targetHeading, double _wall, double timeout, double max, int spdratio) {

    pros::c::screen_print(pros::E_TEXT_MEDIUM, 5, "pid: %f, %f, %f, %f", 1,2,3,4);        


    // Reset sensors
    front_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    front_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    front_left_motor.set_zero_position(0);
    front_right_motor.set_zero_position(0);


    //drive PID
    double kP_drive = 5.0;
    double kI_drive = 0.0;
    double kD_drive = 0.2;
    double driveS_error;

    //turn PID
    double kP_turn = 2.0;
    double kI_turn = 0.0;
    double kD_turn = 7.0;
    double turnS_error;

    double driveError, drivePrevError;
    double turnError, turnPrevError;

    double settleTime;
    double repeat = 0;
    while (true) {
        repeat += 1;
        //Drive PID
        driveError = targetInches - getfrontwallpos(_wall);
        float driveP = driveError * kP_drive;
        float driveD = (driveError - drivePrevError) * kD_drive;
        driveS_error += driveError;              // 1
        driveS_error = fmin(driveS_error, 100);  // 2
        driveS_error = fmax(driveS_error, -100); // 2
        if (driveError * drivePrevError < 0)
          driveS_error = 0; // 3
        float driveI = kI_drive * driveS_error;

        double driveOutput = (driveP + driveI + driveD) * spdratio;

        //Turn PID
        double currentHeading = imu.get_heading(); 
        turnError = targetHeading - currentHeading;

        // Handle wraparound
        if (turnError > 180) turnError -= 360;
        if (turnError < -180) turnError += 360;

        float turnP = turnError * kP_turn;
        float turnD = (turnError - turnPrevError) * kD_turn;
        turnS_error += turnError;              // 1
        turnS_error = fmin(turnS_error, 100);  // 2
        turnS_error = fmax(turnS_error, -100); // 2
        if (turnError * turnS_error < 0)
          turnS_error = 0; // 3
        float turnI = kI_turn * turnS_error;

        double turnOutput = turnP + turnI + turnD;

        //outputs-----------------------------------------
        double turnScale = 1.0 - std::min(fabs(turnError) / 20.0, 1.0);

        driveOutput *= turnScale;

        double leftPower = driveOutput - turnOutput;
        double rightPower = driveOutput + turnOutput;


        double maxMag = std::max(fabs(leftPower), fabs(rightPower));
        if (maxMag > 100) {
          double scale = 100 / maxMag;
          leftPower *= scale;
          rightPower *= scale;
        }

        leftPower  = std::clamp(leftPower, -max, max);
        rightPower = std::clamp(rightPower, -max, max);

        moveleft(-leftPower);
        moveright(-rightPower);

        //exit
        if (fabs(driveError) < 1 && fabs(turnError) < 2 && (leftPower + rightPower)/2 < 25) settleTime += 1;

        drivePrevError = driveError;
        turnPrevError = turnError;

        if (repeat > timeout * 50)
        {
          stop();
          break;
        }

        if (settleTime > 10)
        {
          stop();
          break;
        }

        //pros::c::screen_print(pros::E_TEXT_MEDIUM, repeat++, "pid: %f, %f, %f, %f", leftPower, rightPower, driveError, turnError);        

        pros::delay(20);
    }
}


void pidmove(float target_inch, float settletime_TOL, float timeout, float max, float E_TOL, float D_TOL)
{ // MOVE MOVE MOVE


  float lastError;
  float kp = 5.1; // for new robot
  float kd = 0.2; // for new robot
  float ki = 0.0; // for new roobot
  float s_error = 0;
  float spd;

  front_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  front_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  front_left_motor.set_zero_position(0);
  front_right_motor.set_zero_position(0);
  
  float settletime = 0;
  int repeat_limit = 2000000;
  int repeat = 0;

  while (true)
  {
    float error = target_inch - (((front_right_motor.get_position() + front_left_motor.get_position()) / 2) * 0.02127120025);
    float P = error * kp;
    float D = (error - lastError) * kd;
    s_error += error;
    s_error = fmin(s_error, 100);
    s_error = fmax(s_error, -100);
    // Brain.Screen.printAt(10,20,"error=%f",error);
    if (error * lastError < 0)
      s_error = 0;

    if (fabs(error) < E_TOL && spd < D_TOL)
    {
      settletime++;
    }

    if (repeat > 100 * timeout)
    {
      stop();
      break;
    }

    if (settletime > settletime_TOL)
    {
      stop();
      break;
    }

    float I = ki * s_error;
    lastError = error;
    spd = (P + D + I);
    move(std::clamp(spd, -max, max));
    repeat++;
    pros::c::delay(15);
  }
}