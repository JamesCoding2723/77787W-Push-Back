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
  float kp = 2.8; // for new robot
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
  float kp = 0.95;      // for new robot
  float kd = 0.2;      // for new robot
  float ki = 0.15;     // for new roobot
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

float start_heading = 0;

void pidTurnAbs(float target, float rotate_tolocal, float timeout, float max)
{ // ROTATE with tolerate variable
  float pTol = rotate_tolocal;
  float dTol = rotate_tolocal;
  float lastError;
  float error;
  float kp = 0.8;      // for new robot
  float kd = 0.4;      // for new robot
  float ki = 0.02;     // for new robot
  //float spd_ratio = 2; // 0.5
  float s_error = 0;
  int n = 0;
  int repeat = 0;
  while (true)
  {
    if (std::abs(target - std::fmod(imu.get_heading(), 360)) < std::abs(target - std::fmod(360 - imu.get_heading(), 360)))
    {
      error = target - std::fmod(imu.get_heading(), 360);
    }
    else
    {
      error = target - std::fmod(360 - imu.get_heading(), 360);
    }
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

    if (repeat > timeout)
    {
      stop();
      break;
    }

    float pidspd = (P + D + I);

    if (std::abs(pidspd) < 23)
      pidspd = sign(pidspd) * 23;

    turn(std::clamp(pidspd, -max, max));
    // pros::c::screen_print(pros::E_TEXT_MEDIUM, n++, "pid: %f, %f, %f", (P+D+I), imu.get_rotation(), error);
    repeat++;

    lastError = error;
    pros::c::delay(10);
  }
}



void pidswingRel(float target, float rotate_tolocal, float timeout, bool side)
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
  //target = imu.get_rotation() + target;

  while (true)
  {

    float error = target - imu.get_rotation();
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


/*void bwallMove(float target_inch, float tolerence_inch, int wall, float timeout, float max)
{ // MOVE MOVE MOVE

  // float target = target_inch
  // float tolerence = InchToEncoderunit(tolerence_inch);
  float lastError;
  float kp = 2.8; // for new robot
  float kd = 0.2; // for new robot
  float ki = 0.0; // for new roobot
  float s_error = 0;
  int n = 0;
  int line_number = 1;
  float speed_ratio = 2.2, spd;
  
  int repeat_limit = 2000000;
  int repeat = 0;

  while (true)
  {
    float error = target_inch - wallpos(wall);
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
    spd = (P + D + I) * speed_ratio;
    if (std::abs(spd) < 15)
      spd = sign(spd) * 15;
    
    move(std::clamp(-spd, -max, max));

    pros::c::screen_print(pros::E_TEXT_MEDIUM, n++, "pid: %f, %f, %f", (P+D+I), wallpos(wall), bdistance.get());
    repeat++;
    pros::c::delay(10);
  }
}*/


void drivePIDGyro(double targetInches, double targetHeading, double timeout, double settleTime, double max) {

    // Reset sensors
    front_right_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    front_left_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    front_left_motor.set_zero_position(0);
    front_right_motor.set_zero_position(0);


    //drive PID
    double kP_drive = 2.8;
    double kI_drive = 0.0;
    double kD_drive = 0.2;
    double driveS_error;

    //turn PID
    double kP_turn = 0.95;
    double kI_turn = 0.15;
    double kD_turn = 0.2;
    double turnS_error;

    double driveError = 0, drivePrevError = 0;
    double turnError = 0, turnPrevError = 0;

    int repeat = 0;
    while (settleTime < 10) {
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
        if (driveError * turnS_error < 0)
          driveS_error = 0; // 3
        float turnI = kI_turn * turnS_error;

        double turnOutput = turnP + turnI + turnD;

        //outputs-----------------------------------------
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
        if (fabs(driveError) < 15 && fabs(turnError) < 2 && (leftPower + rightPower)/2 < 25)
            settleTime++;
        else
            settleTime = 0;

        drivePrevError = driveError;
        turnPrevError = turnError;

        if (repeat > timeout * 50)
        {
          stop();
          break;
        }

        pros::delay(20);
    }
    stop();
}
