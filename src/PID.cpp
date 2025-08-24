#include "pros/rtos.h"
#include "pros/screen.h"
#include <cmath>
#include "robot_config.h"
#include "basic_functions.h"

float InchToEncoderunit(float distance)
{
  return ((distance * 360) / 10.21017) * (4 / 3); // gear ratio is 4:3
}

void pidMoveold(float target_inch, float tolerence_inch, float timeout)
{ // MOVE MOVE MOVE

  // float target = target_inch
  // float tolerence = InchToEncoderunit(tolerence_inch);
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

  while (true)
  {
    float error = target_inch - (vertical_encoder.get_position() * 0.0002399827721);
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
    move(spd);
    repeat++;
    pros::c::delay(10);
    // pros::c::screen_print(pros::E_TEXT_MEDIUM, line_number++, "error: %f", vertical_encoder.get_position());
  }
}

void pidTurnRel(float target, float rotate_tolocal, float timeout)
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
  // imu.set_rotation(0);
  target = imu.get_rotation() + target;

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

    if (repeat > timeout)
    {
      stop();
      break;
    }

    float pidspd = (P + D + I);

    if (std::abs(pidspd) < 23)
      pidspd = sign(pidspd) * 23;

    turn(pidspd);
    // pros::c::screen_print(pros::E_TEXT_MEDIUM, n++, "pid: %f, %f, %f", (P+D+I), imu.get_rotation(), error);
    repeat++;

    lastError = error;
    pros::c::delay(10);
  }
}

float start_heading = 0;

void pidTurnAbs(float target, float rotate_tolocal, float timeout)
{ // ROTATE with tolerate variable
  float pTol = rotate_tolocal;
  float dTol = rotate_tolocal;
  float lastError;
  float error;
  float kp = 0.8;      // for new robot
  float kd = 0.4;      // for new robot
  float ki = 0.02;     // for new roobot
  float spd_ratio = 2; // 0.5
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

    turn(pidspd);
    // pros::c::screen_print(pros::E_TEXT_MEDIUM, n++, "pid: %f, %f, %f", (P+D+I), imu.get_rotation(), error);
    repeat++;

    lastError = error;
    pros::c::delay(10);
  }
}