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



int sign (float);

extern int ftintakespd();

void  setftintakespd(float);

void ftintake();

extern float backintakespd;


