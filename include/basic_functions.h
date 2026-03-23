#include <cmath>

int sign(float);

extern int intakespd1;

extern int intakespd2;

extern int intake2spd;

void setintakespd(float);

void setintake3spd(float);

void setintakespddiff(float, float);

void setintake2spd(float);

void intake();

void intake2();

void intake3();

void moveleft(float);

void moveright(float);

void move(float);

double motorpos();

void resetmotorpos();

void turn(float);

void stop();

void moveDis(float, float);

void moveForSec(float, bool, float);

extern bool jeminmechtoggle;

void jeminmecht();

extern bool jeminloadertoggle;

void jeminloadert();

extern bool jemintaketoggle;

void jemintaket();

extern bool jeminwingtoggle;

void jeminwingt();

extern bool jeminchoptoggle;

void jeminchopt();

extern bool jeminparktoggle;

void jeminparkt();

void moveforward(float, bool, float);

void wait(float);

float rad2deg(float);

float deg2rad(float);

void setwall_heading(float);

double getwallpos(float wall_heading);

void imu_display_task(void*);

double getfrontwallpos(float wall_heading);

void setjeminparallel(float _time, bool _active);

void jeminloaderd();

extern bool side;

extern bool sort_on;

extern bool Msort_on;

extern bool is_sorting;

void color_sort();

void motor_sort();

extern bool storing;

void store();

void midscore();

extern bool score_on;

void score();

void skillscore();

extern bool lowgoal_on;

void lowscore();

void auton_selector_task(void*);

extern int auton_selection;

extern const int max_autons;

extern bool auton_locked;