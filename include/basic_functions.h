#include <cmath>

int sign(float);

extern int intakespd1;

extern int intakespd2;

extern int intake2spd;

void setintakespd(float);

void setintakespddiff(float, float);

void setintake2spd(float);

void intake();

void intake2();

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

void moveforward(float, bool, float);

void wait(float);

float rad2deg(float);

float deg2rad(float);