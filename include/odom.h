
extern double start_heading;

extern double x;

extern double y;

extern float lastForwardPos;
extern float lastHeading;
extern float lastPitch;
extern float currForwardPos, currHeading;

void GPStracking();

double getX();
double getY();

void setX(double);
void setY(double);

void GPSmove(float, float);