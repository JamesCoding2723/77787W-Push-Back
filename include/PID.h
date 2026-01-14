float InchToEncoderunit(float);

void pidMoveold(float, float, float, float max = 70);

void pidTurnRel(float, float, float, float max = 50);

void pidTurnAbs(float, float, float, float max = 100);

void pidswingRel(float, float, float, bool);

void bwallMove(float, float, int, float, float max = 50);

void pidGyro(double, double, double, double max = 70);