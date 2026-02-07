float InchToEncoderunit(float);

void pidMoveold(float, float, float, float max = 70);

void pidTurnRel(float, float, float, float max = 50);

void pidTurnAbs(float, float, float, float max = 100);

void pidswingAbs(float, float, float, bool);

void bwallMove(float, float, int, float, float max = 50);

void pidGyro(double, double, double, double max = 70, double E_TOL = 1, double D_TOL = 25, double _settle = 15, float _turnscale = 15);

void pidWallMove(float, float, float _wall, float, float max = 70);

void pidwallGyro(double, double, float _wall, double, double max = 70);

void pidFrontWallGyro(double , double , double _wall, double , double max = 70);

void pidmove(float, float, float, float max = 100, float E_TOL = 1, float D_TOL = 25);