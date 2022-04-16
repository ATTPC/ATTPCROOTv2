#include "S800Calc.h"

#include "S800Calibration.h"

void S800Calc::ApplyCalibration(S800 *s800, S800Calibration *cal)
{
   cal->S800Calculate(s800, this);
}
