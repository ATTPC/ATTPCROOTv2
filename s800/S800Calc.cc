#include "S800Calibration.hh"
#include "S800Calc.hh"

void S800Calc::ApplyCalibration(S800* s800, S800Calibration* cal) {
   cal->S800Calculate(s800,this);
}
