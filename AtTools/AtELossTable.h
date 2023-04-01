#ifndef ATELOSSTABLE_H
#define ATELOSSTABLE_H

#include "AtELossModel.h"
#include "AtSpline.h"

#include <vector>
namespace AtTools {

class AtELossTable : public AtELossModel {
protected:
   tk::spline fdXdE;

public:
   AtELossTable(const std::vector<double> &energy, const std::vector<double> &dEdX, double density = 0);

   virtual double GetdEdx(double energy) const override;
   virtual double GetRange(double energyIni, double energyFin = 0) const override;
   virtual double GetEnergyLoss(double energyIni, double distance) const override
   {
      return energyIni - GetEnergy(energyIni, distance);
   }
   virtual double GetEnergy(double energyIni, double distance) const override;
};

} // namespace AtTools
#endif //#ifndef ATELOSSTABLE_H
