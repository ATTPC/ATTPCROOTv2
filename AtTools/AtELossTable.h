#ifndef ATELOSSTABLE_H
#define ATELOSSTABLE_H
// IWYU pragma: no_include <ext/alloc_traits.h>

#include "AtELossModel.h"
#include "AtSpline.h"

#include <string>
#include <vector>

namespace AtTools {

class AtELossTable : public AtELossModel {
protected:
   tk::spline fdXdE;

   double fDistErr{1e-4};

public:
   AtELossTable(double density = 0) : AtELossModel(density) {}
   AtELossTable(const std::vector<double> &energy, const std::vector<double> &dEdX, double density = 0);

   /// Set error in particle range when calculating energy losses.
   void SetDistanceError(double val) { fDistErr = val; }
   void LoadSrimTable(std::string fileName);
   /**
    * Load energy loss table from LISE++ (export of stopping power graph to file).
    * @param[in] fileName Name of text file.
    * @param[in] mass Mass of the fragment in units of u.
    * @param[in] density Density of target in g/cm^3. If positive, units of table are assumed to be MeV/mg/cm^2. If not
    * positive units of the table are assumed to be MeV/um and the target density is taken to be |density|.
    *@param[in] column The column in the table to use as the energy model (default LISE is 2).
    */
   void LoadLiseTable(std::string fileName, double mass, double density, int column = 2);

   virtual double GetdEdx(double energy) const override;
   virtual double GetRange(double energyIni, double energyFin = 0) const override;
   virtual double GetEnergyLoss(double energyIni, double distance) const override
   {
      return energyIni - GetEnergy(energyIni, distance);
   }
   virtual double GetEnergy(double energyIni, double distance) const override;

   [[deprecated]] double GetEnergyOld(double energyIni, double distance) const;

private:
   void LoadTable(const std::vector<double> &energy, const std::vector<double> &dEdX);
   double GetUnitConversion(const std::string &unit);
};

} // namespace AtTools
#endif // #ifndef ATELOSSTABLE_H
