#ifndef ATMCFISSION_H
#define ATMCFISSION_H
#include "AtBaseEvent.h" // for AtBaseEvent
#include "AtMCFitter.h"  // for AtMCFitter, AtMCFitter::ClusterPtr
#include "AtMCResult.h"
#include "AtPatternEvent.h" // for AtPatternEvent

#include <Math/Point3D.h>     // for PositionVector3D
#include <Math/Point3Dfwd.h>  // for XYZPoint
#include <Math/Vector3D.h>    // for DisplacementVector3D
#include <Math/Vector3Dfwd.h> // for XYZVector
#include <Math/Vector4D.h>    // for LorentzVector
#include <Math/Vector4Dfwd.h> // for PxPyPzEVector
#include <TClonesArray.h>     // for TClonesArray

#include <array>      // for array
#include <functional> // for function
#include <vector>     // for vector
class AtFissionEvent;
namespace MCFitter {
struct Ion {
   int Z;
   int A;
};

class AtMCFission : public AtMCFitter {
protected:
   using XYZPoint = ROOT::Math::XYZPoint;
   using XYZVector = ROOT::Math::XYZVector;
   using XYZEVector = ROOT::Math::PxPyPzEVector;
   using ObjectiveFuncCharge =
      std::function<double(const std::vector<double> &exp, const std::vector<double> &sim, const double *par)>;
   Ion fCN{85, 204};

   // Range of valid guesses for Z (not inclusive)
   int fZmin = 26;
   int fZmax = 59;

   /// Average beam direction. We will sample the beam direction as deviations from this vector.
   /// Default value taken from Joe's plots in the overleaf on space charge
   XYZVector fNominalBeamDir{11.464, 3.754, 1000};

   /// Objective function to minimize the difference between A*sim and exp charge curves.
   ObjectiveFuncCharge fObjCharge{ObjectiveChargeChi2};

   float fAmp = 1;
   bool fFitAmp = true;

public:
   AtMCFission(SimPtr sim, ClusterPtr cluster, PulsePtr pulse) : AtMCFitter(sim, cluster, pulse) {}
   virtual ~AtMCFission() = default;
   void SetCN(Ion cn) { fCN = cn; }
   void SetChargeObjective(ObjectiveFuncCharge obj) { fObjCharge = obj; }
   void SetAmp(float amp);
   void SetZRange(int Zmin, int Zmax)
   {
      fZmin = Zmin;
      fZmax = Zmax;
   }

   // Old static functinos used to test different objective functions in early analysis
   static double ObjectiveChargeChi2(const std::vector<double> &exp, const std::vector<double> &sim, const double *par);
   static double
   ObjectiveChargeChi2Norm(const std::vector<double> &exp, const std::vector<double> &sim, const double *par);
   static double
   ObjectiveChargeDiff2(const std::vector<double> &exp, const std::vector<double> &sim, const double *par);

protected:
   virtual void CreateParamDistros() override;
   virtual void SetParamDistributions(const AtPatternEvent &event) override;
   virtual double ObjectiveFunction(const AtBaseEvent &expEvent, int SimEventID, AtMCResult &definition) override;
   virtual TClonesArray SimulateEvent(AtMCResult &definition) override;
   virtual AtMCResult DefineEvent() override;

protected:
   // Actual objective functions used (position is just for diagnostics, not used in minimization)
   double ObjectiveCharge(const AtFissionEvent &expEvent, int SimEventID, AtMCResult &def);
   double ObjectiveCharge(const std::array<std::vector<double>, 2> &exp, const std::array<std::vector<double>, 2> &sim,
                          AtMCResult &definition);
   double ObjectivePositionPads(const AtFissionEvent &expEvent, int SimEventID);

   // Objective functions not in use
   double ObjectivePosition(const AtFissionEvent &expEvent, int SimEventID);
   double ObjectiveChargePads(const AtFissionEvent &expEvent, int SimEventID, AtMCResult &def);

   XYZPoint GetVertex(AtMCResult &);
   std::array<Ion, 2> GetFragmentSpecies(AtMCResult &, const Ion &CN);

   XYZVector GetBeamDir(AtMCResult &, const std::array<XYZVector, 2> &ffDir, double pTrans);
   XYZVector GetBeamDirSample(AtMCResult &, const std::array<XYZVector, 2> &ffDir);
   XYZVector GetBeamDirSameV(AtMCResult &, const std::array<XYZVector, 2> &ffDir);
   std::array<XYZVector, 2> GetMomDirLab(AtMCResult &);

   void SetMomMagnitude(std::array<XYZVector, 2> &mom, double pTrans);

   double ObjectivePosition(double uE, double sE, double uO, double sO);
   double ObjectivePosition4(double uE, double sE, double uO, double sO);
   double ObjectivePosition3(double uE, double sE, double uO, double sO);
   double ObjectivePosition2(double uE, double sE, double uO, double sO);

   // Returns the average total kinetic energy from viola systematics in MeV
   static double violaEn(int A, int Z) { return 0.1189 * Z * Z / std::pow(A, 1.0 / 3.0) + 7.3; }
};

} // namespace MCFitter

#endif // ATMCFISSION_H
