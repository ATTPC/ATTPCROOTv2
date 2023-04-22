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

#include <array> // for array
#include <cmath> // for sqrt

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
   Ion fCN{85, 204};

   /// Average beam direction. We will sample the beam direction as deviations from this vector.
   /// Default value taken from Joe's plots in the overleaf on space charge
   XYZVector fNominalBeamDir{11.464, 3.754, 1000};

public:
   AtMCFission(SimPtr sim, ClusterPtr cluster, PulsePtr pulse) : AtMCFitter(sim, cluster, pulse) {}
   virtual ~AtMCFission() = default;
   void SetCN(Ion cn) { fCN = cn; }

protected:
   virtual void CreateParamDistros() override;
   virtual void SetParamDistributions(const AtPatternEvent &event) override;
   virtual double ObjectiveFunction(const AtBaseEvent &expEvent, int SimEventID, AtMCResult &definition) override;
   virtual TClonesArray SimulateEvent(AtMCResult &definition) override;
   virtual AtMCResult DefineEvent() override;

protected:
   double ObjectivePosition(const AtFissionEvent &expEvent, int SimEventID);
   double ObjectiveCharge(const AtFissionEvent &expEvent, int SimEventID, AtMCResult &def);

   XYZPoint GetVertex(AtMCResult &);
   std::array<Ion, 2> GetFragmentSpecies(AtMCResult &, const Ion &CN);

   XYZVector GetBeamDir(AtMCResult &, const std::array<XYZVector, 2> &ffDir);
   XYZVector GetBeamDirSameV(AtMCResult &, const std::array<XYZVector, 2> &ffDir);
   std::array<XYZVector, 2> GetMomDirLab(AtMCResult &);

   void SetMomMagnitude(std::array<XYZVector, 2> &mom, double pTrans);

   double ObjectivePosition(double uE, double sE, double uO, double sO);
   double ObjectivePosition4(double uE, double sE, double uO, double sO);
   double ObjectivePosition3(double uE, double sE, double uO, double sO);
   double ObjectivePosition2(double uE, double sE, double uO, double sO);

   double ObjectiveCharge(const std::array<std::vector<double>, 2> &exp, const std::array<std::vector<double>, 2> &sim,
                          AtMCResult &definition);

   // Returns the average total kinetic energy from viola systematics in MeV
   static double violaEn(int A, int Z) { return 0.1189 * Z * Z / std::pow(A, 1.0 / 3.0) + 7.3; }
};

} // namespace MCFitter

#endif // ATMCFISSION_H
