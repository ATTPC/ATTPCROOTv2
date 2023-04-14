#ifndef ATMCFISSION_H
#define ATMCFISSION_H

#include "AtMCFitter.h"

#include <Math/AxisAngle.h>
#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <Math/Rotation3D.h>
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h> // for XYZVector
#include <Math/Vector4D.h>
#include <Math/VectorUtil.h>

#include <vector>
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
   virtual void SetParamsFromEvent(const AtPatternEvent &event) override;
   virtual double ObjectiveFunction(const AtBaseEvent &expEvent, int SimEventID) override;
   virtual void SimulateEvent() override;

public:
   XYZPoint SampleVertex();
   std::array<Ion, 2> SampleFragmentSpecies();
   XYZVector SampleBeamDir();
   std::array<XYZVector, 2> SampleMomDir();
   void SetMomMagnitude(XYZVector beamDir, std::array<XYZVector, 2> &mom, const std::array<double, 2> &pTrans);

   // Returns the average total kinetic energy from viola systematics in MeV
   static double violaEn(int A, int Z) { return 0.1189 * Z * Z / std::pow(A, 1.0 / 3.0) + 7.3; }

   static double GetGamma(double KE, double m1, double m2);
   static double GetVelocity(double gamma);
   static double GetBeta(double gamma);
   static double GetRelMom(double gamma, double mass);
   static double AtoE(double Amu);
   static double EtoA(double mass) { return mass / 931.5; }

   template <class Vector>
   static XYZEVector Get4Vector(Vector mom, double m)
   {
      return {mom.X(), mom.Y(), mom.Z(), std::sqrt(mom.Mag2() + m * m)};
   }
};

} // namespace MCFitter

#endif // ATMCFISSION_H
