#ifndef AT_SIMPLE_SIMULATION_H
#define AT_SIMPLE_SIMULATION_H

#include "AtMCPoint.h"

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h> // for XYZVector
#include <Math/Vector4D.h>
#include <Math/Vector4Dfwd.h> // for PxPyPzEVector
#include <TClonesArray.h>
#include <TObject.h>

#include <functional> // for function
#include <map>
#include <memory>
#include <mutex>
#include <string>  // for string
#include <utility> // for pair
namespace AtTools {
class AtELossModel;
}
class TGeoVolume;
class AtSpaceChargeModel;

/**
 * Class for simulating simple events using AtELossModels.
 * Units in this class are MeV (energy), mm (distance) MeV/c (momentum).
 */
class AtSimpleSimulation {
protected:
   struct ParticleID {
      int A;
      int Z;

      bool operator<(const ParticleID &other) const;
   };
   using SpaceChargeModel = std::shared_ptr<AtSpaceChargeModel>;
   using ModelPtr = std::shared_ptr<AtTools::AtELossModel>;
   using XYZPoint = ROOT::Math::XYZPoint;
   using XYZVector = ROOT::Math::XYZVector;
   using PxPyPzEVector = ROOT::Math::PxPyPzEVector;

   std::map<ParticleID, ModelPtr> fModels;
   SpaceChargeModel fSCModel{nullptr};
   double fDistStep{1.}; // Distance step in mm for particles
   std::mutex fGeoMutex;

   // Variables to across an entire event
   static thread_local int fTrackID;
   static thread_local TClonesArray fMCPoints;

public:
   /**
    * Assumes that the IO manager has been initialized (it will attempt to construct the branch needed here).
    */
   AtSimpleSimulation(std::string geoFile);
   AtSimpleSimulation();
   AtSimpleSimulation(const AtSimpleSimulation &other) = default;
   ~AtSimpleSimulation() = default;

   void RegisterBranch(std::string branchName = "AtTpcPoint", bool pers = true);
   void AddModel(int Z, int A, ModelPtr model);
   void SetSpaceChargeModel(SpaceChargeModel model) { fSCModel = model; }
   void SetDistanceStep(double step) { fDistStep = step; } //<In mm

   void NewEvent();

   /**
    * Simulates a particle over a given distance and returns the position and momentum of the particle at the stoping
    * point. Uses Z and A to provide a model to the protected version of SimulateParticle (see below for more
    * information on the simulation).
    */
   std::pair<XYZPoint, PxPyPzEVector> SimulateParticle(
      int Z, int A, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
      std::function<bool(XYZPoint, PxPyPzEVector)> func = [](XYZPoint pos, PxPyPzEVector mom) { return true; });

   AtMCPoint &GetMcPoint(int i) { return dynamic_cast<AtMCPoint &>(*fMCPoints.At(i)); }
   int GetNumPoints() { return fMCPoints.GetEntries(); }
   TClonesArray &GetPointsArray() { return fMCPoints; }
   SpaceChargeModel GetSpaceChargeModel() { return fSCModel; }

protected:
   bool IsInVolume(const std::string &volName, const XYZPoint &point);
   std::string GetVolumeName(const XYZPoint &point);

   /**
    * Simulates a particle over a given distance and returns the position and momentum of the particle at the stoping
    * point. By default the particle will stop when it reaches the end of the TPC. A user defined function can test the
    * position and momentum of the particle for each time step and stop it when a given condition is met (such as a
    * depth in the TPC or an energy to stop at).
    */
   std::pair<XYZPoint, PxPyPzEVector> SimulateParticle(
      ModelPtr model, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
      std::function<bool(XYZPoint, PxPyPzEVector)> func = [](XYZPoint pos, PxPyPzEVector mom) { return true; });
   void AddHit(double ELoss, const XYZPoint &pos, const PxPyPzEVector &mom, double length);
   TGeoVolume *GetVolume(const XYZPoint &pos);
};

#endif // AT_SIMPLE_SIMULATION_H
