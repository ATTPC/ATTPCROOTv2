#ifndef AT_SIMPLE_SIMULATION_H
#define AT_SIMPLE_SIMULATION_H

#include <Math/Point3D.h>
#include <Math/Vector3D.h>
#include <TClonesArray.h>

#include <map>
#include <memory>
class AtELossModel;

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
   using ModelPtr = std::shared_ptr<AtELossModel>;
   using XYZPoint = ROOT::Math::XYZPoint;
   using XYZVector = ROOT::Math::XYZVector;

   std::map<ParticleID, ModelPtr> fModels;
   TClonesArray fMCPoints;

   // Variables to across an entire event
   int fTrackID;

   double fDistStep{1.}; // Distance step in mm for particles

public:
   /**
    * Assumes that the IO manager has been initialized (it will attempt to construct the branch needed here).
    */
   AtSimpleSimulation(bool isPersistant = true, std::string branchName = "AtTpcPoint");

   ~AtSimpleSimulation() = default;

   void AddModel(int Z, int A, ModelPtr model);

   void NewEvent();
   void SimulateParticle(int Z, int A, const XYZPoint &iniPos, const XYZVector &iniMom);

protected:
   bool IsInVolume(const std::string &volName, const XYZPoint &point);
   std::string GetVolumeName(const XYZPoint &point);

   void SimulateParticle(ModelPtr model, const XYZPoint &iniPos, const XYZVector &iniMom);
   void AddHit(double ELoss, const XYZPoint &pos, const XYZVector &mom, double length);
};

#endif // AT_SIMPLE_SIMULATION_H
