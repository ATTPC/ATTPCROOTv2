#ifndef AT_SIMPLE_SIMULATION_H
#define AT_SIMPLE_SIMULATION_H

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h> // for XYZVector
#include <Math/Vector4D.h>
#include <Math/Vector4Dfwd.h> // for PxPyPzEVector

#include <map>
#include <memory>
#include <string> // for string

class TClonesArray;
namespace AtTools {
class AtELossModel;
}
class TGeoVolume;

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
   using ModelPtr = std::shared_ptr<AtTools::AtELossModel>;
   using XYZPoint = ROOT::Math::XYZPoint;
   using XYZVector = ROOT::Math::XYZVector;
   using PxPyPzEVector = ROOT::Math::PxPyPzEVector;

   std::map<ParticleID, ModelPtr> fModels;
   TClonesArray *fMCPoints{nullptr};

   // Variables to across an entire event
   int fTrackID{0};

   double fDistStep{1.}; // Distance step in mm for particles

public:
   /**
    * Assumes that the IO manager has been initialized (it will attempt to construct the branch needed here).
    */
   AtSimpleSimulation(std::string geoFile);

   ~AtSimpleSimulation() = default;

   void Init(std::string branchName = "AtTpcPoint");
   void AddModel(int Z, int A, ModelPtr model);

   void NewEvent();
   void SimulateParticle(int Z, int A, const XYZPoint &iniPos, const PxPyPzEVector &iniMom);

protected:
   bool IsInVolume(const std::string &volName, const XYZPoint &point);
   std::string GetVolumeName(const XYZPoint &point);

   void SimulateParticle(ModelPtr model, const XYZPoint &iniPos, const PxPyPzEVector &iniMom);
   void AddHit(double ELoss, const XYZPoint &pos, const PxPyPzEVector &mom, double length);
   TGeoVolume *GetVolume(const XYZPoint &pos);
};

#endif // AT_SIMPLE_SIMULATION_H
