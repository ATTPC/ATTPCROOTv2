#ifndef AT_STANDALONE_SIMULATION_H
#define AT_STANDALONE_SIMULATION_H

#include "AtMCPoint.h"
#include "AtSimpleSimulation.h"

#include <Math/Point3Dfwd.h>
#include <Math/Vector4Dfwd.h>
#include <TClonesArray.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>

class AtSpaceChargeModel;

/**
 * Standalone simulation wrapper that owns an AtSimpleSimulation transport engine
 * and adds hit recording to a thread-local TClonesArray.
 *
 * Used by AtMCFitter, AtMCFission, and similar callers that manage their own
 * event loop rather than running inside the FairRoot simulation pipeline.
 */
class AtStandaloneSimulation {
public:
   using SpaceChargeModel = std::shared_ptr<AtSpaceChargeModel>;
   using XYZPoint = ROOT::Math::XYZPoint;
   using PxPyPzEVector = ROOT::Math::PxPyPzEVector;

   explicit AtStandaloneSimulation(std::unique_ptr<AtSimpleSimulation> engine);
   ~AtStandaloneSimulation() = default;

   /// Access the underlying transport engine for configuration (models, fields, etc.)
   AtSimpleSimulation *GetEngine() { return fEngine.get(); }

   void SetSpaceChargeModel(SpaceChargeModel model) { fSCModel = std::move(model); }
   SpaceChargeModel GetSpaceChargeModel() { return fSCModel; }
   void SetStandaloneVolumeName(const std::string &name) { fVolumeName = name; }

   void RegisterBranch(std::string branchName = "AtTpcPoint", bool pers = true);
   void NewEvent();

   /**
    * Simulate a particle within the configured standalone volume.
    * Hits are recorded to the thread-local TClonesArray.
    */
   std::pair<XYZPoint, PxPyPzEVector> SimulateParticle(
      int Z, int A, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
      std::function<bool(XYZPoint, PxPyPzEVector)> func = [](XYZPoint, PxPyPzEVector) { return true; });

   AtMCPoint &GetMcPoint(int i) { return dynamic_cast<AtMCPoint &>(*fMCPoints.At(i)); }
   int GetNumPoints() { return fMCPoints.GetEntries(); }
   TClonesArray &GetPointsArray() { return fMCPoints; }

private:
   std::unique_ptr<AtSimpleSimulation> fEngine;
   SpaceChargeModel fSCModel{nullptr};
   std::string fVolumeName{"drift_volume"};

   static thread_local int fTrackID;
   static thread_local TClonesArray fMCPoints;

   void AddHit(double ELoss, const XYZPoint &pos, const PxPyPzEVector &mom, double length);
};

#endif // AT_STANDALONE_SIMULATION_H
