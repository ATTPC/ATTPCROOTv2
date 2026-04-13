#ifndef AT_SIMPLE_SIMULATION_H
#define AT_SIMPLE_SIMULATION_H

#include "AtMCPoint.h"
#include "AtSimTransport.h"

#include <Math/Point3Dfwd.h>
#include <Math/Vector3Dfwd.h>
#include <Math/Vector4Dfwd.h>
#include <TClonesArray.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace AtTools {
class AtELossManager;
class AtELossModel;
} // namespace AtTools
class AtSpaceChargeModel;

/**
 * Standalone user-facing simulation wrapper: owns an AtSimTransport transport engine
 * and records detector hits into a thread-local TClonesArray.
 *
 * Used by AtMCFitter, AtMCFission, and standalone analysis macros that manage their own
 * event loop rather than running inside the FairRoot simulation pipeline. For pipeline
 * integration (FairRoot event loop + AtTpc detector coupling), use AtSimTransportTask.
 *
 * Most configuration calls (AddModel, SetMagneticField, SetMaxStep, ...) are thin
 * forwarders to the embedded AtSimTransport so historical single-class macros work
 * without reaching through GetEngine().
 */
class AtSimpleSimulation {
public:
   using ModelPtr = std::shared_ptr<AtTools::AtELossModel>;
   using SpaceChargeModel = std::shared_ptr<AtSpaceChargeModel>;
   using XYZPoint = ROOT::Math::XYZPoint;
   using XYZVector = ROOT::Math::XYZVector;
   using PxPyPzEVector = ROOT::Math::PxPyPzEVector;

   // ---- Construction ----

   AtSimpleSimulation();
   explicit AtSimpleSimulation(std::unique_ptr<AtSimTransport> engine);
   explicit AtSimpleSimulation(const std::string &geoFile);
   AtSimpleSimulation(const std::string &geoFile, std::shared_ptr<AtTools::AtELossManager> manager);
   explicit AtSimpleSimulation(std::shared_ptr<AtTools::AtELossManager> manager);
   ~AtSimpleSimulation() = default;

   // ---- Transport-engine access and thin forwarders ----

   AtSimTransport *GetEngine() { return fEngine.get(); }

   void AddModel(int Z, int A, ModelPtr model) { fEngine->AddModel(Z, A, std::move(model)); }
   void AddModel(int Z, int A, const std::string &materialName, ModelPtr model)
   {
      fEngine->AddModel(Z, A, materialName, std::move(model));
   }
   /// Legacy signature from the develop-era API; `massAmu` is ignored (mass is now taken
   /// from the TParticlePDG database / model itself).
   void AddModel(int Z, int A, ModelPtr model, double /*massAmu*/) { AddModel(Z, A, std::move(model)); }

   void SetManager(std::shared_ptr<AtTools::AtELossManager> manager) { fEngine->SetManager(std::move(manager)); }
   AtTools::AtELossManager *GetManager() { return fEngine->GetManager(); }

   void SetMagneticField(const XYZVector &bField) { fEngine->SetMagneticField(bField); }
   void SetElectricField(const XYZVector &eField) { fEngine->SetElectricField(eField); }
   void SetMaxStep(double stepMm) { fEngine->SetMaxStep(stepMm); }
   void SetDistanceStep(double stepMm) { fEngine->SetMaxStep(stepMm); }
   void SetStopTolerance(double stopTolMeV) { fEngine->SetStopTolerance(stopTolMeV); }
   void SetMaxTransportSteps(int maxSteps) { fEngine->SetMaxTransportSteps(maxSteps); }

   // ---- Hit-recording configuration ----

   void SetSpaceChargeModel(SpaceChargeModel model) { fSCModel = std::move(model); }
   SpaceChargeModel GetSpaceChargeModel() { return fSCModel; }
   void SetStandaloneVolumeName(const std::string &name) { fVolumeName = name; }

   // ---- Event lifecycle ----

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
   std::unique_ptr<AtSimTransport> fEngine;
   SpaceChargeModel fSCModel{nullptr};
   std::string fVolumeName{"drift_volume"};

   static thread_local int fTrackID;
   static thread_local TClonesArray fMCPoints;

   void AddHit(double ELoss, const XYZPoint &pos, const PxPyPzEVector &mom, double length);
};

#endif // AT_SIMPLE_SIMULATION_H
