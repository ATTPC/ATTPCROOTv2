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
class AtELossModelFactory;
} // namespace AtTools
class TGeoVolume;
class TGeoManager;
class TGeoNavigator;
class AtSpaceChargeModel;

/**
 * Class for simulating simple events using AtELossModels.
 * Units in this class are MeV (energy), mm (distance) MeV/c (momentum).
 *
 * When E/B fields are set (non-zero), the simulation uses AtPropagator (Lorentz force + RK4
 * adaptive stepping) to produce curved tracks. When both fields are zero the existing
 * straight-line fast path is used.
 */
class AtSimpleSimulation {
protected:
   struct ParticleID {
      int A;
      int Z;

      bool operator<(const ParticleID &other) const;
   };

   struct ParticleInfo {
      std::shared_ptr<AtTools::AtELossModel> model;
      double charge; ///< Particle charge in Coulombs
      double mass;   ///< Particle mass in MeV/c²
   };

   using SpaceChargeModel = std::shared_ptr<AtSpaceChargeModel>;
   using ModelPtr = std::shared_ptr<AtTools::AtELossModel>;
   using XYZPoint = ROOT::Math::XYZPoint;
   using XYZVector = ROOT::Math::XYZVector;
   using PxPyPzEVector = ROOT::Math::PxPyPzEVector;

   std::map<ParticleID, ParticleInfo> fModels;
   std::shared_ptr<AtTools::AtELossModelFactory> fModelFactory{nullptr};
   SpaceChargeModel fSCModel{nullptr};
   double fDistStep{1.}; // Distance step in mm for straight-line propagation
   std::mutex fGeoMutex;
   TGeoManager *fGeoManager{nullptr};
   TGeoNavigator *fNavigator{nullptr};

   XYZVector fEField{0, 0, 0}; ///< Electric field in V/m (used by AtPropagator)
   XYZVector fBField{0, 0, 0}; ///< Magnetic field in T (used by AtPropagator)
   double fMaxPropStep{1e-3};  ///< Max step size in m for the adaptive stepper (default 1 mm)
   double fCurvedStopTol{0.1}; ///< Curved-track stop tolerance in MeV; avoids pathological late stopping tails
   std::string fStandaloneVolumeName{"drift_volume"}; ///< Volume name for standalone SimulateParticle hit recording

   // Variables to across an entire event
   static thread_local int fTrackID;
   static thread_local TClonesArray fMCPoints;

public:
   struct TransportStep {
      int trackID = -1;
      int pdg = 0;
      std::string preVolumeName;
      std::string postVolumeName;
      double energyLoss = 0.0;  // MeV
      double length = 0.0;      // mm
      double trackMass = 0.0;   // MeV/c^2
      XYZPoint prePosition;
      XYZPoint postPosition;
      PxPyPzEVector preMomentum;
      PxPyPzEVector postMomentum;
   };

   using StepCallback = std::function<bool(const TransportStep &)>;

   // ---- Construction ----

   /**
    * Assumes that the IO manager has been initialized (it will attempt to construct the branch needed here).
    */
   AtSimpleSimulation(std::string geoFile);
   AtSimpleSimulation();
   AtSimpleSimulation(const AtSimpleSimulation &other) = delete; // Implicitly deleted because of std::mutex
   ~AtSimpleSimulation() = default;

   // ---- Standalone hit-writing API ----
   // Used by AtMCFission and direct callers. These methods manage a thread-local
   // TClonesArray of AtMCPoints and write hits directly during transport.

   void RegisterBranch(std::string branchName = "AtTpcPoint", bool pers = true);

   // ---- Transport API ----
   // Shared by both standalone and detector-coupled paths.

   /**
    * Register an energy loss model for a particle species. Charge is derived as Z*e and
    * mass as A * 931.494 MeV/c². Use the overload with massAmu for higher accuracy.
    */
   void AddModel(int Z, int A, ModelPtr model);

   /**
    * Register an energy loss model with an explicit nuclear mass (in amu).
    */
   void AddModel(int Z, int A, ModelPtr model, double massAmu);

   /**
    * Set a model factory for automatic energy loss model creation.
    * When set, if a particle species (Z, A) is encountered without a registered model,
    * the factory will be used to create one from the geometry material at the particle's position.
    */
   void SetModelFactory(std::shared_ptr<AtTools::AtELossModelFactory> factory) { fModelFactory = std::move(factory); }

   void SetSpaceChargeModel(SpaceChargeModel model) { fSCModel = model; }
   void SetDistanceStep(double step) { fDistStep = step; } ///< Step size in mm (straight-line path)

   void SetElectricField(XYZVector eField) { fEField = eField; } ///< Electric field in V/m
   void SetMagneticField(XYZVector bField) { fBField = bField; } ///< Magnetic field in T
   /// Maximum step size (m) for the RK4 adaptive stepper in curved-track mode (default: 1e-3 m = 1 mm).
   void SetMaxPropagationStep(double stepM) { fMaxPropStep = stepM; }
   void SetCurvedStopTolerance(double stopTolMeV) { fCurvedStopTol = stopTolMeV; }
   void SetStandaloneVolumeName(const std::string &name) { fStandaloneVolumeName = name; }

   void NewEvent();

   /**
    * Simulates a particle over a given distance and returns the position and momentum of the particle at the stopping
    * point. Uses Z and A to provide a model to the protected version of SimulateParticle.
    * When E/B fields are non-zero, AtPropagator is used for curved-track propagation.
    */
   std::pair<XYZPoint, PxPyPzEVector> SimulateParticle(
      int Z, int A, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
      std::function<bool(XYZPoint, PxPyPzEVector)> func = [](XYZPoint pos, PxPyPzEVector mom) { return true; });

   // ---- Detector-coupled transport API ----
   // Used by AtSimpleSimulationTask for Geant4 drop-in replacement. Transport steps are
   // delivered via callback; hit recording is handled by the detector (AtTpc).

   /**
    * Transport a particle through the loaded geometry without writing detector hits.
    * This is intended for detector-coupled adapters that want transport state but keep hit
    * semantics in the detector code.
    */
   std::pair<XYZPoint, PxPyPzEVector> TransportParticle(int Z, int A, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
                                                        StepCallback callback);

   AtMCPoint &GetMcPoint(int i) { return dynamic_cast<AtMCPoint &>(*fMCPoints.At(i)); }
   int GetNumPoints() { return fMCPoints.GetEntries(); }
   TClonesArray &GetPointsArray() { return fMCPoints; }
   SpaceChargeModel GetSpaceChargeModel() { return fSCModel; }
   bool IsInsideGeometry(const XYZPoint &point) { return GetVolume(point) != nullptr; }
   std::string GetVolumeNameAt(const XYZPoint &point) { return GetVolumeName(point); }

protected:
   bool IsInVolume(const std::string &volName, const XYZPoint &point);
   std::string GetVolumeName(const XYZPoint &point);

   /**
    * Core transport loop. Propagates a particle through the geometry, invoking callback at each step.
    * Continues while the particle is inside the geometry (GetVolume != nullptr) and KE > threshold.
    * The callback controls early stopping by returning false.
    * Selects curved-track (RK4) or straight-line path based on field settings.
    */
   std::pair<XYZPoint, PxPyPzEVector> PropagateParticle(const ParticleInfo &info, int pdg, const XYZPoint &iniPos,
                                                        const PxPyPzEVector &iniMom, const StepCallback &callback);

   void AddHit(double ELoss, const XYZPoint &pos, const PxPyPzEVector &mom, double length);
   TGeoVolume *GetVolume(const XYZPoint &pos);

   /// Attempt to auto-create an energy loss model using fModelFactory and the geometry material at pos.
   void TryAutoCreateModel(int Z, int A, const XYZPoint &pos);
};

#endif // AT_SIMPLE_SIMULATION_H
