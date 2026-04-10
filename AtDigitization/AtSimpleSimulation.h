#ifndef AT_SIMPLE_SIMULATION_H
#define AT_SIMPLE_SIMULATION_H

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h> // for XYZVector
#include <Math/Vector4D.h>
#include <Math/Vector4Dfwd.h> // for PxPyPzEVector

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

/**
 * Transport engine for simulating particles using AtELossModels.
 * Units in this class are MeV (energy), mm (distance) MeV/c (momentum).
 *
 * When B fields are set (non-zero) or a field function is provided, the simulation uses
 * AtPropagator (Lorentz force + RK4 adaptive stepping) to produce curved tracks. When
 * both fields are zero the existing straight-line fast path is used.
 *
 * This class handles only transport. For standalone hit recording, see AtStandaloneSimulation.
 * For FairRoot pipeline integration, see AtSimpleSimulationTask.
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

public:
   using ModelPtr = std::shared_ptr<AtTools::AtELossModel>;
   using XYZPoint = ROOT::Math::XYZPoint;
   using XYZVector = ROOT::Math::XYZVector;
   using PxPyPzEVector = ROOT::Math::PxPyPzEVector;

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
   using FieldFunc = std::function<std::pair<XYZVector, XYZVector>(const XYZPoint &)>;

   // ---- Construction ----

   AtSimpleSimulation(std::string geoFile);
   AtSimpleSimulation();
   AtSimpleSimulation(std::shared_ptr<AtTools::AtELossModelFactory> factory);
   AtSimpleSimulation(std::string geoFile, std::shared_ptr<AtTools::AtELossModelFactory> factory);
   AtSimpleSimulation(const AtSimpleSimulation &other) = delete; // Implicitly deleted because of std::mutex
   ~AtSimpleSimulation() = default;

   // ---- Model management ----

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

   // ---- Field and step configuration ----

   void SetDistanceStep(double step) { fDistStep = step; } ///< Step size in mm (straight-line path)
   void SetElectricField(XYZVector eField) { fEField = eField; } ///< Electric field in V/m
   void SetMagneticField(XYZVector bField) { fBField = bField; } ///< Magnetic field in T
   /// Maximum step size (m) for the RK4 adaptive stepper in curved-track mode (default: 1e-3 m = 1 mm).
   void SetMaxPropagationStep(double stepM) { fMaxPropStep = stepM; }
   /// Kinetic energy (MeV) below which the particle is considered stopped. Applies to both paths.
   void SetStopTolerance(double stopTolMeV) { fStopTol = stopTolMeV; }
   /// Maximum transport steps before aborting. Applies to both curved and straight-line paths.
   void SetMaxTransportSteps(int maxSteps) { fMaxTransportSteps = maxSteps; }

   /**
    * Set a position-dependent field function. When set, the field is queried at each
    * transport step instead of using the uniform field vectors. The function takes a
    * position in mm and returns (E-field in V/m, B-field in T).
    */
   void SetFieldFunction(FieldFunc func) { fFieldFunc = std::move(func); }

   // ---- Transport API ----

   /**
    * Transport a particle through the loaded geometry without writing detector hits.
    * Steps are delivered via callback; hit recording is the caller's responsibility.
    */
   std::pair<XYZPoint, PxPyPzEVector> TransportParticle(int Z, int A, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
                                                        StepCallback callback);

   // ---- Geometry queries ----

   bool IsInsideGeometry(const XYZPoint &point) { return GetVolume(point) != nullptr; }
   std::string GetVolumeNameAt(const XYZPoint &point) { return GetVolumeName(point); }

protected:
   std::map<ParticleID, ParticleInfo> fModels;
   std::shared_ptr<AtTools::AtELossModelFactory> fModelFactory{nullptr};
   double fDistStep{1.}; // Distance step in mm for straight-line propagation
   std::mutex fGeoMutex;
   TGeoManager *fGeoManager{nullptr};
   TGeoNavigator *fNavigator{nullptr};

   XYZVector fEField{0, 0, 0}; ///< Electric field in V/m (used by AtPropagator)
   XYZVector fBField{0, 0, 0}; ///< Magnetic field in T (used by AtPropagator)
   double fMaxPropStep{1e-3};     ///< Max step size in m for the adaptive stepper (default 1 mm)
   double fStopTol{0.1};          ///< KE stop tolerance in MeV; shared by both transport paths
   int fMaxTransportSteps{200000}; ///< Max steps before aborting; shared by both transport paths
   FieldFunc fFieldFunc{nullptr};  ///< Optional position-dependent field function

   bool IsInVolume(const std::string &volName, const XYZPoint &point);
   std::string GetVolumeName(const XYZPoint &point);

   /**
    * Core transport loop. Propagates a particle through the geometry, invoking callback at each step.
    * Continues while the particle is inside the geometry (GetVolume != nullptr) and KE > threshold.
    * The callback controls early stopping by returning false.
    * Selects curved-track (RK4) or straight-line path based on field settings.
    */
   std::pair<XYZPoint, PxPyPzEVector> PropagateParticle(ParticleInfo info, int pdg, const XYZPoint &iniPos,
                                                        const PxPyPzEVector &iniMom, const StepCallback &callback);

   TGeoVolume *GetVolume(const XYZPoint &pos);

   /// Attempt to auto-create an energy loss model using fModelFactory and the geometry material at pos.
   void TryAutoCreateModel(int Z, int A, const XYZPoint &pos);
};

#endif // AT_SIMPLE_SIMULATION_H
