#ifndef AT_SIM_TRANSPORT_H
#define AT_SIM_TRANSPORT_H

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h>
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h>
#include <Math/Vector4D.h>
#include <Math/Vector4Dfwd.h>

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace AtTools {
class AtELossModel;
class AtELossManager;
} // namespace AtTools

class TGeoVolume;
class TGeoManager;
class TGeoNavigator;
class TGeoMaterial;

/**
 * Transport engine for propagating particles through a ROOT geometry using energy-loss
 * models supplied by an AtELossManager.
 *
 * Internal units are mm (distance), MeV (energy), MeV/c (momentum).
 *
 * When a B field is set (non-zero) or a field function is provided, the simulation uses
 * AtPropagator (Lorentz force + RK4 adaptive stepping) to produce curved tracks. When
 * both fields are zero the existing straight-line fast path is used.
 *
 * This class handles only transport. For standalone hit recording see AtSimpleSimulation;
 * for FairRoot pipeline integration see AtSimTransportTask.
 */
class AtSimTransport {
public:
   using ModelPtr = std::shared_ptr<AtTools::AtELossModel>;
   using XYZPoint = ROOT::Math::XYZPoint;
   using XYZVector = ROOT::Math::XYZVector;
   using PxPyPzEVector = ROOT::Math::PxPyPzEVector;

   /**
    * Per-step snapshot delivered to the transport callback.
    *
    * Units: mm (position/length), MeV (energy), MeV/c (momentum). Pre/post pair semantics
    * (not entering/exiting flags) — detector-level flag state lives in AtTpc::StepState
    * and is assembled by the adapter in AtSimTransportTask::SubmitDetectorStep.
    *
    * Track identity is owned by the caller (attach it via the callback closure); the
    * engine does not carry a trackID. Material name lets callbacks observe medium
    * transitions without querying geometry themselves.
    */
   struct TransportStep {
      int pdg = 0;
      std::string preVolumeName;
      std::string postVolumeName;
      std::string materialName;
      double energyLoss = 0.0; ///< MeV lost in this step
      double length = 0.0;     ///< mm accumulated since transport start
      double trackMass = 0.0;  ///< MeV/c² (projectile mass used by the energy-loss model)
      XYZPoint prePosition;
      XYZPoint postPosition;
      PxPyPzEVector preMomentum;
      PxPyPzEVector postMomentum;
   };

   /**
    * Callback invoked by the transport loop once per step. Return true to continue, false
    * to stop the particle immediately. Invoked after the step's energy loss has been
    * applied; post* fields reflect the state at the end of the step.
    */
   using StepCallback = std::function<bool(const TransportStep &)>;

   /// Position-dependent field function. Position in mm; returns (E-field in V/m, B-field in T).
   using FieldFunc = std::function<std::pair<XYZVector, XYZVector>(const XYZPoint &)>;

   // ---- Construction ----

   /// Construct with a default (accept-only) AtELossManager. Callers who want auto-generation
   /// should pass an AtELossManagerBetheBloch / CATIMA subclass explicitly.
   AtSimTransport();
   explicit AtSimTransport(std::shared_ptr<AtTools::AtELossManager> manager);
   explicit AtSimTransport(std::string geoFile);
   AtSimTransport(std::string geoFile, std::shared_ptr<AtTools::AtELossManager> manager);
   AtSimTransport(const AtSimTransport &other) = delete; // std::mutex
   ~AtSimTransport() = default;

   // ---- Manager access ----

   AtTools::AtELossManager *GetManager() { return fManager.get(); }
   void SetManager(std::shared_ptr<AtTools::AtELossManager> manager) { fManager = std::move(manager); }

   /// Register a pre-built model (material-agnostic); delegates to the manager.
   /// Preserved for drop-in compatibility with existing MCFission / AtMCFitter calls.
   void AddModel(int Z, int A, ModelPtr model);

   /// Register a pre-built model for a specific material name.
   void AddModel(int Z, int A, const std::string &materialName, ModelPtr model);

   // ---- Field and step configuration ----

   /// Maximum distance per transport step, in mm. Applies to both curved and straight-line
   /// paths; only one is active at a time (selected by field settings).
   void SetMaxStep(double stepMm) { fMaxStep = stepMm; }
   /// Legacy alias for SetMaxStep; kept so pre-manager macros keep compiling. Prefer SetMaxStep.
   void SetDistanceStep(double stepMm) { SetMaxStep(stepMm); }
   void SetElectricField(XYZVector eField) { fEField = eField; } ///< V/m
   void SetMagneticField(XYZVector bField) { fBField = bField; } ///< T
   void SetStopTolerance(double stopTolMeV) { fStopTol = stopTolMeV; }
   void SetMaxTransportSteps(int maxSteps) { fMaxTransportSteps = maxSteps; }
   /// Maximum consecutive minimum-size RK4 steps before the curved path gives up.
   void SetMaxMinStepStreak(int steps) { fMaxMinStepStreak = steps; }
   /// Scale factor above stepper's fMinStep used to classify a step as "minimum-size."
   void SetMinStepGuardScale(double scale) { fMinStepGuardScale = scale; }
   void SetFieldFunction(FieldFunc func) { fFieldFunc = std::move(func); }

   // ---- Transport API ----

   /**
    * Transport a particle through the loaded geometry. Hit recording is the caller's
    * responsibility (attach hooks via the callback).
    *
    * Throws std::invalid_argument if no model can be obtained from the manager for this
    * particle + start material, or if the start position is outside the geometry.
    */
   std::pair<XYZPoint, PxPyPzEVector>
   TransportParticle(int Z, int A, const XYZPoint &iniPos, const PxPyPzEVector &iniMom, StepCallback callback);

   // ---- Geometry queries ----

   bool IsInsideGeometry(const XYZPoint &point) { return GetVolume(point) != nullptr; }
   std::string GetVolumeNameAt(const XYZPoint &point) { return GetVolumeName(point); }

private:
   std::shared_ptr<AtTools::AtELossManager> fManager;
   double fMaxStep{1.}; ///< mm (straight-line step distance; also used as max step for adaptive RK4)
   std::mutex fGeoMutex;
   TGeoManager *fGeoManager{nullptr};
   TGeoNavigator *fNavigator{nullptr};

   XYZVector fEField{0, 0, 0}; ///< V/m
   XYZVector fBField{0, 0, 0}; ///< T
   double fStopTol{1e-3};      ///< MeV — KE threshold below which the particle is considered stopped
   int fMaxTransportSteps{200000};
   int fMaxMinStepStreak{4096}; ///< Consecutive min-size RK4 steps before aborting the curved path.
   double fMinStepGuardScale{1.01};
   FieldFunc fFieldFunc{nullptr};

   struct ParticleInfo {
      ModelPtr model;
      double charge{0.}; ///< Coulombs
      double mass{0.};   ///< MeV/c²
   };

   /// Look up (or build) model + particle-kinematic info for (Z, A) in the current material.
   /// Returns info with model == nullptr if no model is available.
   ParticleInfo LookupParticleInfo(int Z, int A, const XYZPoint &pos);

   std::string GetVolumeName(const XYZPoint &point);
   TGeoVolume *GetVolume(const XYZPoint &pos);
   TGeoMaterial *GetMaterial(const XYZPoint &pos);

   /// Core transport loop, branches on field settings (curved RK4 vs straight-line).
   std::pair<XYZPoint, PxPyPzEVector> PropagateParticle(int Z, int A, int pdg, const XYZPoint &iniPos,
                                                        const PxPyPzEVector &iniMom, const StepCallback &callback);

   std::pair<XYZPoint, PxPyPzEVector> PropagateCurved(int Z, int A, int pdg, const XYZPoint &iniPos,
                                                      const PxPyPzEVector &iniMom, const StepCallback &callback);

   std::pair<XYZPoint, PxPyPzEVector> PropagateStraightLine(int Z, int A, int pdg, const XYZPoint &iniPos,
                                                            const PxPyPzEVector &iniMom, const StepCallback &callback);

   static TransportStep BuildStep(int pdg, std::string preVolumeName, std::string postVolumeName,
                                  std::string materialName, const XYZPoint &posBefore, const XYZPoint &posAfter,
                                  const PxPyPzEVector &momBefore, const PxPyPzEVector &momAfter, double eLoss,
                                  double length, double mass);
};

#endif // AT_SIM_TRANSPORT_H
