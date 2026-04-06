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
} // namespace AtTools
class TGeoVolume;
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
   SpaceChargeModel fSCModel{nullptr};
   double fDistStep{1.}; // Distance step in mm for straight-line propagation
   std::mutex fGeoMutex;

   XYZVector fEField{0, 0, 0}; ///< Electric field in V/m (used by AtPropagator)
   XYZVector fBField{0, 0, 0}; ///< Magnetic field in T (used by AtPropagator)
   double fMaxPropStep{1e-3};  ///< Max step size in m for the adaptive stepper (default 1 mm)

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

   /**
    * Assumes that the IO manager has been initialized (it will attempt to construct the branch needed here).
    */
   AtSimpleSimulation(std::string geoFile);
   AtSimpleSimulation();
   AtSimpleSimulation(const AtSimpleSimulation &other) = delete; // Implicitly deleted because of std::mutex
   ~AtSimpleSimulation() = default;

   void RegisterBranch(std::string branchName = "AtTpcPoint", bool pers = true);

   /**
    * Register an energy loss model for a particle species. Charge is derived as Z*e and
    * mass as A * 931.494 MeV/c². Use the overload with massAmu for higher accuracy.
    */
   void AddModel(int Z, int A, ModelPtr model);

   /**
    * Register an energy loss model with an explicit nuclear mass (in amu).
    */
   void AddModel(int Z, int A, ModelPtr model, double massAmu);

   void SetSpaceChargeModel(SpaceChargeModel model) { fSCModel = model; }
   void SetDistanceStep(double step) { fDistStep = step; } ///< Step size in mm (straight-line path)

   void SetElectricField(XYZVector eField) { fEField = eField; } ///< Electric field in V/m
   void SetMagneticField(XYZVector bField) { fBField = bField; } ///< Magnetic field in T
   /// Maximum step size (m) for the RK4 adaptive stepper in curved-track mode (default: 1e-3 m = 1 mm).
   void SetMaxPropagationStep(double stepM) { fMaxPropStep = stepM; }

   void NewEvent();

   /**
    * Simulates a particle over a given distance and returns the position and momentum of the particle at the stopping
    * point. Uses Z and A to provide a model to the protected version of SimulateParticle.
    * When E/B fields are non-zero, AtPropagator is used for curved-track propagation.
    */
   std::pair<XYZPoint, PxPyPzEVector> SimulateParticle(
      int Z, int A, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
      std::function<bool(XYZPoint, PxPyPzEVector)> func = [](XYZPoint pos, PxPyPzEVector mom) { return true; });

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
    * Core simulation loop. Selects straight-line or curved-track path based on field settings.
    */
   std::pair<XYZPoint, PxPyPzEVector> SimulateParticle(
      const ParticleInfo &info, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
      std::function<bool(XYZPoint, PxPyPzEVector)> func = [](XYZPoint pos, PxPyPzEVector mom) { return true; });

   std::pair<XYZPoint, PxPyPzEVector> TransportParticle(const ParticleInfo &info, int pdg, const XYZPoint &iniPos,
                                                        const PxPyPzEVector &iniMom, const StepCallback &callback);

   void AddHit(double ELoss, const XYZPoint &pos, const PxPyPzEVector &mom, double length);
   TGeoVolume *GetVolume(const XYZPoint &pos);
};

#endif // AT_SIMPLE_SIMULATION_H
