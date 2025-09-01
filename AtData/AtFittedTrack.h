#ifndef ATFITTEDTRACK_H
#define ATFITTEDTRACK_H

#include "AtFitTrackMetadata.h"

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h>
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h>
#include <Rtypes.h>
#include <TMath.h>
#include <TObject.h>
#include <TString.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

class TBuffer;
class TClass;
class TMemberInspector;

class AtFittedTrack : public TObject {
public:
   using XYZVector = ROOT::Math::XYZVector;
   using TrackMetadataPtr = std::unique_ptr<AtFitTrackMetadata>;

   struct Kinematics {
      Double_t kineticEnergy{-1};    // Kinetic energy
      Double_t theta{-1};            // Theta scattering angle
      Double_t phi{-1};              // Phi scattering angle
      Double_t kineticEnergyXtr{-1}; // Extrapolated kinetic energy
      Double_t thetaXtr{-1};         // Extrapolated theta scattering angle
      Double_t phiXtr{-1};           // Extrapolated phi scattering angle
   };

   struct ParticleInfo {
      TString idPDG{""}; // PDG code of the particle
      Int_t charge{0};   // Charge number of the particle
      Double_t mass{-1}; // Mass of the particle in amu
   };

   struct TrackProperties {
      XYZVector initialPosition;         // Position of the first hit
      XYZVector initialPositionXtr;      // Position of the point closest to (0,0)
      Double_t extrapolatedDistance{-1}; // Distance initialPosition->initialPositionXtr along the pattern
      Double_t distancePOCA{-1};         // Distance initialPositionXtr->(0,0)
      Double_t trackLength{-1};          // Distance initialPosition->End of charge
      Double_t trackLengthXtr{-1};       // Distance initialPositionXtr->End of charge
      Double_t estimateTotalCharge{-1};  // Sum of the charge of all hits
      Double_t estimateDeDx{-1};         // Sum of the charge of all hits divided by range
      Int_t trackPoints{-1};             // Number of hits in the track
   };

private:
   Int_t fTrackID{-1}; //< Track ID from pattern recognition

   // Kinematic variables obtained by the fit.
   std::vector<Kinematics> fKinematics;

   // Particle information.
   std::vector<ParticleInfo> fParticleInfo;

   // Vertex where the particle has originated from.
   std::vector<XYZVector> fVertex;

   // Track properties.
   TrackProperties fTrackProperties;

   // Copy of the AtFitTrackResult object corresponding to the fit used for this track.
   TrackMetadataPtr fTrackMetadata{nullptr};

public:
   AtFittedTrack() = default;
   ~AtFittedTrack() = default;

   void SetTrackID(Int_t trackid) { fTrackID = trackid; }

   void SetKinematics(int particleIdx, Double_t energy, Double_t theta, Double_t phi, Double_t energyxtr,
                      Double_t thetaxtr, Double_t phixtr);
   void SetParticleInfo(int particleIdx, std::string pdg, Int_t charge, Double_t mass);
   void SetVertex(int particleIdx, XYZVector point);

   void
   SetKinematics(Double_t energy, Double_t theta, Double_t phi, Double_t energyxtr, Double_t thetaxtr, Double_t phixtr)
   {
      SetKinematics(0, energy, theta, phi, energyxtr, thetaxtr, phixtr);
   }

   void SetParticleInfo(std::string pdg, Int_t charge, Double_t mass) { SetParticleInfo(0, pdg, charge, mass); }

   void SetVertex(XYZVector point) { SetVertex(0, point); }

   void SetTrackProperties(XYZVector initialPosition, XYZVector initialPositionXtr, Double_t extrapolatedDistance,
                           Double_t distancePOCA, Double_t trackLength, Double_t trackLengthXtr,
                           Double_t estimateTotalCharge, Int_t trackPoints)
   {
      fTrackProperties.initialPosition = initialPosition;
      fTrackProperties.initialPositionXtr = initialPositionXtr;
      fTrackProperties.extrapolatedDistance = extrapolatedDistance;
      fTrackProperties.distancePOCA = distancePOCA;
      fTrackProperties.trackLength = trackLength;
      fTrackProperties.trackLengthXtr = trackLengthXtr;
      fTrackProperties.estimateTotalCharge = estimateTotalCharge;
      fTrackProperties.estimateDeDx = estimateTotalCharge / trackLengthXtr;
      fTrackProperties.trackPoints = trackPoints;
   }

   void SetTrackMetadata(TrackMetadataPtr trackMetadata) { fTrackMetadata = std::move(trackMetadata); }

   const Int_t GetTrackID() { return fTrackID; }

   const Kinematics GetKinematics(int particleIdx) { return fKinematics[particleIdx]; }
   const ParticleInfo GetParticleInfo(int particleIdx) { return fParticleInfo[particleIdx]; }
   const XYZVector GetVertex(int particleIdx) { return fVertex[particleIdx]; }

   const Kinematics GetKinematics() { return fKinematics[0]; }
   const ParticleInfo GetParticleInfo() { return fParticleInfo[0]; }
   const XYZVector GetVertex() { return fVertex[0]; }

   const TrackProperties GetTrackProperties() { return fTrackProperties; }

   TrackMetadataPtr &GetTrackMetadata() { return fTrackMetadata; }

   ClassDef(AtFittedTrack, 2);
};

#endif
