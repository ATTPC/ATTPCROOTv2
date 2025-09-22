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
      Double_t kineticEnergy{-1}; // Kinetic energy
      Double_t theta{-1};         // Theta scattering angle
      Double_t phi{-1};           // Phi scattering angle
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
   std::vector<Kinematics> fKinematicsXtr;

   // Particle information.
   std::vector<ParticleInfo> fParticleInfo;

   // Vertex where the particle has originated from.
   std::vector<XYZVector> fVertex;

   // Track properties.
   TrackProperties fTrackProperties;

   // Copy of the AtFitTrackResult object corresponding to the fit used for this track.
   TrackMetadataPtr fTrackMetadata{nullptr};

   // Deprecated members needed to keep support of deprecated methods.
   [[deprecated("No PRA information is supposed to be kept in AtFittedTrack.")]] Double_t fEnergyPRA{0};
   [[deprecated("No PRA information is supposed to be kept in AtFittedTrack.")]] Double_t fThetaPRA{0};
   [[deprecated("No PRA information is supposed to be kept in AtFittedTrack.")]] Double_t fPhiPRA{0};
   [[deprecated("No PRA information is supposed to be kept in AtFittedTrack.")]] XYZVector fInitialPosPRA;
   [[deprecated("No backwards statistics is supposed to be stored. May not be needed in all fitters.")]] Float_t fBChi2{
      0};
   [[deprecated("No backwards statistics is supposed to be stored. May not be needed in all fitters.")]] Float_t fBNdf{
      0};
   [[deprecated("Not all experiments are done with magnetic field.")]] Float_t fBrho{0};
   [[deprecated("No IC information is supposed to be kept in AtFittedTrack.")]] Float_t fIonChamberEnergy{0};
   [[deprecated("No IC information is supposed to be kept in AtFittedTrack.")]] Int_t fIonChamberTime{0};
   [[deprecated("No excitation energy is supposed to be kept in AtFittedTrack.")]] Float_t fExcitationEnergy{0};
   [[deprecated("No excitation energy is supposed to be kept in AtFittedTrack.")]] Float_t fExcitationEnergyXtr{0};

public:
   AtFittedTrack() = default;
   ~AtFittedTrack() = default;

   void SetTrackID(Int_t trackid) { fTrackID = trackid; }

   void SetKinematics(int particleIdx, Double_t energy, Double_t theta, Double_t phi);
   void SetKinematicsXtr(int particleIdx, Double_t energyxtr, Double_t thetaxtr, Double_t phixtr);
   void SetParticleInfo(int particleIdx, std::string pdg, Int_t charge, Double_t mass);
   void SetVertex(int particleIdx, XYZVector point);

   void SetKinematics(Double_t energy, Double_t theta, Double_t phi) { SetKinematics(0, energy, theta, phi); }
   void SetKinematicsXtr(Double_t energyxtr, Double_t thetaxtr, Double_t phixtr)
   {
      SetKinematicsXtr(0, energyxtr, thetaxtr, phixtr);
   }

   void SetParticleInfo(std::string pdg, Int_t charge, Double_t mass) { SetParticleInfo(0, pdg, charge, mass); }

   void SetVertex(XYZVector point) { SetVertex(0, point); }

   void SetTrackPropertiesStruct(XYZVector initialPosition, XYZVector initialPositionXtr, Double_t extrapolatedDistance,
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

   const Kinematics GetKinematics(int particleIdx = 0) { return fKinematics[particleIdx]; }
   const Kinematics GetKinematicsXtr(int particleIdx = 0) { return fKinematicsXtr[particleIdx]; }
   const ParticleInfo GetParticleInfo(int particleIdx = 0) { return fParticleInfo[particleIdx]; }
   const XYZVector GetVertex(int particleIdx = 0) { return fVertex[particleIdx]; }

   const TrackProperties GetTrackPropertiesStruct() { return fTrackProperties; }

   TrackMetadataPtr &GetTrackMetadata() { return fTrackMetadata; }

   // Old deprecated methods.
   [[deprecated("Replaced by SetKinematics() and SetKinematicsXtr().")]] void
   SetEnergyAngles(Float_t energy, Float_t energyxtr, Float_t theta, Float_t phi, Float_t energypra, Float_t thetapra,
                   Float_t phipra)
   {
      fKinematics[0].kineticEnergy = energy;
      fKinematics[0].theta = theta;
      fKinematics[0].phi = phi;
      fKinematicsXtr[0].kineticEnergy = energyxtr;
      fEnergyPRA = energypra;
      fThetaPRA = thetapra;
      fPhiPRA = phipra;
   }
   [[deprecated("This information now lives in the TrackProperties struct. Check the new SetTrackProperties().")]] void
   SetVertexPosition(XYZVector inipos, XYZVector iniposPRA, XYZVector iniposxtr)
   {
      fTrackProperties.initialPosition = inipos;
      fInitialPosPRA = iniposPRA;
      fVertex[0] = iniposxtr;
   }
   [[deprecated("Statistics now live inside the AtFitTrackMetadata. Check SetTrackMetadata().")]] void
   SetStats(Float_t pvalue, Float_t chi2, Float_t bchi2, Float_t ndf, Float_t bndf, Bool_t conv)
   {
      if (fTrackMetadata == nullptr)
         fTrackMetadata = std::make_unique<AtFitTrackMetadata>();
      fTrackMetadata->SetPValue(pvalue);
      fTrackMetadata->SetChi2(chi2);
      fTrackMetadata->SetNdf(ndf);
      fTrackMetadata->SetFitConverged(conv);
      fTrackMetadata->SetTrackID(fTrackID);

      fBChi2 = bchi2;
      fBNdf = bndf;
   }
   [[deprecated("The TrackProperties have changed. Please check the new SetTrackProperties() method.")]] void
   SetTrackProperties(Int_t charge, Float_t brho, Float_t eloss, Float_t dedx, std::string pdg, Int_t points)
   {
      fParticleInfo[0].charge = charge;
      fBrho = brho;
      fTrackProperties.estimateTotalCharge = eloss;
      fTrackProperties.estimateDeDx = dedx;
      fParticleInfo[0].idPDG = pdg;
      fTrackProperties.trackPoints = points;
   }
   [[deprecated("Ion chamber information is no longer saved in the AtFittedTrack. This has been saved here still, but "
                "refrain from using this further.")]] void
   SetIonChamber(Float_t icenergy, Int_t ictime)
   {
      fIonChamberEnergy = icenergy;
      fIonChamberTime = ictime;
   }
   [[deprecated("Excitation energy is no longer saved in the AtFittedTrack. This has been saved here still, but "
                "refrain from using this further.")]] void
   SetExcitationEnergy(Float_t exenergy, Float_t exenergyxtr)
   {
      fExcitationEnergy = exenergy;
      fExcitationEnergyXtr = exenergyxtr;
   }
   [[deprecated("This information now lives in the TrackProperties struct. Check the new SetTrackProperties().")]] void
   SetDistances(Float_t distancextr, Float_t length, Float_t poca)
   {
      fTrackProperties.extrapolatedDistance = distancextr;
      fTrackProperties.trackLength = length;
      fTrackProperties.distancePOCA = poca;
   }

   [[deprecated("Please check GetKinematics() and GetKinematicsXtr().")]] const std::tuple<
      Float_t, Float_t, Float_t, Float_t, Float_t, Float_t, Float_t>
   GetEnergyAngles()
   {
      return std::forward_as_tuple(fKinematics[0].kineticEnergy, fKinematicsXtr[0].kineticEnergy, fKinematics[0].theta,
                                   fKinematics[0].phi, fEnergyPRA, fThetaPRA, fPhiPRA);
   }
   [[deprecated("Please check GetVertex().")]] const std::tuple<XYZVector, XYZVector, XYZVector> GetVertices()
   {
      return std::forward_as_tuple(fTrackProperties.initialPosition, fInitialPosPRA, fVertex[0]);
   }
   [[deprecated("Statistics now live inside the AtFitTrackMetadata. Check GetTrackMetadata().")]] const std::tuple<
      Float_t, Float_t, Float_t, Float_t, Float_t, Bool_t>
   GetStats()
   {
      if (fTrackMetadata == nullptr)
         return std::forward_as_tuple(0, 0, 0, 0, 0, 0);
      return std::forward_as_tuple(fTrackMetadata->GetPValue(), fTrackMetadata->GetChi2(), fBChi2,
                                   fTrackMetadata->GetNdf(), fBNdf, fTrackMetadata->GetFitConverged());
   }
   [[deprecated("The TrackProperties have changed. Please check the new GetTrackProperties() method.")]] const std::
      tuple<Int_t, Float_t, Float_t, Float_t, std::string, Int_t>
      GetTrackProperties()
   {
      return std::forward_as_tuple(fParticleInfo[0].charge, fBrho, fTrackProperties.estimateTotalCharge,
                                   fTrackProperties.estimateDeDx, fParticleInfo[0].idPDG.Data(),
                                   fTrackProperties.trackPoints);
   }
   [[deprecated("Ion chamber information is no longer saved in the AtFittedTrack. This has been saved here still, but "
                "refrain from using this further.")]] const std::tuple<Float_t, Float_t>
   GetIonChamber()
   {
      return std::forward_as_tuple(fIonChamberEnergy, fIonChamberTime);
   }
   [[deprecated("Excitation energy is no longer saved in the AtFittedTrack. This has been saved here still, but "
                "refrain from using this further.")]] const std::tuple<Float_t, Float_t>
   GetExcitationEnergy()
   {
      return std::forward_as_tuple(fExcitationEnergy, fExcitationEnergyXtr);
   }
   [[deprecated(
      "This information now lives in the TrackProperties struct. Check the new SetTrackProperties().")]] const std::
      tuple<Float_t, Float_t, Float_t>
      GetDistances()
   {
      return std::forward_as_tuple(fTrackProperties.extrapolatedDistance, fTrackProperties.trackLength,
                                   fTrackProperties.distancePOCA);
   }

   ClassDef(AtFittedTrack, 2);
};

#endif
