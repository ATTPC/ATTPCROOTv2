#ifndef ATFITTEDTRACK_H
#define ATFITTEDTRACK_H

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h>
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h>
#include <Rtypes.h>
#include <TMath.h>
#include <TObject.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

class TBuffer;
class TClass;
class TMemberInspector;

class AtFittedTrack : public TObject {

   using XYZVector = ROOT::Math::XYZVector;

private:
   Int_t fTrackID{-1}; //< Track ID from pattern recognition

   Float_t fEnergy{0};
   Float_t fTheta{0};
   Float_t fPhi{0};
   Float_t fEnergyPRA{0};
   Float_t fThetaPRA{0};
   Float_t fPhiPRA{0};

   Float_t fExcitationEnergy{0};

   XYZVector fInitialPos;    // xiniFitVec,yiniFitVec,ziniFitVec;
   XYZVector fInitialPosPRA; // xiniPRAVec,yiniPRAVec,ziniPRAVec;
   XYZVector fInitialPosXtr;

   Float_t fIonChamberEnergy{0};
   Int_t fIonChamberTime{0};

   Float_t fEnergyXtr{0};
   Float_t fExcitationEnergyXtr{0};

   Float_t fDistanceXtr{0};
   Float_t fTrackLength{0};
   Float_t fPOCAXtr{0};

   Float_t fPValue{0};
   Float_t fChi2{0};
   Float_t fBChi2{0};
   Float_t fNdf{0};
   Float_t fBNdf{0};
   Bool_t fFitConverged{0};

   Int_t fCharge{0};
   Float_t fBrho{0};
   Float_t fELossADC{0};
   Float_t fDEdxADC{0};
   std::string fPDG{0};
   Int_t fTrackPoints{0};

public:
   AtFittedTrack();
   ~AtFittedTrack() = default;

   inline void SetTrackID(Int_t trackid) { fTrackID = trackid; }

   inline void SetEnergyAngles(Float_t energy, Float_t energyxtr, Float_t theta, Float_t phi, Float_t energypra,
                               Float_t thetapra, Float_t phipra)
   {
      fEnergy = energy;
      fEnergyXtr = energyxtr;
      fTheta = theta;
      fPhi = phi;
      fEnergyPRA = energypra;
      fThetaPRA = thetapra;
      fPhiPRA = phi;
   }

   inline void SetVertexPosition(XYZVector inipos, XYZVector iniposPRA, XYZVector iniposxtr)
   {
      fInitialPos = inipos;
      fInitialPosPRA = iniposPRA;
      fInitialPosXtr = iniposxtr;
   }

   inline void SetStats(Float_t pvalue, Float_t chi2, Float_t bchi2, Float_t ndf, Float_t bndf, Bool_t conv)
   {
      fPValue = pvalue;
      fChi2 = chi2;
      fBChi2 = bchi2;
      fNdf = ndf;
      fBNdf = bndf;
      fFitConverged = conv;
   }

   inline void
   SetTrackProperties(Int_t charge, Float_t brho, Float_t eloss, Float_t dedx, std::string pdg, Int_t points)
   {
      fCharge = charge;
      fBrho = brho;
      fELossADC = eloss;
      fDEdxADC = dedx;
      fPDG = pdg;
      fTrackPoints = points;
   }

   inline void SetIonChamber(Float_t icenergy, Int_t ictime)
   {
      fIonChamberEnergy = icenergy;
      fIonChamberTime = ictime;
   }

   inline void SetExcitationEnergy(Float_t exenergy, Float_t exenergyxtr)
   {
      fExcitationEnergy = exenergy;
      fExcitationEnergyXtr = exenergyxtr;
   }

   inline void SetDistances(Float_t distancextr, Float_t length, Float_t poca)
   {
      fDistanceXtr = distancextr;
      fTrackLength = length;
      fPOCAXtr = poca;
   }

   const Int_t GetTrackID() { return fTrackID; }

   const std::tuple<Float_t, Float_t, Float_t, Float_t, Float_t, Float_t, Float_t> GetEnergyAngles();
   const std::tuple<XYZVector, XYZVector, XYZVector> GetVertices();
   const std::tuple<Float_t, Float_t, Float_t, Float_t, Float_t, Bool_t> GetStats();
   const std::tuple<Int_t, Float_t, Float_t, Float_t, std::string, Int_t> GetTrackProperties();
   const std::tuple<Float_t, Float_t> GetIonChamber();
   const std::tuple<Float_t, Float_t> GetExcitationEnergy();
   const std::tuple<Float_t, Float_t, Float_t> GetDistances();

   ClassDef(AtFittedTrack, 1);
};

#endif