#ifndef ATHIT_H
#define ATHIT_H

#include "TROOT.h"
#include "TObject.h"
#include "TVector3.h"

class AtHit : public TObject {
public:
   AtHit();
   AtHit(AtHit *hit);
   AtHit(Int_t hitID, TVector3 vec, Double_t charge);
   AtHit(Int_t hitID, Double_t x, Double_t y, Double_t z, Double_t charge);
   AtHit(Int_t PadNum, Int_t hitID, Double_t x, Double_t y, Double_t z, Double_t charge);
   ~AtHit();

   struct MCSimPoint {
      int pointID;
      int trackID;
      double energy;
      double eloss;
      double angle;
      int A;
      int Z;
      MCSimPoint() {}
      MCSimPoint(int pointID_, int trackID_, double energy_, double eloss_, double angle_, int A_, int Z_)
         : pointID(pointID_), trackID(trackID_), energy(energy_), eloss(eloss_), angle(angle_), A(A_), Z(Z_)
      {
      }
   };

   //!< Track ID setter
   void SetTrackID(Int_t trackID);
   //!< Hit ID setter
   void SetHitID(Int_t hitID);
   //!< Hit setter
   void SetHit(Int_t hitID, TVector3 vec, Double_t charge);
   void SetHit(Int_t hitID, Double_t x, Double_t y, Double_t z, Double_t charge);
   void SetHit(Int_t PadNum, Int_t hitID, Double_t x, Double_t y, Double_t z, Double_t charge);
   //!< Position setter
   void SetPosition(TVector3 vec);
   void SetPosition(Double_t x, Double_t y, Double_t z);
   void SetPositionCorr(Double_t x, Double_t y, Double_t z);
   //!< Position sigma setter
   void SetPosSigma(TVector3 vec);
   void SetPosSigma(Double_t dx, Double_t dy, Double_t dz);
   //!< Charge setter
   void SetCharge(Double_t charge);
   //!< Clustered flag setter
   void SetIsClustered(Bool_t value = kTRUE);
   //!< Cluster stter
   void SetClusterID(Int_t clusterID);
   //!< Aux setter
   void SetIsAux(bool value);

   void SetQHit(Double_t Qhit);
   void SetHitMult(Int_t HitMult);
   void SetTimeStamp(Int_t Time);
   void SetTimeStampCorr(Double_t TimeCorr);
   void SetTimeStampCorrInter(Double_t TimeCorrInter);
   void SetBaseCorr(Double_t BaseCorr);
   void SetSlopeCnt(Int_t cnt);

   void SetMCSimPoint(AtHit::MCSimPoint point);

   //!< Track ID getter
   Int_t GetTrackID() const;
   //!< Hit ID getter
   Int_t GetHitID() const;
   //!< Position getter
   TVector3 GetPosition() const;
   //!< Position sigma getter
   TVector3 GetPositionCorr() const;
   TVector3 GetPosSigma() const;
   //!< Charge getter
   Double_t GetCharge() const;
   //!< Clustered flag getter
   Bool_t IsClustered() const;
   //!< Cluster ID getter
   Int_t GetClusterID() const;
   Int_t GetHitPadNum() const;
   Double_t GetQHit() const;
   Int_t GetHitMult() const;
   Int_t GetTimeStamp() const;
   Double_t GetTimeStampCorr() const;
   Double_t GetTimeStampCorrInter() const;
   Double_t GetBaseCorr() const;
   Int_t GetSlopeCnt() const;
   std::vector<AtHit::MCSimPoint> &GetMCSimPointArray();

   bool IsAux() const;

   Int_t fPadNum;
   Int_t fTimeStamp; // Time Stamp of the Hit
   Double_t fTimeStampCorr;
   Double_t fTimeStampCorrInter;
   Double_t fBaseCorr;
   Int_t fSlopeCnt;

protected:
   //!< Track ID having this hit
   Int_t fTrackID;
   //!< Hit ID
   Int_t fHitID;
   //!< Position
   TVector3 fPosition;
   //!< Position error
   TVector3 fPositionSigma;
   TVector3 fPositionCorr; // Position corrected by the Lorentz Angle
   //!< Charge
   Double_t fCharge;
   //!< Integrated pulse charge
   Double_t fQhit;
   //!< Clustered flag
   Bool_t fIsClustered;
   //!< Cluster ID having this hit
   Int_t fClusterID;

   Int_t fHitMult; //!< Hit multiplicity in the pad where the hit was found

   std::vector<AtHit::MCSimPoint> fMCSimPointArray;

   bool kIsAux;

   ClassDef(AtHit, 3);
};

#endif
