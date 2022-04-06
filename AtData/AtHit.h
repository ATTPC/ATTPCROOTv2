#ifndef ATHIT_H
#define ATHIT_H

#include <Math/Point3Dfwd.h>
#include <Math/Vector3Dfwd.h>
#include <Rtypes.h>
#include <RtypesCore.h>
#include <Math/Point3D.h>
#include <Math/Vector3D.h>
#include <algorithm>
#include <vector>

#include <TObject.h>

class TBuffer;
class TClass;
class TMemberInspector;

using XYZPoint = ROOT::Math::XYZPoint;
using XYZVector = ROOT::Math::XYZVector;

class AtHit : public TObject {
public:
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

protected:
   Double_t fCharge;
   Int_t fHitID;
   Int_t fPadNum;
   XYZPoint fPosition;

   Int_t fTrackID = -1;
   Double_t fTraceIntegral = -1; //!< Integrated pulse charge of full trace
   Int_t fHitMult = 1;           //!< Hit multiplicity in the pad where the hit was found
   Bool_t fIsAux = false;
   Int_t fTimeStamp = 0;
   Double_t fTimeStampCorr = 0;
   Double_t fTimeStampCorrInter = 0;
   Double_t fBaseCorr = 0;
   Int_t fSlopeCnt = 0;

   XYZPoint fPositionSigma; //!< Position error
   XYZPoint fPositionCorr;  // Position corrected by the Lorentz Angle
   std::vector<AtHit::MCSimPoint> fMCSimPointArray;

public:
   AtHit(Int_t hitID = -1);
   AtHit(Int_t hitID, const XYZPoint &location, Double_t charge);
   AtHit(Int_t hitID, Int_t padNum, const XYZPoint &location, Double_t charge);
   AtHit(Int_t hitID, Double_t x, Double_t y, Double_t z, Double_t charge);
   AtHit(Int_t hitID, Int_t padNum, Double_t x, Double_t y, Double_t z, Double_t charge);
   AtHit(const AtHit &hit) = default;
   virtual ~AtHit() = default;

   void SetTrackID(Int_t trackID) { fTrackID = trackID; }
   void SetHitID(Int_t hitID) { fHitID = hitID; }
   void SetPadNum(Int_t padNum) { fPadNum = padNum; }
   void SetPosition(const XYZPoint &pos) { fPosition = pos; }
   void SetPosition(Double_t x, Double_t y, Double_t z) { fPosition = XYZPoint(x, y, z); }
   void SetPositionCorr(const XYZPoint &pos) { fPositionCorr = pos; }
   void SetPositionCorr(Double_t x, Double_t y, Double_t z) { fPositionCorr = XYZPoint(x, y, z); }
   void SetPositionSigma(const XYZPoint &vec) { fPositionSigma = vec; }
   void SetPositionSigma(Double_t dx, Double_t dy, Double_t dz) { fPositionSigma = XYZVector(dx, dy, dz); }
   void SetCharge(Double_t charge) { fCharge = charge; }
   void SetIsAux(Bool_t value = true) { fIsAux = value; }
   void SetTraceIntegral(Double_t integral) { fTraceIntegral = integral; }
   void SetHitMult(Int_t HitMult) { fHitMult = HitMult; }
   void SetTimeStamp(Int_t Time) { fTimeStamp = Time; }
   void SetTimeStampCorr(Double_t TimeCorr) { fTimeStampCorr = TimeCorr; }
   void SetTimeStampCorrInter(Double_t TimeCorrInter) { fTimeStampCorrInter = TimeCorrInter; }
   void SetBaseCorr(Double_t BaseCorr) { fBaseCorr = BaseCorr; }
   void SetSlopeCnt(Int_t cnt) { fSlopeCnt = cnt; }
   void AddMCSimPoint(const AtHit::MCSimPoint &point) { fMCSimPointArray.push_back(point); }

   Int_t GetTrackID() const { return fTrackID; }
   Int_t GetHitID() const { return fHitID; }
   const XYZPoint &GetPosition() const { return fPosition; }
   const XYZPoint &GetPositionCorr() const { return fPositionCorr; }
   const XYZPoint &GetPositionSigma() const { return fPositionSigma; }
   Double_t GetCharge() const { return fCharge; }
   Int_t GetPadNum() const { return fPadNum; }
   Double_t GetTraceIntegral() const { return fTraceIntegral; }
   Int_t GetHitMult() const { return fHitMult; }
   Int_t GetTimeStamp() const { return fTimeStamp; }
   Double_t GetTimeStampCorr() const { return fTimeStampCorr; }
   Double_t GetTimeStampCorrInter() const { return fTimeStampCorrInter; }
   Double_t GetBaseCorr() const { return fBaseCorr; }
   Int_t GetSlopeCnt() const { return fSlopeCnt; }
   std::vector<AtHit::MCSimPoint> &GetMCSimPointArray() { return fMCSimPointArray; }
   Bool_t IsAux() const { return fIsAux; }

   static Bool_t SortHit(const AtHit &lhs, const AtHit &rhs) { return lhs.GetPadNum() < rhs.GetPadNum(); }
   static Bool_t SortHitTime(const AtHit &lhs, const AtHit &rhs) { return lhs.GetTimeStamp() < rhs.GetTimeStamp(); }

   ClassDef(AtHit, 4);
};

#endif
