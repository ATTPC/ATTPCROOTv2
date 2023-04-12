#ifndef ATHIT_H
#define ATHIT_H

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h>
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h>
#include <Rtypes.h>
#include <TObject.h>

#include <algorithm>
#include <memory>
#include <vector>
class TBuffer;
class TClass;
class TMemberInspector;
namespace H5 {
class CompType;
}

/**
 * @brief Point in space with charge.
 *
 * A hit describes a point in the detector where charge was deposited, It is the output
 * of a PSA method.
 */
class AtHit : public TObject {
public:
   using XYZPoint = ROOT::Math::XYZPoint;
   using XYZVector = ROOT::Math::XYZVector;
   struct MCSimPoint;

protected:
   Double_t fCharge;              //< Charge of hit
   Double_t fChargeVariance{0};   //< Charge variance
   XYZPoint fPosition;            //< Position of hit
   XYZVector fPositionVariance{}; //< Position variance (unused by AtHitCluster)
   Int_t fHitID;                  //< Unique ID of hit
   Int_t fPadNum;                 //< Pad that generated hit

   Double_t fTraceIntegral{-1};     //< Integrated pulse charge of full trace
   Int_t fHitMult{1};               //< Hit multiplicity in the pad where the hit was found
   Int_t fTimeStamp{0};             //< TB of hit
   Double_t fTimeStampCorr{0};      //< TB of hit using center of gravity
   Double_t fTimeStampCorrInter{0}; //< Interpolated TB of hit

   std::vector<AtHit::MCSimPoint> fMCSimPointArray;

public:
   AtHit(Int_t hitID = -1);                                              //< Default constructor for IO
   AtHit(Int_t padNum, XYZPoint location, Double_t charge);              //< Primary constructor
   AtHit(Int_t hitID, Int_t padNum, XYZPoint location, Double_t charge); //< Specify hit ID on creation
   AtHit(const AtHit &) = default;                                       //< Copy constructor
   AtHit(AtHit &&) = default;                                            //< Move constructor
   AtHit &operator=(const AtHit &) = default;                            //< Copy assignment
   AtHit &operator=(AtHit &&) = default;                                 //< Move assignment
   virtual ~AtHit() = default;
   virtual std::unique_ptr<AtHit> Clone(); //< Create a copy of sub-type

   /// Returns the type specification of a hit
   H5::CompType GetHDF5Type();

   void SetCharge(Double_t charge) { fCharge = charge; }
   void SetChargeVariance(Double_t chargeVar) { fChargeVariance = chargeVar; }
   void SetPosition(const XYZPoint &pos) { fPosition = pos; }
   virtual void SetPositionVariance(const XYZVector &vec) { fPositionVariance = vec; }
   void SetHitID(Int_t hitID) { fHitID = hitID; }
   void SetPadNum(Int_t padNum) { fPadNum = padNum; }

   void SetTraceIntegral(Double_t integral) { fTraceIntegral = integral; }
   void SetHitMult(Int_t HitMult) { fHitMult = HitMult; }
   void SetTimeStamp(Int_t Time) { fTimeStamp = Time; }
   void SetTimeStampCorr(Double_t TimeCorr) { fTimeStampCorr = TimeCorr; }
   void SetTimeStampCorrInter(Double_t TimeCorrInter) { fTimeStampCorrInter = TimeCorrInter; }

   void AddMCSimPoint(const AtHit::MCSimPoint &point) { fMCSimPointArray.push_back(point); }

   Int_t GetHitID() const { return fHitID; }
   const XYZPoint &GetPosition() const { return fPosition; }
   const XYZVector &GetPositionVariance() const { return fPositionVariance; }
   XYZVector GetPositionSigma() const;
   Double_t GetCharge() const { return fCharge; }
   Int_t GetPadNum() const { return fPadNum; }
   Double_t GetTraceIntegral() const { return fTraceIntegral; }
   Int_t GetHitMult() const { return fHitMult; }
   Int_t GetTimeStamp() const { return fTimeStamp; }
   Double_t GetTimeStampCorr() const { return fTimeStampCorr; }
   Double_t GetTimeStampCorrInter() const { return fTimeStampCorrInter; }
   const std::vector<AtHit::MCSimPoint> &GetMCSimPointArray() const { return fMCSimPointArray; }

   static Bool_t SortHit(const AtHit &lhs, const AtHit &rhs) { return lhs.GetPadNum() < rhs.GetPadNum(); }
   static Bool_t SortHit(const std::unique_ptr<AtHit> &lhs, const std::unique_ptr<AtHit> &rhs)
   {
      return SortHit(*lhs, *rhs);
   }

   static Bool_t SortHitTimePtr(const std::unique_ptr<AtHit> &lhs, const std::unique_ptr<AtHit> &rhs)
   {
      return SortHitTime(*lhs, *rhs);
   }

   static Bool_t SortHitTime(const AtHit &lhs, const AtHit &rhs) { return lhs.GetTimeStamp() < rhs.GetTimeStamp(); }

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

   ClassDef(AtHit, 5);
};

/**
 * Data structure representing a hit in an HDF5 file
 */
struct AtHit_t {
   double x;
   double y;
   double z;
   int t;
   double A;
   int trackID;
   int pointIDMC;
   int trackIDMC;
   double energyMC;
   double elossMC;
   double angleMC;
   int AMC;
   int ZMC;
};

#endif
