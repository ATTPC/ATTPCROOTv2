/*******************************************************************
 * Daughter class for Circular Hough Space transformation           *
 * Log: Class started 26-10-2015                                    *
 * Author: Y. Ayyad (NSCL ayyadlim@nscl.msu.edu)                    *
 ********************************************************************/

#ifndef AtHOUGHSPACECIRCLE_H
#define AtHOUGHSPACECIRCLE_H

#include "AtHoughSpace.h"
#include "TH2F.h"

// FairRoot classes
#include "FairRootManager.h"
#include "FairLogger.h"

#include "AtDigiPar.h"
#include "AtRansac.h"
#include "AtTrack.h"

class AtHoughSpaceCircle : public AtHoughSpace {
public:
   AtHoughSpaceCircle();
   ~AtHoughSpaceCircle();

   TH2F *GetHoughSpace(TString ProjPlane);
   void CalcHoughSpace(AtEvent *event, Bool_t YZplane, Bool_t XYplane, Bool_t XZplane);
   void CalcHoughSpace(AtEvent *event, TH2Poly *hPadPlane);
   void CalcHoughSpace(AtEvent *event, TH2Poly *hPadPlane, const multiarray &PadCoord); // Circular Hough+RANSAC+MCQ
   void CalcHoughSpace(AtProtoEvent *protoevent, Bool_t q1, Bool_t q2, Bool_t q3, Bool_t q4);
   void CalcMultiHoughSpace(AtEvent *event);
   void CalcHoughSpace(AtEvent *event);

   void SetELossFuncArray(std::vector<std::function<Double_t(Double_t, std::vector<Double_t> &)>> &func_array);

   Double_t GetXCenter() const;
   Double_t GetYCenter() const;
   std::vector<Double_t> const *GetRadiusDist() const;
   std::vector<Int_t> const *GetTimeStamp() const;
   std::vector<Double_t> const *GetPhi() const;
   std::vector<Double_t> const *GetTheta() const;
   std::vector<Double_t> const *GetDl() const;

   Double_t GetIniPhi() const;
   Double_t GetIniTheta() const;
   Double_t GetIniRadius() const;
   Double_t GetIniPhiRansac() const;
   Double_t GetIniThetaRansac() const;
   Double_t GetIniRadiusRansac() const;

   AtHit const *GetIniHit() const;
   AtHit const *GetIniHitRansac() const;

   Double_t const *GetInitialParameters() const;
   std::vector<double> const GetInitialParametersVector() const;

   std::vector<Double_t> GetPosXMin() const;
   std::vector<Double_t> GetPosYMin() const;
   std::vector<Double_t> GetPosZMin() const;
   std::vector<Double_t> GetPosXExp() const;
   std::vector<Double_t> GetPosYExp() const;
   std::vector<Double_t> GetPosZExp() const;
   std::vector<Double_t> GetPosXInt() const;
   std::vector<Double_t> GetPosYInt() const;
   std::vector<Double_t> GetPosZInt() const;
   std::vector<Double_t> GetPosXBack() const;
   std::vector<Double_t> GetPosYBack() const;
   std::vector<Double_t> GetPosZBack() const;

   void FillHoughMap(Double_t ang, Double_t dist);
   std::pair<Double_t, Double_t> GetHoughPar();
   std::vector<Double_t> GetRansacPar();

   // Test function
   static Double_t GetEloss(Double_t c0, std::vector<Double_t> &par);
   void SetElossParameters(const std::vector<Double_t> &par);

   // TODO: void SetThreshold(Double_t value);

   struct FitPar {
      Double_t sThetaMin;
      Double_t sEnerMin;
      TVector3 sPosMin;
      Double_t sBrhoMin;
      Double_t sBMin;
      Double_t sPhiMin;
      Double_t sChi2Min;
      TVector3 sVertexPos;
      Double_t sVertexEner;
      Double_t sMinDistAppr;
      Int_t sNumMCPoint;
      Double_t sNormChi2;
   };

   FitPar FitParameters;
   TH2F *HistHoughXY;

   std::vector<AtHit> *fClusteredHits;
   std::vector<Double_t> fElossPar;

protected:
   void FillHistHoughXY(AtEvent *event, Int_t nstep = 10);
   void CalculateParameters(AtEvent *event);
   void Clustering(AtEvent *event);

   std::pair<Double_t, Double_t>
   CalHoughParameters(TH2F *hist);                     // TODO: implement it as virtual member in the base class
   std::pair<Double_t, Double_t> CalHoughParameters(); // Overloaded version for std::map
   Int_t GetTBMult(Int_t TB, std::vector<AtHit> *harray, Int_t index);
   AtTrack &FindCandidateTrack(const std::vector<AtTrack *> &tracks);
   void GetDeviation(std::vector<AtHit> *hits, Double_t &_x_dev, Double_t &_y_dev);
   void GetTBDeviation(std::vector<AtHit> *hits, Float_t &maxdev_ratio, Double_t &mean_dev_x, Double_t &mean_dev_y);
   Int_t
   GetDensityOfHits(std::vector<AtHit> *hits, Int_t index, Int_t tb_range); // Gets number of hits around a time bucket
   std::vector<AtHit> GetTBHitArray(Int_t TB, std::vector<AtHit> *harray);
   std::vector<AtTrack *> smoothRadius(AtTrack &trackCand);
   double calcAngleFromArcLength(AtTrack *track);

   // TODO: Double_t fThreshold;
   std::map<std::vector<Float_t>, Int_t> HoughMap_XY;
   std::vector<Float_t> HoughPar;
   std::vector<Double_t> *fRadius;
   std::vector<Int_t> *fTimeStamp;
   std::vector<Double_t> *fPhi;
   std::vector<Double_t> *fTheta;
   std::vector<Double_t> *fDl;

   TH2F *HistHoughAux;
   Double_t fXCenter;
   Double_t fYCenter;

   AtHit *fIniHit;
   AtHit *fIniHitRansac;
   Double_t fIniRadius;
   Double_t fIniRadiusRansac;
   Double_t fIniPhi;
   Double_t fIniPhiRansac;
   Double_t fIniTheta;
   Double_t fIniThetaRansac;
   Int_t fIniTS;
   Int_t fIniHitID;
   Double_t fDriftVelocity;
   Int_t fTBTime;

   Bool_t kDebug;
   Bool_t kHough;

   std::vector<Double_t> fPosXmin;
   std::vector<Double_t> fPosYmin;
   std::vector<Double_t> fPosZmin;
   std::vector<Double_t> fPosXexp;
   std::vector<Double_t> fPosYexp;
   std::vector<Double_t> fPosZexp;
   std::vector<Double_t> fPosXinter;
   std::vector<Double_t> fPosYinter;
   std::vector<Double_t> fPosZinter;
   std::vector<Double_t> fPosXBack;
   std::vector<Double_t> fPosYBack;
   std::vector<Double_t> fPosZBack;

   std::map<ULong64_t, Int_t> HoughMap; // 8 byte for the key, unsigned, no negative distance in Linear Hough space is
                                        // expected for each quadrant (Radius and Z are the variables)
   std::vector<ULong64_t> HoughMapKey;
   std::pair<Double_t, Double_t> fHoughLinePar;
   std::vector<Double_t> fRansacLinePar;

   Double_t fStdDeviationLimit;

   Double_t fParameter[8];
   std::vector<std::function<Double_t(Double_t, std::vector<Double_t> &)>> fEloss_func_array; // !

   std::vector<double> fParAux;

   AtRANSACN::AtRansac RansacSmoothRadius;

   struct maxpersecond {
      template <typename Lhs, typename Rhs>
      bool operator()(const Lhs &lhs, const Rhs &rhs) const
      {
         return lhs.second < rhs.second;
      }
   };

   ClassDef(AtHoughSpaceCircle, 3);
};

#endif
