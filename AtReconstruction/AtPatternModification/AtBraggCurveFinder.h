#ifndef AtBRAGGCURVEFINDER_H
#define AtBRAGGCURVEFINDER_H

#include "AtHit.h"
#include "AtPSAHitPerTB.h"
#include "AtPatternModification.h"
#include "AtTrack.h"

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <TMath.h>

#include <cmath>
#include <random>

class AtBraggCurveFinder : public AtPatternModification {
public:
   using XYZPoint = ROOT::Math::XYZPoint;

protected:
   // Line distance threshold to be used by the AtFindVertex.
   Double_t fLineDistThreshold{30};

   // Pointer to AtPSAHitPerTB and parameters related to it.
   std::unique_ptr<AtPSAHitPerTB> fPSA{nullptr};
   int fTSSemiWidth{5};
   bool fNeedPSA{true};

   // Parameters related to the histograming.
   double fBinSize{1}; // 1mm by default
   double fMaxLength{TMath::Sqrt(std::pow(1000., 2) + std::pow(250., 2))};
   double fELossRelativeError{0.1};
   int fNumSmoothingSteps{1};

   // Random number generator to smooth the Bragg curve.
   std::uniform_real_distribution<double> fUniform{-0.5, 0.5};
   std::mt19937_64 fRNG;

public:
   AtBraggCurveFinder() = default;
   ~AtBraggCurveFinder() = default;

   virtual void Init() override;

   virtual void
   ModifyPatternEvent(AtPatternEvent *patternEvent, AtRawEvent *rawEvent = nullptr, AtEvent *event = nullptr) override;

   void SetLineDistThreshold(Double_t lineDistThreshold) { fLineDistThreshold = lineDistThreshold; }
   void SetTSSemiWidth(int value) { fTSSemiWidth = value; }
   void SetNeedPSA(bool value) { fNeedPSA = value; }

   void SetBinSize(double value) { fBinSize = value; }
   void SetMaxLength(double value) { fMaxLength = value; }
   void SetELossRelativeError(double value) { fELossRelativeError = value; }
   void SetNumSmoothingSteps(int value) { fNumSmoothingSteps = value; }

protected:
   virtual AtTrack
   GetModifiedTrack(const AtTrack &track, AtRawEvent *rawEvent = nullptr, AtEvent *event = nullptr) override;
   void ProcessHit(XYZPoint vertex, AtHit hit, AtTrack &modifiedTrack, AtRawEvent *rawEvent);
   void ProcessHit(XYZPoint vertex, AtHit hit, AtTrack &modifiedTrack);

   void GenerateBraggCurveHistogram(AtTrack &modifiedTrack);

   void InitializePSA();
};

#endif
