#ifndef AtBRAGGCURVEFITTER_H
#define AtBRAGGCURVEFITTER_H

#include "AtBraggFitMetadata.h"
#include "AtELossModel.h"
#include "AtFittedTrack.h"
#include "AtFitter.h"
#include "AtPunchThroughChecker.h"

#include <TMinuit.h>

#include <string>
#include <utility>

namespace EventFit {

class AtBraggCurveFitter : public AtFitter {
public:
   using TrackMetadataPtr = std::unique_ptr<AtFitTrackMetadata>;
   using TrackMetadatasVector = std::vector<TrackMetadataPtr>;
   using BraggFitMetadatasSet = std::set<AtBraggFitMetadata *, std::function<bool(AtBraggFitMetadata *, AtBraggFitMetadata *)>>;
   using ELossModelPtr = std::unique_ptr<AtTools::AtELossModel>;
   using ELossModelsVector = std::vector<ELossModelPtr>;

protected:
   // The ELossModels to be used to obtain the theoretical ELoss values. Each ELoss model correspond to a different candidate particle.
   static ELossModelsVector fELossModels;
   static int fProjectileIdx;
   static int fValuesPerBin;

   // Information regarding the experimental Bragg curve.
   static std::vector<double> fExperimentalIntegratedELossValues;
   static std::vector<double> fExperimentalRangeValues;
   static std::vector<double> fExperimentalIntegratedELossErrors;
   static int fNBins;
   static double fBinSize;
   static double fMinimumRange;
   double fHoleRadius{30}; // mm

   // The AtPunchThroughChecker that will check if a track is punching through or not.
   AtTools::AtPunchThroughChecker *fPunchThroughChecker;

   // The TMinuit which will perform the minimization.
   TMinuit *fMinuit{nullptr};
   double fArglist[10];
   double fMinuitMaxCalls{5000};
   double fMinuitTolerance{0.1};
   int fIerflg{0};

   // What the amplitude factor should approximate to, and with which precision.
   static double fEstimatedAmplitudeFactor; // ADC/MeV
   static double fEstimatedAmplitudeFactorPrecision; // ADC/MeV

   // Parameters related to the estimation of the initial kinetic energy guess. Fitting precision too.
   double fStartingEstimatedKinE{0.1}; // MeV
   double fEstimatedKinEStep{0.01}; // MeV
   double fKinEPrecision{0.25}; // MeV

public:
   AtBraggCurveFitter(ELossModelsVector eLossModels);
   ~AtBraggCurveFitter();

   virtual void FitEvent(AtTrackingEvent *trackingEvent, AtPatternEvent *patternEvent,
                         AtFitMetadata *fitMetadata = nullptr, AtRawEvent *rawEvent = nullptr,
                         AtEvent *event = nullptr) override;
   virtual void Init() override;

   void SetValuesPerBin(int value) { fValuesPerBin = value; }
   void SetMinuitMaxCalls(double value) { fMinuitMaxCalls = value; }
   void SetMinuitTolerance(double value) { fMinuitTolerance = value; }
   void SetEstimatedAmplitudeFactor(double value) { fEstimatedAmplitudeFactor = value; }
   void SetEstimatedAmplitudeFactorPrecision(double value) { fEstimatedAmplitudeFactorPrecision = value; }
   void SetStartingEstimatedKinE(double value) { fStartingEstimatedKinE = value; }
   void SetEstimatedKinEStep(double value) { fEstimatedKinEStep = value; }
   void SetKinEPrecision(double value) { fKinEPrecision = value; }
   void SetDistanceThreshold(double value) { fPunchThroughChecker->SetDistanceThreshold(value); }

protected:
   virtual AtFittedTrack *GetFittedTrack(AtTrack *track, AtFitMetadata *fitMetadata = nullptr,
                                         AtRawEvent *rawEvent = nullptr, AtEvent *event = nullptr) override;

   // Compare function that will be used to sort the fit results for a given track.
   static bool
   CompareTrackFitsFunction(AtBraggFitMetadata *braggMetadataA, AtBraggFitMetadata *braggMetadataB);

   // FCN function that will be minimized by TMinuit.
   static void BraggFCN(int &npar, double *gin, double &fval, double *par, int iflag);
};

} // namespace EventFit

#endif
