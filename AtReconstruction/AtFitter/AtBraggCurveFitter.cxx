#include "AtBraggCurveFitter.h"

#include "AtTrack.h"
#include "AtTrackingEvent.h"
#include "AtPatternEvent.h"

#include <TMinuit.h>
#include <TSystem.h>

#include <algorithm>

// Initialization of all static members, so that AtReconstruction doesn't die.
EventFit::AtBraggCurveFitter::ELossModelsVector EventFit::AtBraggCurveFitter::fELossModels;
int EventFit::AtBraggCurveFitter::fProjectileIdx{0};
int EventFit::AtBraggCurveFitter::fValuesPerBin{200};
std::vector<double> EventFit::AtBraggCurveFitter::fExperimentalIntegratedELossValues;
std::vector<double> EventFit::AtBraggCurveFitter::fExperimentalRangeValues;
std::vector<double> EventFit::AtBraggCurveFitter::fExperimentalIntegratedELossErrors;
int EventFit::AtBraggCurveFitter::fNBins{0};
double EventFit::AtBraggCurveFitter::fBinSize{0};
double EventFit::AtBraggCurveFitter::fMinimumRange{30};
double EventFit::AtBraggCurveFitter::fEstimatedAmplitudeFactor{3.5e3}; // ADC/MeV
double EventFit::AtBraggCurveFitter::fEstimatedAmplitudeFactorPrecision{3e2}; // ADC/MeV

EventFit::AtBraggCurveFitter::AtBraggCurveFitter(ELossModelsVector eLossModels)
{
   fPunchThroughChecker = new AtTools::AtPunchThroughChecker();
   fELossModels = std::move(eLossModels);
}

EventFit::AtBraggCurveFitter::~AtBraggCurveFitter()
{
   delete fPunchThroughChecker;
   delete fMinuit;
}

void EventFit::AtBraggCurveFitter::FitEvent(AtTrackingEvent *trackingEvent, AtPatternEvent *patternEvent, AtFitMetadata *fitMetadata, AtRawEvent *rawEvent, AtEvent *event)
{
   // Check if fMinuit was initialised.
   if (fMinuit == nullptr) {
      LOG(warning) << " Minuit was not initialised! Please, check if you ran the Init() function of the fitter before passing it to the fitter task! Initialising now!";
      Init();
   }

   EventFit::AtFitter::FitEvent(trackingEvent, patternEvent, fitMetadata, rawEvent, event);
}

void EventFit::AtBraggCurveFitter::Init()
{
   gSystem->Load("libMinuit");

   // Set up the TMinuit.
   fMinuit = new TMinuit(2); // 2 parameters -> kinetic energy and amplitude factor.
   fMinuit->SetFCN(BraggFCN);
   fArglist[0] = fMinuitMaxCalls;
   fArglist[1] = fMinuitTolerance;
}

AtFittedTrack *EventFit::AtBraggCurveFitter::GetFittedTrack(AtTrack *track, AtFitMetadata *fitMetadata,
                                                            AtRawEvent *rawEvent, AtEvent *event)
{

   // First thing, we extract the Bragg curve ELoss information from the AtTrack.
   AtTrack::BraggCurve braggCurve = track->GetBraggCurve();
   fExperimentalIntegratedELossValues = braggCurve.IntegratedELossValues;
   fExperimentalRangeValues = braggCurve.RangeValues;
   fExperimentalIntegratedELossErrors = braggCurve.ELossErrors;
   fNBins = braggCurve.nBins;
   fBinSize = braggCurve.binSize;

   // Check for punsh through.
   Bool_t isPunchThrough = fPunchThroughChecker->IsPunchThrough(track);

   // In case the AtTrack has punched through or the ELoss profile has not been reconstructed for any reason, we do not fit and return "empty" metadatas and fitted track.
   if (isPunchThrough || !fBinSize) {
      if (isPunchThrough)
         LOG(info) << "Track with ID " << track->GetTrackID() << " has punched through. Skipping the fitting and adding empty fit metadatas!";
      else
         LOG(info) << "Track with ID " << track->GetTrackID() << " has does not have a reconstructed ELoss profile. Skipping the fitting and adding empty fit metadatas!";

      // We add an empty entry to the AtFitMetadata for each ELoss model anyways.
      if (fitMetadata) {
         TrackMetadatasVector trackMetadatasVector;
         fProjectileIdx = 0;
         while (fProjectileIdx < fELossModels.size()) {
            std::unique_ptr<AtBraggFitMetadata> uniqueBraggFitMetadata = std::make_unique<AtBraggFitMetadata>();
            uniqueBraggFitMetadata->SetIsPunchThrough(isPunchThrough);
            uniqueBraggFitMetadata->SetIsReconstructedELoss(fBinSize);
            uniqueBraggFitMetadata->SetFitConverged(kFALSE);
            uniqueBraggFitMetadata->SetTrackID(track->GetTrackID());
            uniqueBraggFitMetadata->SetFitID(fProjectileIdx);

            trackMetadatasVector.push_back(std::move(uniqueBraggFitMetadata));

            fProjectileIdx++;
         }
         fitMetadata->SetTrackMetadatasVector(track->GetTrackID(), std::move(trackMetadatasVector));
      }

      // Also, empty AtBraggFitMetadata for the AtFittedTrack that is required.
      std::unique_ptr<AtBraggFitMetadata> uniqueBestFitMetadata = std::make_unique<AtBraggFitMetadata>();
      uniqueBestFitMetadata->SetIsPunchThrough(isPunchThrough);
      uniqueBestFitMetadata->SetIsReconstructedELoss(fBinSize);
      uniqueBestFitMetadata->SetFitConverged(kFALSE);
      uniqueBestFitMetadata->SetTrackID(track->GetTrackID());

      AtFittedTrack *notFittedTrack = new AtFittedTrack();
      notFittedTrack->SetTrackID(track->GetTrackID());
      notFittedTrack->SetTrackMetadata(std::move(uniqueBestFitMetadata));
      return notFittedTrack;
   }



   // The minimum range where we should be able to get ELoss values for.
   fMinimumRange = fHoleRadius / TMath::Sin(track->GetGeoTheta());

   // Get the index with maximum ELoss, and get the associated range.
   int maxELossIndex =
      std::max_element(fExperimentalIntegratedELossValues.begin(), fExperimentalIntegratedELossValues.end()) -
      fExperimentalIntegratedELossValues.begin();
   double estimatedRange = fExperimentalRangeValues[maxELossIndex];

   // Clear the set in case it's filled from previous track.
   BraggFitMetadatasSet trackMetadatasSet = std::set<AtBraggFitMetadata *, std::function<bool(AtBraggFitMetadata *, AtBraggFitMetadata *)>>(CompareTrackFitsFunction);

   // Now, we iterate over all possible particles that this AtTrack may be.
   fProjectileIdx = 0;
   while (fProjectileIdx < fELossModels.size()) {
      // We obtain the estimation of the kinetic energy from the estimated range, for this candidate particle.
      double estimatedKinE{fStartingEstimatedKinE};
      while (fELossModels[fProjectileIdx]->GetRange(estimatedKinE) < estimatedRange)
         estimatedKinE += fEstimatedKinEStep;

      fMinuit->DefineParameter(0, "kinE", estimatedKinE, fKinEPrecision, estimatedKinE - 2 * fKinEPrecision, estimatedKinE + 2 * fKinEPrecision);
      fMinuit->DefineParameter(1, "amplFactor", fEstimatedAmplitudeFactor, fEstimatedAmplitudeFactorPrecision,
                               fEstimatedAmplitudeFactor - fEstimatedAmplitudeFactorPrecision,
                               fEstimatedAmplitudeFactor + fEstimatedAmplitudeFactorPrecision); // ADC/MeV

      // We perform the fitting for the current candidate particle.
      fMinuit->mnexcm("SIMPLEX", fArglist, 2, fIerflg);

      // Extract the fit result and store it on the fit metadata of this track.
      Double_t fitPar[2], fitUnc[2];
      for (int i = 0; i < 2; i++)
         fMinuit->GetParameter(i, fitPar[i], fitUnc[i]);

      Double_t chi2, fedm, errdef;
      Int_t npari, nparx, istat;
      fMinuit->mnstat(chi2, fedm, errdef, npari, nparx, istat);

      AtBraggFitMetadata *braggFitMetadata = new AtBraggFitMetadata();
      braggFitMetadata->SetChi2(chi2);
      braggFitMetadata->SetTrackID(track->GetTrackID());
      braggFitMetadata->SetFitID(fProjectileIdx);
      braggFitMetadata->SetFitConverged(kTRUE); // I'm setting to true by default because I don't know how to check with minuit :C

      //braggFitMetadata->SetPValue(pvalue???); // will be calculated in the future.
      //braggFitMetadata->SetNdf(ndf???); // ""

      braggFitMetadata->SetELossModelName(fELossModels[fProjectileIdx]->GetELossModelName());
      braggFitMetadata->SetKineticEnergy(fitPar[0]);
      braggFitMetadata->SetKineticEnergyUncertainty(fitUnc[0]);
      braggFitMetadata->SetAmplitudeFactor(fitPar[1]);
      braggFitMetadata->SetAmplitudeFactorUncertainty(fitUnc[1]);
      braggFitMetadata->SetPDGCode(fELossModels[fProjectileIdx]->GetPDGCode());
      braggFitMetadata->SetAtomicMassNumber(fELossModels[fProjectileIdx]->GetAtomicMassNumber());
      braggFitMetadata->SetChargeNumber(fELossModels[fProjectileIdx]->GetChargeNumber());
      braggFitMetadata->SetMassAmu(fELossModels[fProjectileIdx]->GetMassAmu());
      braggFitMetadata->SetIsPunchThrough(isPunchThrough);
      braggFitMetadata->SetIsReconstructedELoss(fBinSize);

      // Compute the ELoss profile that best fits the experimental values and store them in the metadata.
      auto integratedELossValues = fELossModels[fProjectileIdx]->GetIntegratedELoss(fitPar[0], fBinSize, fValuesPerBin, 0.001, fNBins * fBinSize);
      braggFitMetadata->SetELossFitValues(integratedELossValues);

      // Add metadata to set.
      trackMetadatasSet.insert(braggFitMetadata);

      // Move to next candidate particle.
      fProjectileIdx++;
   }

   // Store the fit metadata in a vector, ordered based on the better fit criteria for this fitter.
   TrackMetadatasVector trackMetadatasVector;
   AtBraggFitMetadata *bestFitTrackMetadata{nullptr};
   for (auto trackMetadata : trackMetadatasSet) {
      if (bestFitTrackMetadata == nullptr)
         bestFitTrackMetadata = trackMetadata;
      std::unique_ptr<AtBraggFitMetadata> uniqueBraggFitMetadata(trackMetadata);
      trackMetadatasVector.push_back(std::move(uniqueBraggFitMetadata));
   }

   // Construct the AtFittedTrack based on the best fit, i.e. the first element in the set.
   AtFittedTrack *fittedTrack = new AtFittedTrack();
   fittedTrack->SetTrackID(track->GetTrackID());
   fittedTrack->SetKinematics(bestFitTrackMetadata->GetKineticEnergy(), track->GetGeoTheta(), track->GetGeoPhi());
   fittedTrack->SetParticleInfo(bestFitTrackMetadata->GetPDGCode().Data(), bestFitTrackMetadata->GetChargeNumber(), bestFitTrackMetadata->GetMassAmu());
   //fittedTrack->SetVertex(???); //TO-DO
   //fittedTrack->SetTrackPropertiesStruct(???); //TO-DO
   std::unique_ptr<AtBraggFitMetadata> uniqueBestFitMetadata = std::make_unique<AtBraggFitMetadata>(*bestFitTrackMetadata);
   fittedTrack->SetTrackMetadata(std::move(uniqueBestFitMetadata));

   // Add the corresponding vector of fit metadatas for this track to the AtFitMetadata of this event.
   if (fitMetadata)
      fitMetadata->SetTrackMetadatasVector(track->GetTrackID(), std::move(trackMetadatasVector));

   return fittedTrack;
}

bool EventFit::AtBraggCurveFitter::CompareTrackFitsFunction(AtBraggFitMetadata *braggMetadataA,
                                                            AtBraggFitMetadata *braggMetadataB)
{
   // Extract the relevant parameters for this comparison criteria.
   double amplitudeFactorA = braggMetadataA->GetAmplitudeFactor();
   double chi2A = braggMetadataA->GetChi2();
   double amplitudeFactorB = braggMetadataB->GetAmplitudeFactor();
   double chi2B = braggMetadataB->GetChi2();

   // First, we check the amplitude factor criteria.
   double minAmplitudeFactor = fEstimatedAmplitudeFactor - fEstimatedAmplitudeFactorPrecision;
   double maxAmplitudeFactor = fEstimatedAmplitudeFactor + fEstimatedAmplitudeFactorPrecision;
   bool amplitudeAInRange = minAmplitudeFactor <= amplitudeFactorA && amplitudeFactorA <= maxAmplitudeFactor;
   bool amplitudeBInRange = minAmplitudeFactor <= amplitudeFactorB && amplitudeFactorB <= maxAmplitudeFactor;

   // If either is in range and the other is not, we choose the one in range.
   if (amplitudeAInRange && !amplitudeBInRange)
      return true;

   if (!amplitudeAInRange && amplitudeBInRange)
      return false;

   // If both are in range or neither is in range, we order by chi2.
   if (chi2A <= chi2B)
      return true;
   return false;
}

void EventFit::AtBraggCurveFitter::BraggFCN(int &npar, double *gin, double &fval, double *par, int iflag)
{
   // Fitting parameters.
   double kineticEnergy = par[0];
   double amplitudeFactor = par[1];

   // Get the integrated dE/dx bin by bin.
   auto integratedELossValues = fELossModels[fProjectileIdx]->GetIntegratedELoss(kineticEnergy, fBinSize, fValuesPerBin, 0.001, fNBins * fBinSize);

   // Commpute the chi2 value.
   fval = 0;
   for (int i = 0; i < fExperimentalIntegratedELossValues.size(); i++) {
      double range = fExperimentalRangeValues[i] + fBinSize / 2.;
      double experimentalELossValue = fExperimentalIntegratedELossValues[i];
      double experimentalELossError = fExperimentalIntegratedELossErrors[i];
      double modelELossValue = integratedELossValues[i].first;

      // Check if bin lives outside the beam region.
      if (range < fMinimumRange)
         continue;

      // Check if there is ELoss in bin.
      if (experimentalELossValue == 0)
         continue;

      // Prevent dividing by 0.
      if (experimentalELossError > 0)
         fval += std::pow((experimentalELossValue - amplitudeFactor * modelELossValue) / experimentalELossError, 2);
   }
}
