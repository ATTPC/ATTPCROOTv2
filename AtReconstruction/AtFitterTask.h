/*********************************************************************
 *   Fitter Task AtFitterTask.hh			             *
 *   Author: Y. Ayyad ayyadlim@frib.msu.edu            	             *
 *   Log: 3/10/2021 					             *
 *								     *
 *********************************************************************/

#ifndef ATFITTERTASK
#define ATFITTERTASK

#include "AtFormat.h"
#include "AtKinematics.h"
#include "AtParsers.h"

#include <FairTask.h>

#include <Rtypes.h>

#include "AbsFitterInfo.h"
#include "AbsKalmanFitter.h"
#include "ConstField.h"
#include "DAF.h"
#include "EventDisplay.h"
#include "Exception.h"
#include "FairLogger.h"
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRunAna.h"
#include "FieldManager.h"
#include "FitStatus.h"
#include "KalmanFitStatus.h"
#include "KalmanFitterInfo.h"
#include "KalmanFitterRefTrack.h"
#include "MaterialEffects.h"
#include "MeasuredStateOnPlane.h"
#include "MeasurementFactory.h"
#include "MeasurementOnPlane.h"
#include "MeasurementProducer.h"

#include <cstddef>
#include <string>
#include <vector>

class AtDigiPar;
class FairLogger;
class TBuffer;
class TClass;
class TClonesArray;
class TMemberInspector;
class AtTrack;

namespace AtTools {
class AtTrackTransformer;
} // namespace AtTools
namespace AtFITTER {
class AtFitter;
} // namespace AtFITTER
namespace genfit {
class Track;
} // namespace genfit

class AtFitterTask : public FairTask {

public:
   // AtFitterTask();
   ~AtFitterTask() = default;
   AtFitterTask(std::unique_ptr<AtFITTER::AtFitter> fitter);

   void SetInputBranch(TString branchName);
   void SetOutputBranch(TString branchName);
   void SetPersistence(Bool_t value = kTRUE);

   virtual InitStatus Init();
   virtual void SetParContainers();
   virtual void Exec(Option_t *opt);

   inline void SetMagneticField(Float_t magfield) { fMagneticField = magfield; }
   inline void SetMinIterations(Int_t miniter) { fMinIterations = miniter; }
   inline void SetMaxIterations(Int_t maxiter) { fMaxIterations = maxiter; }
   inline void SetPDGCode(Int_t pdgcode) { fPDGCode = pdgcode; }
   inline void SetMass(Float_t mass) { fMass = mass; }
   inline void SetAtomicNumber(Int_t znum) { fAtomicNumber = znum; }
   inline void SetNumFitPoints(Float_t numpoints) { fNumFitPoints = numpoints; }
   inline void SetMaxBrho(Float_t maxbrho) { fMaxBrho = maxbrho; }
   inline void SetMinBhro(Float_t minbrho) { fMinBrho = minbrho; }
   inline void SetELossFile(std::string file) { fELossFile = file; }

private:
   TString fInputBranchName;
   TString fOutputBranchName;

   Bool_t fIsPersistence; //!< Persistence check variable

   std::unique_ptr<AtFITTER::AtFitter> fFitter;
   std::unique_ptr<AtTools::AtTrackTransformer> fTrackTransformer;
   std::shared_ptr<AtTools::AtKinematics> fKinematics;

   AtDigiPar *fPar{nullptr};
   TClonesArray *fPatternEventArray;
   TClonesArray fTrackingEventArray;

   // Need input from macro
   std::size_t fEventCnt{0};
   Float_t fMagneticField{2.0};
   Int_t fMinIterations{5};
   Int_t fMaxIterations{20};
   Int_t fPDGCode{2212};
   Float_t fMass{1.00727646};
   Int_t fAtomicNumber{1};
   Float_t fNumFitPoints{0.90};
   Float_t fMaxBrho{3.0};
   Float_t fMinBrho{0.01};
   std::string fELossFile{""};

   Bool_t fSimulationConv{0};
   Bool_t fEnableMerging{0};
   Bool_t fEnableSingleVertexTrack{0};
   Bool_t fEnableReclustering{0};
   Double_t fClusterSize{0};
   Double_t fClusterRadius{0};

   enum Exp { e20020, e20009, a1954, a1975, a1954b };
   Exp fExpNum;
   std::vector<AtTools::IonFitInfo> *ionList;

   // To move somewhere else
   std::vector<AtTrack *> FindSingleTracks(std::vector<AtTrack *> &tracks);
   Double_t CenterDistance(AtTrack *trA, AtTrack *trB);
   Bool_t CompareTracks(AtTrack *trA, AtTrack *trB);
   Bool_t CheckOverlap(AtTrack *trA, AtTrack *trB);

   ClassDef(AtFitterTask, 1);
};

#endif
