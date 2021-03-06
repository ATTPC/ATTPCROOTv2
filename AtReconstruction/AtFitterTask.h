/*********************************************************************
 *   Fitter Task AtFitterTask.hh			             *
 *   Author: Y. Ayyad ayyadlim@frib.msu.edu            	             *
 *   Log: 3/10/2021 					             *
 *								     *
 *********************************************************************/

#ifndef ATFITTERTASK
#define ATFITTERTASK

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

#include "AtFitter.h"
#include "AtGenfit.h"

// GENFIT2 classes
#include "AbsKalmanFitter.h"
#include "KalmanFitterRefTrack.h"
#include "DAF.h"
//#include "GFRaveVertexFactory.h"
#include "ConstField.h"
#include "FieldManager.h"
#include "MaterialEffects.h"
#include "TGeoMaterialInterface.h"
#include "MeasurementFactory.h"
#include "MeasurementProducer.h"
#include "EventDisplay.h"

#include "TClonesArray.h"

#include <exception>

class AtFitterTask : public FairTask {

public:
   AtFitterTask();
   ~AtFitterTask();

   virtual InitStatus Init();
   virtual void SetParContainers();
   virtual void Exec(Option_t *opt);

   void SetPersistence(Bool_t value = kTRUE);
   void SetFitterAlgorithm(Int_t value = 0);
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
   Bool_t fIsPersistence; //!< Persistence check variable
   FairLogger *fLogger;
   AtDigiPar *fPar;
   TClonesArray *fPatternEventArray;
   AtFITTER::AtFitter *fFitter;
   Int_t fFitterAlgorithm;

   TClonesArray *fGenfitTrackArray;
   std::vector<genfit::Track> *fGenfitTrackVector;

   std::size_t fEventCnt;
   Float_t fMagneticField;
   Int_t fMinIterations;
   Int_t fMaxIterations;
   Int_t fPDGCode;
   Float_t fMass;
   Int_t fAtomicNumber;
   Float_t fNumFitPoints;
   Float_t fMaxBrho;
   Float_t fMinBrho;
   std::string fELossFile;

   ClassDef(AtFitterTask, 1);
};

#endif
