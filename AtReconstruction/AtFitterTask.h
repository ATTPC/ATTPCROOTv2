/*********************************************************************
 *   Fitter Task AtFitterTask.hh			             *
 *   Author: Y. Ayyad ayyadlim@frib.msu.edu            	             *
 *   Log: 3/10/2021 					             *
 *								     *
 *********************************************************************/

#ifndef ATFITTERTASK
#define ATFITTERTASK

#include <FairTask.h>

#include <Rtypes.h>

#include <cstddef>
#include <string>
#include <vector>

class AtDigiPar;
class FairLogger;
class TBuffer;
class TClass;
class TClonesArray;
class TMemberInspector;

namespace AtFITTER {
class AtFitter;
} // namespace AtFITTER
namespace genfit {
class Track;
} // namespace genfit

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
   AtDigiPar *fPar{nullptr};
   TClonesArray *fPatternEventArray;
   AtFITTER::AtFitter *fFitter{};
   Int_t fFitterAlgorithm{0};

   TClonesArray *fGenfitTrackArray;
   std::vector<genfit::Track> *fGenfitTrackVector;

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

   ClassDef(AtFitterTask, 1);
};

#endif
