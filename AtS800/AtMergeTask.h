#ifndef AtMergeTask_H
#define AtMergeTask_H

#include <Rtypes.h>
#include <RtypesCore.h>
#include <TString.h>
#include <vector>

// FAIRROOT classes
#include <FairTask.h>

// AtTPCROOT classes

class FairLogger;
class S800Calc;
class TBuffer;
class TClass;
class TClonesArray;
class TCutG;
class TF1;
class TFile;
class TGraph;
class TMemberInspector;

class AtMergeTask : public FairTask {

public:
   AtMergeTask();
   ~AtMergeTask();

   void SetPersistence(Bool_t value = kTRUE);
   void SetS800File(TString file);
   void SetGlom(Double_t glom);
   void SetOptiEvtDelta(Int_t EvtDelta);
   void SetPID1cut(TString file);
   void SetPID2cut(TString file);
   void SetPID3cut(TString file);
   void SetTsDelta(Int_t TsDelta);
   void SetParameters(std::vector<Double_t> vec);
   void SetTofObjCorr(std::vector<Double_t> vec);
   void SetMTDCObjRange(std::vector<Double_t> vec);
   void SetMTDCXfRange(std::vector<Double_t> vec);

   Int_t GetS800TsSize();
   Int_t GetMergedTsSize();
   std::vector<Double_t> GetParameters();
   std::vector<Double_t> GetTofObjCorr();
   std::vector<Double_t> GetMTDCObjRange();
   std::vector<Double_t> GetMTDCXfRange();

   Bool_t isInGlom(Long64_t ts1, Long64_t ts2);
   Bool_t isInPID(S800Calc *s800calc);

   virtual InitStatus Init();
   //    virtual void SetParContainers();
   virtual void Exec(Option_t *opt);

private:
   FairLogger *fLogger;

   TClonesArray *fRawEventArray;
   S800Calc *fS800CalcBr;
   TFile *fS800file;

   Int_t fTsEvtS800Size, fEvtMerged, fEvtDelta, fTsDelta;
   TString fS800File;
   std::vector<Long64_t> fS800Ts;
   std::vector<Double_t> fS800Evt;
   std::vector<Double_t> fParameters;
   std::vector<Double_t> fTofObjCorr;
   std::vector<Double_t> fMTDCObjRange;
   std::vector<Double_t> fMTDCXfRange;

   TF1 *fS800TsFunc;
   // TF1 *fOptiFit;
   Double_t fGlom;

   std::vector<TCutG *> fcutPID1;
   std::vector<TCutG *> fcutPID2;
   std::vector<TCutG *> fcutPID3;
   std::vector<TString> fcutPID1File;
   std::vector<TString> fcutPID2File;
   std::vector<TString> fcutPID3File;

   Bool_t fIsPersistence;
   Bool_t fSetCut1;
   Bool_t fSetCut2;
   Bool_t fSetCut3;

   TGraph *fS800TsGraph;

   ClassDef(AtMergeTask, 1);
};

#endif
