#ifndef AtMergeTask_H
#define AtMergeTask_H

#include <FairTask.h>

#include <Rtypes.h>
#include <TString.h>

#include "S800Ana.h"

#include <vector>

class FairLogger;
class S800Calc;
class TBuffer;
class TClass;
class TClonesArray;
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
   void SetATTPCClock(Bool_t value = kTRUE);
   void SetATTPCClockFreq(Double_t value);

   Int_t GetS800TsSize();
   Int_t GetMergedTsSize();

   Bool_t isInGlom(Long64_t ts1, Long64_t ts2);
   Bool_t isInPID(S800Calc *s800calc);

   virtual InitStatus Init();
   //    virtual void SetParContainers();
   virtual void Exec(Option_t *opt);

private:
   FairLogger *fLogger;

   TClonesArray *fRawEventArray{};
   S800Calc *fS800CalcBr;
   TFile *fS800file{};

   Int_t fTsEvtS800Size{}, fEvtMerged{}, fEvtDelta{5}, fTsDelta{1272};
   TString fS800File;
   Long64_t fATTPCTs0{}, fATTPCTsPrev{};
   std::vector<Long64_t> fS800Ts;
   std::vector<Double_t> fS800Evt;
   std::vector<Double_t> fParameters;
   std::vector<Double_t> fTofObjCorr;
   std::vector<Double_t> fMTDCObjRange;
   std::vector<Double_t> fMTDCXfRange;

   TF1 *fS800TsFunc{};
   Double_t fGlom{2};
   Double_t fATTPCClockFreq{};

   std::vector<TString> fcutPID1File;
   std::vector<TString> fcutPID2File;
   std::vector<TString> fcutPID3File;

   Bool_t fIsPersistence{false};
   Bool_t fUseATTPCClock{false};

   S800Ana fS800Ana;

   TGraph *fS800TsGraph{};

   ClassDef(AtMergeTask, 1);
};

#endif
