#include "AtMergeTask.h"

#include "AtRawEvent.h"

#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairTask.h>

#include <Math/ParamFunctor.h>
#include <TClonesArray.h>
#include <TF1.h>
#include <TFile.h>
#include <TGraph.h>
#include <TObject.h>
#include <TTreeReader.h>
#include <TTreeReaderValue.h>

#include "S800Ana.h"
#include "S800Calc.h"

#include <algorithm>
#include <cmath>
#include <iostream>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

ClassImp(AtMergeTask);

AtMergeTask::AtMergeTask() : fLogger(FairLogger::GetLogger()), fS800CalcBr(new S800Calc)
{
   fcutPID1File.clear();
   fcutPID2File.clear();
   fcutPID3File.clear();

   fParameters.clear();
   fTofObjCorr.clear();
   fMTDCObjRange.clear();
   fMTDCXfRange.clear();
}

AtMergeTask::~AtMergeTask()
{
   fS800TsGraph->Delete();
   fS800TsFunc->Delete();
   fS800CalcBr->Delete();
   fRawEventArray->Delete();
   delete fS800file;
}

void AtMergeTask::SetPersistence(Bool_t value)
{
   fIsPersistence = value;
}
void AtMergeTask::SetS800File(TString file)
{
   fS800File = file;
}
void AtMergeTask::SetGlom(Double_t glom)
{
   fGlom = glom;
}
void AtMergeTask::SetOptiEvtDelta(Int_t EvtDelta)
{
   fEvtDelta = EvtDelta;
}
void AtMergeTask::SetPID1cut(TString file)
{
   fcutPID1File.push_back(file);
}
void AtMergeTask::SetPID2cut(TString file)
{
   fcutPID2File.push_back(file);
}
void AtMergeTask::SetPID3cut(TString file)
{
   fcutPID3File.push_back(file);
}
void AtMergeTask::SetTsDelta(Int_t TsDelta)
{
   fTsDelta = TsDelta;
}
void AtMergeTask::SetParameters(std::vector<Double_t> vec)
{
   fParameters = vec;
}
void AtMergeTask::SetTofObjCorr(std::vector<Double_t> vec)
{
   fTofObjCorr = vec;
}
void AtMergeTask::SetMTDCObjRange(std::vector<Double_t> vec)
{
   fMTDCObjRange = vec;
}
void AtMergeTask::SetMTDCXfRange(std::vector<Double_t> vec)
{
   fMTDCXfRange = vec;
}
void AtMergeTask::SetATTPCClock(Bool_t value)
{
   fUseATTPCClock = value;
}
void AtMergeTask::SetATTPCClockFreq(Double_t value)
{
   fATTPCClockFreq = value;
}

Int_t AtMergeTask::GetS800TsSize()
{
   return fTsEvtS800Size;
}
Int_t AtMergeTask::GetMergedTsSize()
{
   return fEvtMerged;
}

Bool_t AtMergeTask::isInGlom(Long64_t ts1, Long64_t ts2)
{
   bool is = false;

   if (ts1 > 0 && ts2 > 0 && fabs(ts1 - ts2) < fGlom)
      is = true;
   return is;
}

InitStatus AtMergeTask::Init()
{

   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(error) << "Cannot find RootManager!";
      return kERROR;
   }

   fRawEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject("AtRawEvent"));
   if (fRawEventArray == nullptr) {
      LOG(error) << "Cannot find AtRawEvent array!";
      return kERROR;
   }

   fTsEvtS800Size = 0;
   fEvtMerged = 0;
   fATTPCTs0 = -1;
   Long64_t S800Ts0 = 0; // special for use of internal AT-TPC TS
   fATTPCTsPrev = 0;

   fS800file = new TFile(fS800File); // NOLINT belongs to ROOT
   TTreeReader reader1("caltree", fS800file);
   TTreeReaderValue<Long64_t> ts(reader1, "fts");

   LOG(info) << cBLUE << "Loading S800 timestamps..." << cNORMAL;
   while (reader1.Next()) {
      fS800Ts.push_back((Long64_t)*ts);
      // fS800Ts.push_back((Long64_t) *ts - fTsDelta);//special for run180 e18027
      fS800Evt.push_back((Double_t)fTsEvtS800Size);
      //------- Special for using internal AT-TPC TS (ex: runs 144 to 168), comment if run149 (but keep the second
      // special part uncommented)
      if (fUseATTPCClock) {
         if (fTsEvtS800Size == 0)
            S800Ts0 = fS800Ts.at(0);
         fS800Ts.at(fTsEvtS800Size) -= S800Ts0;
         if (fTsEvtS800Size < 10)
            std::cout << "Ts S800 " << fS800Ts.at(fTsEvtS800Size) << " S800Ts0 " << S800Ts0 << std::endl;
      }
      //--------
      fTsEvtS800Size++;
   }
   ioMan->Register("s800cal", "S800", fS800CalcBr, fIsPersistence);

   vector<Double_t> S800_ts(fS800Ts.begin(), fS800Ts.end());
   // auto c1 = new TCanvas("c1", "c1", 800, 800);
   // gROOT->SetBatch(kTRUE);//kTRUE not display the plots
   fS800TsGraph =                                            // NOLINT
      new TGraph(fTsEvtS800Size, &S800_ts[0], &fS800Evt[0]); // fTsEvtS800Size instead of 80 (just for the test file)
   // make a function of S800Evt vs S800TS, used then to search the S800 matching TS only among few S800 events, faster
   // than looping on all the events.
   fS800TsFunc = new TF1( // NOLINT
      "fS800TsFunc", [&](double *x, double *) { return fS800TsGraph->Eval(x[0]); }, 0, S800_ts.back(), 0);
   // c1->cd();
   // fS800TsGraph->Draw("AL");
   // fS800TsFunc->Draw("same");

   fS800Ana.SetPID1cut(fcutPID1File);
   fS800Ana.SetPID2cut(fcutPID2File);
   fS800Ana.SetPID3cut(fcutPID3File);
   fS800Ana.SetMTDCXfRange(fMTDCXfRange);
   fS800Ana.SetMTDCObjRange(fMTDCObjRange);
   fS800Ana.SetTofObjCorr(fTofObjCorr);

   return kSUCCESS;
}

void AtMergeTask::Exec(Option_t *opt)
{
   fS800CalcBr->Clear();

   if (fRawEventArray->GetEntriesFast() == 0)
      return;

   auto *rawEvent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));
   Long64_t AtTPCTs = -1;
   if (!fUseATTPCClock) {
      if (rawEvent->GetTimestamps().size() == 1) {
         LOG(warning)
            << cYELLOW
            << " AtMergeTask : only TS based on internal AT-TPC clock available, check fUseATTPCClock unpacking macro"
            << cNORMAL << std::endl;
         return;
      } else
         AtTPCTs = rawEvent->GetTimestamp(1);
   }
   if (fUseATTPCClock) {
      AtTPCTs = rawEvent->GetTimestamp(0);
      if (fATTPCTs0 == -1)
         fATTPCTs0 = AtTPCTs; // special run 275, comment this line and uncomment the following line
      // if(fATTPCTs0==-1) fATTPCTs0=902487436452;// special run 275 the first event is not in s800 data so substract
      // the TS of the second evt
      AtTPCTs =
         (AtTPCTs - fATTPCTs0) / fATTPCClockFreq; // 9.9994347//run146 164 9.9994345 run155 9.999435 run160 9.999434
      // std::cout<<"debug "<<fCounter<<" "<<AtTPCTs<<" "<<fATTPCTsPrev<<" "<<AtTPCTs-fATTPCTsPrev<<std::endl;
      if ((AtTPCTs - fATTPCTsPrev) > 1e+8)
         AtTPCTs -= 429521035; // sometimes the ATTPC TS jump (?)
      if ((AtTPCTs - fATTPCTsPrev) < -1e+8)
         AtTPCTs += 429521035;
      fATTPCTsPrev = AtTPCTs;
   }
   int minj = 0, maxj = 0;
   Double_t S800EvtMatch = -1;
   minj = (int)fS800TsFunc->Eval(AtTPCTs) - fEvtDelta; // define the AtTPC entries range where the matching timestamp
                                                       // should be, to not loop over all the AtTPC entries.
   maxj = (int)fS800TsFunc->Eval(AtTPCTs) + fEvtDelta;

   std::cout << " TS AtTPC " << AtTPCTs << std::endl;

   for (int i = minj; i < maxj; i++) {
      if (i >= 0 && i < fTsEvtS800Size) {
         if (i > 0 && isInGlom(fS800Ts.at(i - 1), fS800Ts.at(i)))
            std::cout << " -- Warning -- Timestamp of consecutive entries from S800 root file within the glom"
                      << std::endl;
         else {
            // Is there a way to check that with the At-TPC "event by event" processing?
            /*if(isInGlom(TsEvtAtTPC.at(i-1),TsEvtAtTPC.at(i)) )
            {
            cout<<" -- Warning -- Timestamp of consecutive entries from AtTPC root file within the glom"<<endl;
          }
          else*/
            if (isInGlom(fS800Ts.at(i) + fTsDelta, AtTPCTs)) { // fTsDelta constant offset likely from the length of the
                                                               // sync signal between S800 and At-TPC
               // if(isInGlom(fS800Ts.at(i),AtTPCTs) ){//special run180
               S800EvtMatch = (int)fS800Evt.at(i);
               std::cout << " in glom " << minj << " " << maxj << " " << i << " " << fS800Ts.at(i) << " " << AtTPCTs
                         << " " << S800EvtMatch << " " << fS800TsFunc->Eval(AtTPCTs) << " " << AtTPCTs - fS800Ts.at(i)
                         << std::endl;
               fEvtMerged++;
               break;
            } else
               S800EvtMatch = -1;
            // std::cout<<" NOT in glom "<<minj<<" "<<maxj<<" "<<i<<" "<<fS800Ts.at(i)<<" "<<AtTPCTs<<"
            // "<<fS800Ts.at(i)-AtTPCTs<<" "<<S800EvtMatch<<std::endl;
         }
      }
   }

   if (S800EvtMatch < 0)
      LOG(warning) << cRED << "NO TS MATCHING !" << cNORMAL;

   if (S800EvtMatch > 0) {
      TTreeReader reader2("caltree", fS800file);
      TTreeReaderValue<S800Calc> *readerValueS800Calc = nullptr;
      readerValueS800Calc = new TTreeReaderValue<S800Calc>(reader2, "s800calc"); // NOLINT

      reader2.SetEntry(S800EvtMatch);

      *fS800CalcBr = (S800Calc)*readerValueS800Calc->Get();

      Bool_t isIn = kFALSE;
      isIn = fS800Ana.isInPID(fS800CalcBr);
      fS800CalcBr->SetIsInCut(isIn);
      rawEvent->SetIsExtGate(isIn);
   }
}
