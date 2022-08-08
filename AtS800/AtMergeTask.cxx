#include "AtMergeTask.h"

#include <FairLogger.h>
#include <FairTask.h>

#include <Math/ParamFunctor.h>
#include <TClonesArray.h>
#include <TCollection.h>
#include <TFile.h>
#include <TList.h>
#include <TObject.h>

// FAIRROOT classes
#include "AtRawEvent.h"

#include <FairRootManager.h>

#include <TCutG.h>
#include <TF1.h>
#include <TGraph.h>
#include <TKey.h>
#include <TTreeReader.h>
#include <TTreeReaderValue.h>

#include "S800Calc.h"

#include <algorithm>
#include <cmath>
#include <iostream>

ClassImp(AtMergeTask);

AtMergeTask::AtMergeTask() : fLogger(FairLogger::GetLogger()), fS800CalcBr(new S800Calc)
{
   fcutPID1.clear();
   fcutPID1File.clear();
   fcutPID2.clear();
   fcutPID2File.clear();
   fcutPID3.clear();
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
   fSetCut1 = kTRUE;
}
void AtMergeTask::SetPID2cut(TString file)
{
   fcutPID2File.push_back(file);
   fSetCut2 = kTRUE;
}
void AtMergeTask::SetPID3cut(TString file)
{
   fcutPID3File.push_back(file);
   fSetCut3 = kTRUE;
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

Int_t AtMergeTask::GetS800TsSize()
{
   return fTsEvtS800Size;
}
Int_t AtMergeTask::GetMergedTsSize()
{
   return fEvtMerged;
}
vector<Double_t> AtMergeTask::GetTofObjCorr()
{
   return fTofObjCorr;
}
vector<Double_t> AtMergeTask::GetMTDCObjRange()
{
   return fMTDCObjRange;
}
vector<Double_t> AtMergeTask::GetMTDCXfRange()
{
   return fMTDCXfRange;
}
vector<Double_t> AtMergeTask::GetParameters()
{
   return fParameters;
}

Bool_t AtMergeTask::isInGlom(Long64_t ts1, Long64_t ts2)
{
   bool is = false;

   if (ts1 > 0 && ts2 > 0 && fabs(ts1 - ts2) < fGlom)
      is = true;
   return is;
}

Bool_t AtMergeTask::isInPID(S800Calc *s800calc)
{
   /*
   Double_t x0_corr_tof = fParameters.at(0);
   Double_t afp_corr_tof = fParameters.at(1);
   Double_t afp_corr_dE = fParameters.at(2);
   Double_t x0_corr_dE = fParameters.at(3);
   Double_t rf_offset = fParameters.at(4);
   Double_t corrGainE1up = fParameters.at(5);
   Double_t corrGainE1down = fParameters.at(6);
   */

   // Double_t S800_timeRf = s800calc->GetMultiHitTOF()->GetFirstRfHit();
   // Double_t S800_timeE1up = s800calc->GetMultiHitTOF()->GetFirstE1UpHit();
   // Double_t S800_timeE1down = s800calc->GetMultiHitTOF()->GetFirstE1DownHit();
   // Double_t S800_timeE1 = sqrt( (corrGainE1up*S800_timeE1up) * (corrGainE1down*S800_timeE1down) );
   // Double_t S800_timeXf = s800calc->GetMultiHitTOF()->GetFirstXfHit();
   // Double_t S800_timeObj = s800calc->GetMultiHitTOF()->GetFirstObjHit();
   vector<Float_t> S800_timeMTDCObj = s800calc->GetMultiHitTOF()->GetMTDCObj();
   vector<Float_t> S800_timeMTDCXf = s800calc->GetMultiHitTOF()->GetMTDCXf();
   Float_t S800_timeObjSelect = -999;
   Float_t S800_timeXfSelect = -999;
   Float_t ObjCorr = -999;

   Double_t S800_x0 = s800calc->GetCRDC(0)->GetX();
   Double_t S800_x1 = s800calc->GetCRDC(1)->GetX();
   // Double_t S800_y0 = s800calc->GetCRDC(0)->GetY();
   // Double_t S800_y1 = s800calc->GetCRDC(1)->GetY();

   // Double_t S800_E1up = s800calc->GetSCINT(0)->GetDEup();
   // Double_t S800_E1down = s800calc->GetSCINT(0)->GetDEdown();

   Double_t S800_ICSum = s800calc->GetIC()->GetSum();

   Double_t S800_afp = atan((S800_x1 - S800_x0) / 1073.);
   // Double_t S800_bfp = atan( (S800_y1-S800_y0)/1073. );
   // Double_t S800_tofCorr = S800_tof + x0_corr_tof*S800_x0 + afp_corr_tof*S800_afp;// - rf_offset;
   // Double_t S800_dE = s800calc->GetSCINT(0)->GetDE();//check if is this scint (0)
   // Double_t S800_dE = sqrt( (corrGainE1up*S800_E1up) * (corrGainE1down* S800_E1down ) );
   // Double_t S800_dECorr = S800_dE + afp_corr_dE*S800_afp + x0_corr_dE*fabs(S800_x0);

   Bool_t is = kFALSE;
   Int_t InCondition1 = 0;
   Int_t InCondition2 = 0;
   Int_t InCondition3 = 0;
   Int_t CondMTDCXfObj = 0;

   //----------- New 10/01 -------------------------------------
   for (float k : S800_timeMTDCXf) {
      if (k > fMTDCXfRange.at(0) && k < fMTDCXfRange.at(1))
         S800_timeXfSelect = k; // 140 to 230
   }
   for (float k : S800_timeMTDCObj) {
      if (k > fMTDCObjRange.at(0) && k < fMTDCObjRange.at(1))
         S800_timeObjSelect = k; //-75 to 0
   }                             //-115 to -20

   Double_t XfObj_tof = S800_timeXfSelect - S800_timeObjSelect;
   if (S800_timeXfSelect != -999 && S800_timeObjSelect != -999) {
      XfObj_tof = S800_timeXfSelect - S800_timeObjSelect;
      CondMTDCXfObj = 1;
   }

   if (CondMTDCXfObj && std::isnan(S800_x0) == 0 && std::isnan(S800_afp) == 0 && std::isnan(S800_ICSum) == 0) {
      ObjCorr = S800_timeObjSelect + fTofObjCorr.at(0) * S800_afp + fTofObjCorr.at(1) * S800_x0; // 100, 0.009
   }

   for (auto &w : fcutPID1)
      if (ObjCorr != -999 && w->IsInside(ObjCorr, XfObj_tof))
         InCondition1 += 1; // or of PID1
   for (auto &w : fcutPID2)
      if (ObjCorr != -999 && w->IsInside(S800_x0, S800_afp))
         InCondition2 += 1; // or of PID2
   for (auto &w : fcutPID3)
      if (ObjCorr != -999 && w->IsInside(ObjCorr, S800_ICSum))
         InCondition3 += 1; // or of PID3

   if (!fSetCut1)
      InCondition1 = 1;
   if (!fSetCut2)
      InCondition2 = 1;
   if (!fSetCut3)
      InCondition3 = 1;

   std::cout << " Number of TCutG files  " << fcutPID1.size() << " " << fcutPID2.size() << " " << fcutPID3.size() << " "
             << InCondition1 << " " << InCondition2 << " " << InCondition3 << '\n';

   Int_t AndofCon = InCondition1 * InCondition2 * InCondition3;

   // if(InCondition1){
   // std::cout<<"InCondition1 TRUE !"<<std::endl;
   //}
   if (AndofCon) {
      is = kTRUE;
      std::cout << "AndOfCond                                          TRUE   TRUE   TRUE !!!" << std::endl;
   }
   //----------- New 10/01 -------------------------------------
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

   fS800file = new TFile(fS800File); // NOLINT belongs to ROOT
   TTreeReader reader1("caltree", fS800file);
   TTreeReaderValue<Long64_t> ts(reader1, "fts");

   while (reader1.Next()) {
      fS800Ts.push_back((Long64_t)*ts);
      // fS800Ts.push_back((Long64_t) *ts - fTsDelta);//special for run180 e18027
      fS800Evt.push_back((Double_t)fTsEvtS800Size);
      if (fTsEvtS800Size < 20)
         std::cout << "Ts S800 " << fS800Ts.at(fTsEvtS800Size) << std::endl;
      fTsEvtS800Size++;
   }
   // ioMan -> RegisterAny("s800cal", fS800CalcBr, fIsPersistence);
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

   // auto c2 = new TCanvas("c2", "c2", 800, 800);
   // c2->cd();
   // gROOT->ProcessLine(".x "+fcutPIDFile);//
   // fcutPID = (TCutG*)gROOT->GetListOfSpecials()->FindObject("CUTG");
   // fcutPID->SetName("fcutPID");

   for (auto &w : fcutPID1File) {
      TFile f(w);
      TIter next(f.GetListOfKeys());
      TKey *key = nullptr;

      while ((key = dynamic_cast<TKey *>(next()))) {
         cout << "PID1 Loading Cut file:  " << key->GetName() << endl;
         fcutPID1.push_back(dynamic_cast<TCutG *>(f.Get(key->GetName())));
      }
   }

   for (auto &w : fcutPID2File) {
      TFile f(w);
      TIter next(f.GetListOfKeys());
      TKey *key = nullptr;

      while ((key = dynamic_cast<TKey *>(next()))) {
         cout << "PID2 Loading Cut file:  " << key->GetName() << endl;
         fcutPID2.push_back(dynamic_cast<TCutG *>(f.Get(key->GetName())));
      }
   }

   for (auto &w : fcutPID3File) {
      TFile f(w);
      TIter next(f.GetListOfKeys());
      TKey *key = nullptr;

      while ((key = dynamic_cast<TKey *>(next()))) {
         cout << "PID3 Loading Cut file:  " << key->GetName() << endl;
         fcutPID3.push_back(dynamic_cast<TCutG *>(f.Get(key->GetName())));
      }
   }
   //----------- New 10/01 -------------------------------------

   return kSUCCESS;
}

/*
void
AtMergeTask::SetParContainers()
{


FairRun *run = FairRun::Instance();
if (!run)
fLogger -> Fatal(MESSAGE_ORIGIN, "No analysis run!");

FairRuntimeDb *db = run -> GetRuntimeDb();
if (!db)
fLogger -> Fatal(MESSAGE_ORIGIN, "No runtime database!");

fPar = (AtDigiPar *) db -> getContainer("AtDigiPar");
if (!fPar)
fLogger -> Fatal(MESSAGE_ORIGIN, "AtDigiPar not found!!");
}
*/

void AtMergeTask::Exec(Option_t *opt)
{
   // if ( fS800CalcBr == NULL ) std::cout<<"fS800CalcBr NULL"<<std::endl;

   fS800CalcBr->Clear();

   if (fRawEventArray->GetEntriesFast() == 0)
      return;

   auto *rawEvent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));
   Long64_t AtTPCTs = rawEvent->GetTimestamp();
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
      std::cout << "NO TS MAtCHING          !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

   if (S800EvtMatch > 0) {
      TTreeReader reader2("caltree", fS800file);
      TTreeReaderValue<S800Calc> *readerValueS800Calc = nullptr;
      readerValueS800Calc = new TTreeReaderValue<S800Calc>(reader2, "s800calc"); // NOLINT

      reader2.SetEntry(S800EvtMatch);

      *fS800CalcBr = (S800Calc)*readerValueS800Calc->Get();

      Bool_t isIn = kFALSE;
      isIn = isInPID(fS800CalcBr);
      fS800CalcBr->SetIsInCut(isIn);
      rawEvent->SetIsExtGate(isIn);
   }
}
