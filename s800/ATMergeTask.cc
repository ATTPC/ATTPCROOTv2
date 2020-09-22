#include "ATMergeTask.hh"

// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"

// ROOT classes
#include "TCanvas.h"
#include "TGraph.h"
#include "TF1.h"
#include "TMath.h"
#include "TCutG.h"
#include "TTreeReader.h"
#include "TTreeReaderValue.h"
#include "TTreeReaderArray.h"

// S800 Classes
#include "S800Event.hh"
#include "S800.hh"
#include "S800Calc.hh"

#include <iostream>

ClassImp(ATMergeTask);

ATMergeTask::ATMergeTask()
{

  fLogger = FairLogger::GetLogger();

  fIsPersistence=kFALSE;
  fEvtDelta=100;
  fGlom=2;

  fS800CalcBr = new S800Calc;

  fcutPID.clear();
  fcutPIDFile.clear();

}

ATMergeTask::~ATMergeTask()
{
  fS800CalcBr->Delete();
  delete fS800file;
}

void   ATMergeTask::SetPersistence(Bool_t value)                  { fIsPersistence     = value; }
void   ATMergeTask::SetS800File(TString file)                  { fS800File     = file; }
void   ATMergeTask::SetGlom(Double_t glom)                  { fGlom     = glom; }
void   ATMergeTask::SetOptiEvtDelta(Int_t EvtDelta)                  { fEvtDelta     = EvtDelta; }
void   ATMergeTask::SetPIDcut(TString file)                  { fcutPIDFile.push_back(file); }

Bool_t ATMergeTask::isInGlom(Long64_t ts1, Long64_t ts2)
{
  bool is=false;

  if(ts1>0 && ts2>0 && fabs(ts1-ts2)<fGlom) is=true;
  return is;
}

Bool_t ATMergeTask::isInPID(S800Calc *s800calc)
{
  Double_t x0_corr_tof = 0.101259;
  Double_t afp_corr_tof = 1177.02;
  Double_t afp_corr_dE = 61.7607;
  Double_t x0_corr_dE = -0.0403;
  Double_t rf_offset = 0.0;
  Double_t S800_rf = s800calc->GetMultiHitTOF()->GetFirstRfHit();
  Double_t S800_x0 = s800calc->GetCRDC(0)->GetXfit();
  Double_t S800_x1 = s800calc->GetCRDC(1)->GetXfit();
  Double_t S800_y0 = s800calc->GetCRDC(0)->GetY();
  Double_t S800_y1 = s800calc->GetCRDC(1)->GetY();
  Double_t S800_E1up = s800calc->GetSCINT(0)->GetDEup(); //check this par
  Double_t S800_E1down = s800calc->GetSCINT(0)->GetDEdown(); //check this par
  Double_t S800_tof = S800_rf;//might change
  Double_t S800_afp = atan( (S800_x1-S800_x0)/1073. );
  Double_t S800_bfp = atan( (S800_y1-S800_y0)/1073. );
  Double_t S800_tofCorr = S800_tof + x0_corr_tof*S800_x0 + afp_corr_tof*S800_afp - rf_offset;
  //Double_t S800_dE = s800calc->GetSCINT(0)->GetDE();//check if is this scint (0)
  Double_t S800_dE = sqrt( (0.6754*S800_E1up) * ( 1.0 * S800_E1down ) );
  Double_t S800_dECorr = S800_dE + afp_corr_dE*S800_afp + x0_corr_dE*fabs(S800_x0);

  Bool_t is=kFALSE;
  Int_t InCondition = 0;

  for(Int_t w=0; w<fcutPID.size();w++) InCondition += fcutPID[w]->IsInside(S800_tofCorr,S800_dECorr); //or of IsInside

  //std::cout <<" Number of TCutG files  "<<fcutPID.size()<<"   "<<InCondition<< '\n';

  if(std::isnan(S800_tofCorr)==0)
    if(InCondition){
      is=kTRUE;
    }
  return is;
}



InitStatus
ATMergeTask::Init()
{

  FairRootManager *ioMan = FairRootManager::Instance();
  if (ioMan == 0) {
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find RootManager!");
    return kERROR;
  }

  fRawEventArray = (TClonesArray *) ioMan -> GetObject("ATRawEvent");
  if (fRawEventArray == 0) {
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find ATRawEvent array!");
    return kERROR;
  }


  fTsEvtS800Size=0;

  fS800file = new TFile(fS800File);
  TTreeReader reader1("caltree", fS800file);
  TTreeReaderValue<Long64_t> ts(reader1,"fts");

  while (reader1.Next()) {
    fS800Ts.push_back((Long64_t) *ts);
    fS800Evt.push_back((Double_t) fTsEvtS800Size);
    //if(fTsEvtS800Size<100) std::cout<<"Ts S800 "<<fS800Ts.at(fTsEvtS800Size)<<std::endl;
    fTsEvtS800Size++;
  }
  ioMan -> RegisterAny("s800cal", fS800CalcBr, fIsPersistence);

  //std::cout << "  File :  " << fS800File<< " Events : " << fTsEvtS800Size <<"  " <<std::endl;


  /* fit of the TS vs entry number, this should be monotonic increasing,
  the function is used then to reduce the size of the S800 event loop
  */
  //I think TS has to be converted into double for TGraph, but would prefer to not have this extra step
  vector <Double_t> S800_ts(fS800Ts.begin(),fS800Ts.end());
  //auto c1 = new TCanvas("c1", "c1", 800, 800);
  Double_t par_fit[2];
  gROOT->SetBatch(kTRUE);//kTRUE not display the plots
  fOptiFit = new TF1("fOptiFit","[1]*x + [0]",0,7E+9);//poly 1 seems relatively ok, fit do not need to be very precise,
  //the fit limit might be an important parmeter
  //TF1 *f1 = new TF1("f1","[2]*x*x + [1]*x + [0]",0,1000);//ploy 2
  TGraph *gS800 = new TGraph(80, &S800_ts[0], &fS800Evt[0]);//fTsEvtS800Size instead of 80 (just for the test file)
  gS800->Fit("fOptiFit");//might be biased by the TS default value
  fOptiFit->GetParameters(&par_fit[0]);
  fOptiFit->SetParameters(par_fit[0],par_fit[1]);
  //c1->cd();
  //gS800->Draw("AL");
  //f1->Draw("same");

  //auto c2 = new TCanvas("c2", "c2", 800, 800);
  //c2->cd();
  //gROOT->ProcessLine(".x "+fcutPIDFile);//
  //fcutPID = (TCutG*)gROOT->GetListOfSpecials()->FindObject("CUTG");
  //fcutPID->SetName("fcutPID");

  for(Int_t w=0; w<fcutPIDFile.size();w++){
    TFile f(fcutPIDFile[w]);
    TIter next(f.GetListOfKeys());
    TKey *key;

    while ((key=(TKey*)next())) {
      cout<<"Loading Cut file:  "<<key->GetName()<<endl;
      fcutPID.push_back( (TCutG*)f.Get(key->GetName()) );
    }

  }



  return kSUCCESS;
}

/*
void
ATMergeTask::SetParContainers()
{


FairRun *run = FairRun::Instance();
if (!run)
fLogger -> Fatal(MESSAGE_ORIGIN, "No analysis run!");

FairRuntimeDb *db = run -> GetRuntimeDb();
if (!db)
fLogger -> Fatal(MESSAGE_ORIGIN, "No runtime database!");

fPar = (ATDigiPar *) db -> getContainer("ATDigiPar");
if (!fPar)
fLogger -> Fatal(MESSAGE_ORIGIN, "ATDigiPar not found!!");
}
*/
void
ATMergeTask::Exec(Option_t *opt)
{
  //if ( fS800CalcBr == NULL ) std::cout<<"fS800CalcBr NULL"<<std::endl;

  fS800CalcBr -> Clear();

  if (fRawEventArray -> GetEntriesFast() == 0) return;

  ATRawEvent *rawEvent = (ATRawEvent *) fRawEventArray -> At(0);
  Long64_t ATTPCTs = rawEvent -> GetTimestamp();
  int minj, maxj;
  Double_t S800EvtMatch=-1;
  minj=(int)fOptiFit->Eval(ATTPCTs)-fEvtDelta;//define the ATTPC entries range where the matching timestamp should be, to not loop over all the ATTPC entries.
  maxj=(int)fOptiFit->Eval(ATTPCTs)+fEvtDelta;

  for(int i=minj;i<maxj;i++)
  {
    if(i>0 && i<fTsEvtS800Size){
      if(isInGlom(fS800Ts.at(i-1),fS800Ts.at(i)) )std::cout<<" -- Warning -- Timestamp of consecutive entries from S800 root file within the ticks window"<<std::endl;
      else{
        //Is there a way to check that with the AT-TPC "event by event" processing?
        /*if(isInGlom(TsEvtATTPC.at(i-1),TsEvtATTPC.at(i)) )
        {
        cout<<" -- Warning -- Timestamp of consecutive entries from ATTPC root file within the glom"<<endl;
      }
      else*/ if(isInGlom(fS800Ts.at(i),ATTPCTs) ){
      S800EvtMatch = (int)fS800Evt.at(i);
      //std::cout<<" in glom "<<minj<<" "<<maxj<<" "<<i<<" "<<fS800Ts.at(i)<<" "<<ATTPCTs<<" "<<S800EvtMatch<<std::endl;
      break;
    }
    else
    S800EvtMatch = -1;
    //std::cout<<" NOT in glom "<<minj<<" "<<maxj<<" "<<i<<" "<<fS800Ts.at(i)<<" "<<ATTPCTs<<" "<<S800EvtMatch<<std::endl;
  }
}
}


if(S800EvtMatch>0) {
  TTreeReader reader2("caltree", fS800file);
  TTreeReaderValue<S800Calc> *readerValueS800Calc;
  readerValueS800Calc = new TTreeReaderValue<S800Calc>(reader2, "s800calc");

  reader2.SetEntry(S800EvtMatch);

  *fS800CalcBr = (S800Calc) *readerValueS800Calc->Get();

  fS800CalcBr->SetIsInCut(isInPID(fS800CalcBr));

  rawEvent->SetIsExtGate(isInPID(fS800CalcBr));



}


}
