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

  //fS800FileType = 0;
  fEvtDelta=100;
  fGlom=2;


  //  fS800Br   = new TClonesArray("S800");
  fS800CalcBr = new TClonesArray("S800Calc");

}

ATMergeTask::~ATMergeTask()
{
}

void   ATMergeTask::SetPersistence(Bool_t value)                  { fIsPersistence     = value; }
//void   ATMergeTask::SetS800FileType(Int_t type)             { fS800FileType  = type; }
void   ATMergeTask::SetS800File(TString file)                  { fS800File     = file; }
void   ATMergeTask::SetGlom(Double_t glom)                  { fGlom     = glom; }
void   ATMergeTask::SetOptiEvtDelta(Int_t EvtDelta)                  { fEvtDelta     = EvtDelta; }
void   ATMergeTask::SetPIDcut(TString file)                  { fcutPIDFile     = file; }



Bool_t ATMergeTask::isInGlom(Long64_t ts1, Long64_t ts2)
{
  bool is=false;
  //std::cout<<"glom "<<ts1<<" "<<ts2<<" "<<fabs(ts1-ts2)<<std::endl;
  if(ts1>0 && ts2>0 && fabs(ts1-ts2)<fGlom) is=true;
  //std::cout<<"glom "<<ts1<<" "<<ts2<<" "<<fabs(ts1-ts2)<<" "<<is<<" "<<fGlom<<std::endl;
  return is;
}

Bool_t ATMergeTask::isInPID(S800Calc *s800calc)
{
  Double_t S800_rf = s800calc->GetMultiHitTOF()->GetFirstRfHit();//might need to change the ToF
  Double_t S800_dE = s800calc->GetSCINT(0)->GetDE();//check if is this scint (0)
  Bool_t is=false;

  if(fcutPID->IsInside(S800_rf,S800_dE)){
    std::cout<<"in cut PID"<<std::endl;
    is=true;
  }
  return is;
}



InitStatus
ATMergeTask::Init()
{

  /*
  if(fS800FileType==0 || fS800FileType==1);
  else{
  fLogger -> Error(MESSAGE_ORIGIN, "Cannot find S800 class type!");
  return kERROR;
}
*/
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
/*
fS800strReader[0]="rawtree"; fS800strReader[1]="caltree";
fS800strTs[0]="fS800.fts"; fS800strTs[1]="fts";
*/
TTreeReader reader("caltree", fS800file);
TTreeReaderValue<Long64_t> ts(reader,"fts");
/*
TTreeReader reader(fS800strReader[fS800FileType], fS800file);
TTreeReaderValue<Long64_t> ts(reader, fS800strTs[fS800FileType]);
*/
while (reader.Next()) {
//for (int j=0; j<40; j++) {
//  reader.Next();
  fS800Ts.push_back((Long64_t) *ts);
  fS800Evt.push_back((Double_t) fTsEvtS800Size);
  //if(fTsEvtS800Size<100) std::cout<<"Ts S800 "<<fS800Ts.at(fTsEvtS800Size)<<std::endl;
  fTsEvtS800Size++;
}
ioMan -> Register("s800cal", "s800", fS800CalcBr, fIsPersistence);
/*
if(fS800FileType==0) ioMan -> Register("s800raw", "s800", fS800Br, fIsPersistence);
else if(fS800FileType==1) ioMan -> Register("s800cal", "s800", fS800CalcBr, fIsPersistence);
*/

std::cout << "  File :  " << fS800File<< " Events : " << fTsEvtS800Size <<"  " <<std::endl;
//delete fS800file;


/* fit of the TS vs entry number, this should be monotonic increasing,
the function is used then to reduce the size of the ATTPC loop in EvTMatch, i.e. save time
*/
//I think TS has to be converted into double for TGraph, but would prefer to not have this extra step
auto c1 = new TCanvas("c1", "c1", 800, 800);
vector <Double_t> S800_ts(fS800Ts.begin(),fS800Ts.end());
Double_t par_fit[2];
fOptiFit = new TF1("fOptiFit","[1]*x + [0]",0,7E+9);//poly 1 seems relatively ok, fit do not need to be very precise
//fOptiFit = new TF1("fOptiFit","[1]*x + [0]",0,1E+13);//poly 1 seems relatively ok, fit do not need to be very precise
//TF1 *f1 = new TF1("f1","[2]*x*x + [1]*x + [0]",0,1000);
TGraph *gS800 = new TGraph(80, &S800_ts[0], &fS800Evt[0]);//fTsEvtS800Size
gS800->Fit("fOptiFit");//might be biased by the TS default value
fOptiFit->GetParameters(&par_fit[0]);
fOptiFit->SetParameters(par_fit[0],par_fit[1]);
c1->cd();
gS800->Draw("AL");
//f1->Draw("same");

auto c2 = new TCanvas("c2", "c2", 800, 800);
c2->cd();
gROOT->ProcessLine(".x "+fcutPIDFile);//
fcutPID = (TCutG*)gROOT->GetListOfSpecials()->FindObject("CUTG");
fcutPID->SetName("fcutPID");


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

  //  fS800Br -> Delete();
  fS800CalcBr -> Delete();

  if (fRawEventArray -> GetEntriesFast() == 0)
  return;

  ATRawEvent *rawEvent = (ATRawEvent *) fRawEventArray -> At(0);
  Long64_t ATTPCTs = rawEvent -> GetTimestamp();
  //std::cout << "  Event Number :  " << rawEvent -> GetEventID() << " TimeStamp : " << rawEvent -> GetTimestamp() <<" TS eval min "<<fOptiFit->Eval(ATTPCTs)-fEvtDelta
  //<<" TS eval max "<<fOptiFit->Eval(ATTPCTs)+fEvtDelta<<std::endl;

  int minj, maxj;
  Double_t S800EvtMatch=-1;
  //vector <int> match;//the index correspond to the S800 root entry number and the content is the ATTPC root entry number matching by TS
  minj=(int)fOptiFit->Eval(ATTPCTs)-fEvtDelta;//define the ATTPC entries range where the matching timestamp should be, to not loop over all the ATTPC entries.
  maxj=(int)fOptiFit->Eval(ATTPCTs)+fEvtDelta;
  for(int i=minj;i<maxj;i++)
  {
    if(i>0 && i<fTsEvtS800Size){
    if(isInGlom(fS800Ts.at(i-1),fS800Ts.at(i)) )std::cout<<" -- Warning -- Timestamp of consecutive entries from S800 root file within the glom"<<std::endl;
    else{
      //Is there a way to check that with the AT-TPC "event by event" processing?
      /*if(isInGlom(TsEvtATTPC.at(i-1),TsEvtATTPC.at(i)) )
      {
      cout<<" -- Warning -- Timestamp of consecutive entries from ATTPC root file within the glom"<<endl;
    }
    else*/ if(isInGlom(fS800Ts.at(i),ATTPCTs) )
    {
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



fS800file = new TFile(fS800File);

/*
fS800strReader[0]="rawtree"; fS800strReader[1]="caltree";
fS800strTs[0]="fS800.fts"; fS800strTs[1]="fts";

TTreeReader reader(fS800strReader[fS800FileType], fS800file);
TTreeReaderValue<S800Event> *readerValueS800Evt;
TTreeReaderValue<S800Calc> *readerValueS800Calc;
if(fS800FileType==0) {
readerValueS800Evt = new TTreeReaderValue<S800Event>(reader, "s800event");
reader.SetEntry(10);//S800EvtMatch
}
else if(fS800FileType==1) {
readerValueS800Calc = new TTreeReaderValue<S800Calc>(reader, "s800calc");
reader.SetEntry(10);//S800EvtMatch
}
*/

TTreeReader reader("caltree", fS800file);
TTreeReaderValue<S800Calc> *readerValueS800Calc;
readerValueS800Calc = new TTreeReaderValue<S800Calc>(reader, "s800calc");

if(S800EvtMatch<0)return;

reader.SetEntry(S800EvtMatch);//S800EvtMatch

/*
S800 *S800Evt = (S800 *) new ((*fS800Br)[0]) S800();
S800Event *fS800Event   = new S800Event();
*/
S800Calc *S800CalcEvt = (S800Calc *) new ((*fS800CalcBr)[0]) S800Calc();

/*
if(fS800FileType==0) {
*fS800Event = (S800Event) *readerValueS800Evt->Get();
S800Evt = (S800*) fS800Event->GetS800();
}
else if(fS800FileType==1) *S800CalcEvt = (S800Calc) *readerValueS800Calc->Get();
*/
*S800CalcEvt = (S800Calc) *readerValueS800Calc->Get();

Bool_t condition_1;
condition_1 = isInPID(S800CalcEvt);
if(!condition_1) return;



}
