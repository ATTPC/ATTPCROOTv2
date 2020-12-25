/*
Merge the S800 calibrated (root) file with AT-TPC (root) file containing the ransac fitted tracks.
The "glom" needs to be set with a measured/estimated value before experiment.
In the main function (S800_ATTPC_merger()) you may want to change :
- the files paths
- S800class (0: raw data, 1: calibrated data)
- ATTPCclass (2: hits pattern, 3: ransac)
ex: S800class=1; ATTPCclass=3; will do the merging of the S800 calibrated data with the ransac data

To do:
--- Check the default value for the timestamp with real data. Update the TsEvt() and IsInGlom() functions if necessary.
Might be necessary to adjust the fit that optimize the double loop, could be biased by the default value.
--- It might not be necessary to copy the entire ATEventH and S800 branch.
Thus it could be nice to have a quick way to select the desired variables to be copied in each branch.
*/

#include <TROOT.h>
#include <TFile.h>
#include <TTree.h>
#include <TTreeReader.h>
#include <TTreeReaderValue.h>
#include <TTreeReaderArray.h>
#include "TGraph.h"
#include <TLeaf.h>
#include <TObject.h>
#include <TMath.h>
#include <Riostream.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <typeinfo>
#include <iomanip>

double glom=2.;//in unit of clock ticks (glom of 1us is 10 units : 10*100ns), case by case not a constant for every experiment
int TsEvtS800Size=0;
int TsEvtATTPCSize=0;


bool isInGlom(Long64_t ts1, Long64_t ts2)
{
  bool is=false;
  if(ts1>0 && ts2>0 && fabs(ts1-ts2)<glom) is=true;
  return is;
}

//fills the timestamps vectors for S800 and ATTPC
void TsEvt(TString str, const Int_t src, vector <Long64_t> &ts, vector <Double_t> &ievt)//src 0 || 1 : S800; src 2 || 3 : ATTPC
{
  TFile *file = new TFile(str);
  TString S800strReader[2]={"rawtree","caltree"};
  TString S800strTs[2]={"fS800.fts","fts"};
  if(src==0||src==1){
    TTreeReader reader(S800strReader[src], file);
    TTreeReaderValue<Long64_t> fts(reader, S800strTs[src]);
    while (reader.Next()) {
      ts.push_back(*fts);
      ievt.push_back(TsEvtS800Size);
      TsEvtS800Size++;
      //if(TsEvtS800Size<110) std::cout<<"s800 "<<*fts<<std::endl;
    }
  }
  else if(src==2 || src==3){
    TTreeReader reader("cbmsim", file);
    TTreeReaderArray<ULong_t> fts(reader, "ATEventH.fTimestamp");
    while (reader.Next()) {
      if(fts.GetSize()>0){
        ts.push_back((Long64_t)fts.At(0));
        ievt.push_back(TsEvtATTPCSize);
        TsEvtATTPCSize++;
        //if(TsEvtATTPCSize<110) std::cout<<"attpc "<<fts.At(0)<<std::endl;
      }
      else ts.push_back(-1);
    }
  }
  delete file;
}

//retrun a vector with the ATTPC entries that match by timestamp the S800 entries
vector <int> EvtMatch(const vector<Long64_t> &TsEvtS800, const vector<Long64_t> &TsEvtATTPC, TF1 *fit)
{
  float perc=0.;
  int minj, maxj, delj=100;
  vector <int> match;//the index correspond to the S800 root entry number and the content is the ATTPC root entry number matching by TS
  for(int i=0;i<TsEvtS800Size;i++)
  {
    if(i>0 && isInGlom(TsEvtS800.at(i-1),TsEvtS800.at(i)) )cout<<" -- Warning -- Timestamp of consecutive entries from S800 root file within the glom"<<endl;
    else {
      match.push_back(-1);
      minj=fit->Eval(TsEvtS800.at(i))-delj;//define the ATTPC entries range where the matching timestamp should be, to not loop over all the ATTPC entries.
      maxj=fit->Eval(TsEvtS800.at(i))+delj;
      for(int j=minj;j<maxj;j++)
      {
        //cout<<" all "<<i<<" "<<j<<" "<<TsEvtS800.at(i)<<" "<<TsEvtATTPC.at(j)<<endl;
        if(j>0 && j<TsEvtATTPCSize && isInGlom(TsEvtATTPC.at(j-1),TsEvtATTPC.at(j)) )
        {
          cout<<" -- Warning -- Timestamp of consecutive entries from ATTPC root file within the glom"<<endl;
        }
        else if(j>0 && j<TsEvtATTPCSize && isInGlom(TsEvtS800.at(i),TsEvtATTPC.at(j)) )
        {
          match.at(i)=j;
          //cout<<" in glom "<<minj<<" "<<maxj<<" "<<i<<" "<<j<<" "<<TsEvtS800.at(i)<<" "<<TsEvtATTPC.at(j)<<" "<<match.at(i)<<endl;
          continue;
        }
      }
    }
    if(int((double(i)/double(TsEvtS800Size)*100.))>perc){
      perc=int(double(i)/double(TsEvtS800Size)*100.);
      printf("\rEvtMatch\033[1;32m %i %%\033[0m", int(perc));
      fflush(stdout);}
    }
    return match;
  }


  void MergeTrees(TString strS800, TString strATTPC, const vector<int> &match, const Int_t S800class, const Int_t ATTPCclass)
  {
    //Get old files, old trees and set top branch address
    TFile *newFile = new TFile("merged.root","recreate");
    int EvtS800=0;
    TFile *fileS800 = new TFile(strS800);
    TFile *fileATTPC = new TFile(strATTPC);

    TString S800strReader[2]={"rawtree","caltree"};
    TTreeReader readerS800(S800strReader[S800class], fileS800);
    TTreeReaderValue<S800Event> *readerValueS800Evt;
    TTreeReaderValue<S800Calc> *readerValueS800Calc;
    if(S800class==0) readerValueS800Evt = new TTreeReaderValue<S800Event>(readerS800, "s800event");
    else if(S800class==1) readerValueS800Calc = new TTreeReaderValue<S800Calc>(readerS800, "s800calc");
    /*TTreeReaderValue<S800Calc> readerValueS800Calc(readerS800, "s800calc");*/

    TTreeReader readerATTPC("cbmsim", fileATTPC);
    TString ATstr[2]={"ATEventH","ATRansac"};
    TTreeReaderValue<TClonesArray> readerValueATTPC(readerATTPC, ATstr[ATTPCclass-2]);

    newFile->cd();
    TTree *newTree = new TTree("tree","merged tree");

    S800 *fS800   = new S800();
    S800Event *fS800Event   = new S800Event();
    S800Calc *fS800Calc = new S800Calc();
    if(S800class==0) newTree->Branch("s800", &fS800);
    else if(S800class==1) newTree->Branch("s800", &fS800Calc);

    ATEvent* eventATTPC0 = new ATEvent();
    ATRANSACN::ATRansac* eventATTPC1  = new ATRANSACN::ATRansac();
    if(ATTPCclass==2) newTree->Branch("attpc", &eventATTPC0);
    else if(ATTPCclass==3) newTree->Branch("attpc", &eventATTPC1);

    while (readerS800.Next()) {
      if(match.at(EvtS800)>0) {
        if(S800class==0) {
          *fS800Event = (S800Event) *readerValueS800Evt->Get();
          fS800 = (S800*) fS800Event->GetS800();
        }
        else if(S800class==1) *fS800Calc = (S800Calc) *readerValueS800Calc->Get();

        readerATTPC.SetEntry(match.at(EvtS800));//event AT-TPC that matches with EvtS800

        if(ATTPCclass==2) eventATTPC0 = (ATEvent*) readerValueATTPC->At(0);
        else if(ATTPCclass==3) eventATTPC1 = dynamic_cast<ATRANSACN::ATRansac*> (readerValueATTPC->At(0));

        newTree->Fill();
      }
      EvtS800++;
    }
    newTree->Write();
    //newTree->Print();
    newFile->Close();

    delete fileS800;
    delete fileATTPC;

    return 0;

  }


  int S800_ATTPC_merger() {
    TString fileS800_1="/home/juan/FairRoot/ATTPCROOTv2_simon/test-runs800-48Ca-RAW.root";
    TString fileS800_2="/home/juan/FairRoot/ATTPCROOTv2_simon/test-runs800-48Ca-CAL.root";
    TString fileATTPC="/home/juan/FairRoot/ATTPCROOTv2_simon/attpcdigi_d2He_1000.root";//projects/ceclub/giraud/ATTPCROOTv2/macro/Unpack_HDF5/
    Int_t S800class, ATTPCclass;
    S800class=1;//0 : rawtree; 1 : caltree
    ATTPCclass=3;//2 : raw hit pattern; 3: ransac fitted hit pattern

    TString FILEstr[4]={fileS800_1,fileS800_2,fileATTPC,fileATTPC}; //maybe too complicated for nothing, could be better to define const strings at the beginning

    vector <Double_t> ievt_ATTPC;
    vector <Long64_t> ATTPCTs;
    vector <Double_t> ievt_S800;
    vector <Long64_t> S800Ts;

    cout<<"Start ---"<<endl;

    TsEvt(FILEstr[S800class],S800class,S800Ts,ievt_S800);
    cout<<"got the S800 timestamps"<<endl;

    TsEvt(FILEstr[ATTPCclass],ATTPCclass,ATTPCTs,ievt_ATTPC);
    cout<<"got the ATTPC timestamps"<<endl;

    /* fit of the TS vs entry number, this should be monotonic increasing,
    the function is used then to reduce the size of the ATTPC loop in EvTMatch, i.e. save time
    */
    //I think TS has to be converted into double for TGraph, but would prefer to not have this extra step
    vector <Double_t> ATTPC_ts(ATTPCTs.begin(),ATTPCTs.end());
    vector <Double_t> S800_ts(S800Ts.begin(),S800Ts.end());
    Double_t par_fit[2];
    TF1 *f1 = new TF1("f1","[1]*x + [0]",0,1000);//poly 1 seems relatively ok, fit do not need to be very precise
    //TF1 *f1 = new TF1("f1","[2]*x*x + [1]*x + [0]",0,1000);
    TGraph *gATTPC = new TGraph(ATTPCTs.size(), &ATTPC_ts[0], &ievt_ATTPC[0]);
    TGraph *gS800 = new TGraph(ATTPCTs.size(), &S800_ts[0], &ievt_S800[0]);
    gATTPC->Fit("f1");//might be biased by the TS default value
    f1->GetParameters(&par_fit[0]);
    f1->SetParameters(par_fit[0],par_fit[1]);
    gS800->Draw("AL");
    gATTPC->Draw("same");
    //gS800->Draw("same");
    f1->Draw("same");
    cout<<"\n";
    vector <int> match=EvtMatch(S800Ts,ATTPCTs,f1);
    cout<<"\ndid the event matching"<<endl;

    //for(int i=0;i<20;i++)cout<<"match "<< i <<" "<<match.at(i)<<endl;

    MergeTrees(FILEstr[S800class],FILEstr[ATTPCclass],match,S800class,ATTPCclass);
    cout<<"Trees merged --- End"<<endl;

    return 0;
  }
