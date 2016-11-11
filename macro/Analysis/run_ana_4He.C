#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TH2.h"
#include "TH1.h"
#include "TH3.h"
#include "TClonesArray.h"
#include "TCanvas.h"
#include "TMath.h"
#include "TClonesArray.h"
#include "TStyle.h"
#include "TTreeReader.h"
#include "TChain.h"
#include "TFileCollection.h"
#include "TError.h"
#include "TMinuit.h"

//#include "../../include/ATEvent.hh"
//#include "../../include/ATHit.hh"
//#include "../../include/ATProtoEvent.hh"
//#include "../../include/ATTrack.hh"
//#include "../../include/ATTrackingEventAna.hh"


void run_ana_4He(TString FileNameHead = "run_",
Int_t num_ev=100000000, Int_t file_ini=100, Int_t file_end=100, TString file="../Kinematics/Decay_kinematics/Kine.txt")
{

  TH1F* VertexH = new TH1F("VertexH","VertexH",500,-100,5000);
  TH1F* AngleH = new TH1F("AngleH","AngleH",200,0,4.0);
  TH1F* AngleSum = new TH1F("AngleSum","AngleSum",200,0,4.0);



  TH2D* Range_vs_AngleH = new TH2D("Range_vs_AngleH","Range_vs_AngleH",200,0,4.0,500,0,1000);
  Range_vs_AngleH->SetMarkerColor(2);
  Range_vs_AngleH->SetMarkerStyle(20);
  Range_vs_AngleH->SetMarkerSize(0.7);

  TH2D* VertexH_vs_AngleH = new TH2D("VertexH_vs_AngleH","VertexH_vs_AngleH",200,0,4.0,500,0,1000);
  VertexH_vs_AngleH->SetMarkerColor(2);
  VertexH_vs_AngleH->SetMarkerStyle(20);
  VertexH_vs_AngleH->SetMarkerSize(0.7);

  FairRunAna* run = new FairRunAna();

  Int_t value;
  Int_t nEve=0;

  TChain *chain = new TChain("cbmsim");

  TString FileNameHead_num;
  TString FileNameHead_chain;
  TString FilePath = "~/data/4He/Analysis/";

  for(Int_t i=file_ini;i<=file_end;i++){
    if(i<10) FileNameHead_num.Form("_000%i",i);
    else if(i<100) FileNameHead_num.Form("_00%i",i);
    else if(i<1000) FileNameHead_num.Form("_0%i",i);
    FileNameHead_chain = FilePath+"run"+FileNameHead_num+".root";
    std::cout<<" File : "<<FileNameHead_chain<<" added"<<std::endl;

    TFile* file = new TFile(FileNameHead_chain.Data(),"READ");

    if(file->IsOpen()){

      TTree* tree = (TTree*) file -> Get("cbmsim");
      Int_t nEvents = tree -> GetEntriesFast();
      std::cout<<" Number of events : "<<nEvents<<std::endl;
      TTreeReader Reader1("cbmsim", file);

      TTreeReaderValue<TClonesArray> trackingArray(Reader1, "ATTrackingEventAna");

      ATTrackingEventAna* fTrackingEvent;

      while (Reader1.Next() && nEve<num_ev) {
        if(nEve%1000==0) std::cout<<" Event number : "<<nEve<<std::endl;
        nEve++;
        fTrackingEvent  = dynamic_cast<ATTrackingEventAna*> (trackingArray->At(0));
        std::vector<ATTrack> tracks = fTrackingEvent->GetTrackArray();

          if(tracks.size()==2)
          {
                ATTrack track_r = tracks.at(0);
                ATTrack track_s = tracks.at(1);

                if( (track_r.GetQuadrant()==0 && track_s.GetQuadrant()==2) || (track_r.GetQuadrant()==2 && track_s.GetQuadrant()==0)){
                  VertexH->Fill(fTrackingEvent->GetVertex());
                  AngleH->Fill(track_r.GetGeoTheta());
                  AngleH->Fill(track_s.GetGeoTheta());
                  AngleSum->Fill(track_s.GetGeoTheta()+track_r.GetGeoTheta());
                  Range_vs_AngleH->Fill(track_r.GetGeoTheta(),track_r.GetLinearRange());
                  Range_vs_AngleH->Fill(track_s.GetGeoTheta(),track_s.GetLinearRange());
                  VertexH_vs_AngleH->Fill(track_r.GetGeoTheta(),fTrackingEvent->GetVertex());
                  VertexH_vs_AngleH->Fill(track_s.GetGeoTheta(),fTrackingEvent->GetVertex());
                }else if( (track_r.GetQuadrant()==1 && track_s.GetQuadrant()==3) || (track_r.GetQuadrant()==3 && track_s.GetQuadrant()==1)){
                  VertexH->Fill(fTrackingEvent->GetVertex());
                  AngleH->Fill(track_r.GetGeoTheta());
                  AngleH->Fill(track_s.GetGeoTheta());
                  AngleSum->Fill(track_s.GetGeoTheta()+track_r.GetGeoTheta());
                  Range_vs_AngleH->Fill(track_r.GetGeoTheta(),track_r.GetLinearRange());
                  Range_vs_AngleH->Fill(track_s.GetGeoTheta(),track_s.GetLinearRange());
                  VertexH_vs_AngleH->Fill(track_r.GetGeoTheta(),fTrackingEvent->GetVertex());
                  VertexH_vs_AngleH->Fill(track_s.GetGeoTheta(),fTrackingEvent->GetVertex());
                }


          }



      }// Tree loop



    }// Open file if


  }// for files


  TCanvas *c1 = new TCanvas("c1","c1",200,10,700,700);
  c1->Divide(2,2);

  c1->cd(1);
  VertexH->Draw();
  c1->cd(2);
  AngleH->Draw();
  c1->cd(3);
  Range_vs_AngleH->Draw();
  c1->cd(4);
  AngleSum->Draw();

  TCanvas *c2 = new TCanvas("c2","c2",200,10,700,700);
  VertexH_vs_AngleH->Draw();


}
