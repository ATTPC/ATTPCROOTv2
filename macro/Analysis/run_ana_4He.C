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

  TH1F* AngleSumMC = new TH1F("AngleSumMC","AngleSumMC",200,0,4.0);
  TH1F* Chi2MC = new TH1F("Chi2MC","Chi2MC",100,0,0.1);

  TH2F* RangevsEner = new TH2F("RangevsEner","RangevsEner",100,0,10,500,0,1000);
  RangevsEner->SetMarkerColor(2);
  RangevsEner->SetMarkerStyle(20);
  RangevsEner->SetMarkerSize(0.7);

  TH2F* VertexEvsEnersum = new TH2F("VertexEvsEnersum","VertexEvsEnersum",400,0,20,400,0,20);
  VertexEvsEnersum->SetMarkerColor(2);
  VertexEvsEnersum->SetMarkerStyle(20);
  VertexEvsEnersum->SetMarkerSize(0.7);

  TH2F* VertexEvsEnersumMC = new TH2F("VertexEvsEnersumMC","VertexEvsEnersumMC",400,0,20,400,0,20);
  VertexEvsEnersumMC->SetMarkerColor(2);
  VertexEvsEnersumMC->SetMarkerStyle(20);
  VertexEvsEnersumMC->SetMarkerSize(0.7);

  TH2F* AnglevsEnersum = new TH2F("AngleEvsEnersum","AngleEvsEnersum",400,0,4.0,400,0,40);
  AnglevsEnersum->SetMarkerColor(2);
  AnglevsEnersum->SetMarkerStyle(20);
  AnglevsEnersum->SetMarkerSize(0.7);

  TH2F* AnglevsEnersumMC = new TH2F("AngleEvsEnersumMC","AngleEvsEnersumMC",400,0,4.0,400,0,40);
  AnglevsEnersumMC->SetMarkerColor(2);
  AnglevsEnersumMC->SetMarkerStyle(20);
  AnglevsEnersumMC->SetMarkerSize(0.7);

  TH2F* VertexXvsEnersum = new TH2F("VertexXvsEnersum","VertexXvsEnersum",500,0,1000,400,0,40);
  VertexXvsEnersum->SetMarkerColor(2);
  VertexXvsEnersum->SetMarkerStyle(20);
  VertexXvsEnersum->SetMarkerSize(0.7);

  TH2D* Range_vs_AngleH = new TH2D("Range_vs_AngleH","Range_vs_AngleH",400,0,4.0,500,0,1000);
  Range_vs_AngleH->SetMarkerColor(2);
  Range_vs_AngleH->SetMarkerStyle(20);
  Range_vs_AngleH->SetMarkerSize(0.7);

  TH2D* VertexH_vs_AngleH = new TH2D("VertexH_vs_AngleH","VertexH_vs_AngleH",400,0,4.0,500,0,1000);
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
                  AngleSumMC->Fill(track_s.FitParameters.sThetaMin+track_r.FitParameters.sThetaMin);
                  Range_vs_AngleH->Fill(track_r.GetGeoTheta(),track_r.GetLinearRange());
                  Range_vs_AngleH->Fill(track_s.GetGeoTheta(),track_s.GetLinearRange());
                  RangevsEner->Fill(track_r.GetGeoEnergy(),track_r.GetLinearRange());
                  RangevsEner->Fill(track_s.GetGeoEnergy(),track_s.GetLinearRange());
                  VertexEvsEnersum->Fill(track_r.GetGeoEnergy()+track_s.GetGeoEnergy(),fTrackingEvent->GetVertexEnergy());
                  AnglevsEnersum->Fill(track_r.GetGeoTheta(),track_r.GetGeoEnergy()/track_s.GetGeoEnergy());
                  AnglevsEnersumMC->Fill(track_r.FitParameters.sThetaMin,track_r.FitParameters.sEnerMin/track_s.FitParameters.sEnerMin);
                  VertexH_vs_AngleH->Fill(track_r.GetGeoTheta(),fTrackingEvent->GetVertex());
                  VertexH_vs_AngleH->Fill(track_s.GetGeoTheta(),fTrackingEvent->GetVertex());
                  VertexXvsEnersum->Fill(track_r.GetGeoEnergy()+track_s.GetGeoEnergy(),fTrackingEvent->GetVertex());
                  Chi2MC->Fill(track_s.FitParameters.sChi2Min);
                  VertexEvsEnersumMC->Fill(track_r.FitParameters.sEnerMin+track_s.FitParameters.sEnerMin,track_r.FitParameters.sVertexEner);

                }else if( (track_r.GetQuadrant()==1 && track_s.GetQuadrant()==3) || (track_r.GetQuadrant()==3 && track_s.GetQuadrant()==1)){

                  VertexH->Fill(fTrackingEvent->GetVertex());
                  AngleH->Fill(track_r.GetGeoTheta());
                  AngleH->Fill(track_s.GetGeoTheta());
                  AngleSum->Fill(track_s.GetGeoTheta()+track_r.GetGeoTheta());
                  AngleSumMC->Fill(track_s.FitParameters.sThetaMin+track_r.FitParameters.sThetaMin);
                  Range_vs_AngleH->Fill(track_r.GetGeoTheta(),track_r.GetLinearRange());
                  Range_vs_AngleH->Fill(track_s.GetGeoTheta(),track_s.GetLinearRange());
                  RangevsEner->Fill(track_r.GetGeoEnergy(),track_r.GetLinearRange());
                  RangevsEner->Fill(track_s.GetGeoEnergy(),track_s.GetLinearRange());
                  VertexEvsEnersum->Fill(track_r.GetGeoEnergy()+track_s.GetGeoEnergy(),fTrackingEvent->GetVertexEnergy());
                  AnglevsEnersum->Fill(track_r.GetGeoTheta(),track_r.GetGeoEnergy()/track_s.GetGeoEnergy());
                  AnglevsEnersumMC->Fill(track_r.FitParameters.sThetaMin,track_r.FitParameters.sEnerMin/track_s.FitParameters.sEnerMin);
                  VertexH_vs_AngleH->Fill(track_r.GetGeoTheta(),fTrackingEvent->GetVertex());
                  VertexH_vs_AngleH->Fill(track_s.GetGeoTheta(),fTrackingEvent->GetVertex());
                  VertexXvsEnersum->Fill(track_r.GetGeoEnergy()+track_s.GetGeoEnergy(),fTrackingEvent->GetVertex());
                  Chi2MC->Fill(track_s.FitParameters.sChi2Min);
                  VertexEvsEnersumMC->Fill(track_r.FitParameters.sEnerMin+track_s.FitParameters.sEnerMin,track_r.FitParameters.sVertexEner);

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

  TCanvas *c3 = new TCanvas("c3","c3",200,10,700,700);
  c3->Divide(2,2);
  c3->cd(1);
  RangevsEner->Draw();
  c3->cd(2);
  VertexEvsEnersum->Draw();
  c3->cd(3);
  AnglevsEnersum->Draw();
  c3->cd(4);
  VertexXvsEnersum->Draw();

  TCanvas *cMC = new TCanvas("cMC","cMC",200,10,700,700);
  cMC->Divide(2,2);
  cMC->cd(1);
  AngleSumMC->Draw();
  cMC->cd(2);
  Chi2MC->Draw();
  cMC->cd(3);
  AnglevsEnersumMC->Draw();
  cMC->cd(4);
  VertexEvsEnersumMC->Draw();



}
