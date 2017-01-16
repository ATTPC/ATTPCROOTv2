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


void run_ana_4He_old(TString FileNameHead = "run_",
Int_t num_ev=100000000, Int_t file_ini=80, Int_t file_end=112, TString file="../Kinematics/Decay_kinematics/Kine.txt")
{

  TH1F* VertexH = new TH1F("VertexH","VertexH",500,-100,5000);
  TH1F* AngleH = new TH1F("AngleH","AngleH",1000,0,180.0);
  TH1F* AngleSum = new TH1F("AngleSum","AngleSum",1000,0,180.0);

  TH1F* AngleSumMC = new TH1F("AngleSumMC","AngleSumMC",1000,0,180.0);
  TH1F* Chi2MC = new TH1F("Chi2MC","Chi2MC",100,0,0.1);

  TH2F* RangevsEner = new TH2F("RangevsEner","RangevsEner",500,0,1000,100,0,15);
  RangevsEner->SetMarkerColor(2);
  RangevsEner->SetMarkerStyle(20);
  RangevsEner->SetMarkerSize(0.7);

  TH2F* E1vsE2 = new TH2F("E1vsE2","E1vsE2",400,0,20,400,0,20);
  E1vsE2->SetMarkerColor(2);
  E1vsE2->SetMarkerStyle(20);
  E1vsE2->SetMarkerSize(0.7);

  TH2F* VertexEvsEnersum = new TH2F("VertexEvsEnersum","VertexEvsEnersum",400,0,20,400,0,20);
  VertexEvsEnersum->SetMarkerColor(2);
  VertexEvsEnersum->SetMarkerStyle(20);
  VertexEvsEnersum->SetMarkerSize(0.7);

  TH2F* VertexEvsEnersumMC = new TH2F("VertexEvsEnersumMC","VertexEvsEnersumMC",400,0,20,400,0,20);
  VertexEvsEnersumMC->SetMarkerColor(2);
  VertexEvsEnersumMC->SetMarkerStyle(20);
  VertexEvsEnersumMC->SetMarkerSize(0.7);

  TH2F* AnglevsEnersum = new TH2F("AngleEvsEnersum","AngleEvsEnersum",1000,0,180.0,400,0,40);
  AnglevsEnersum->SetMarkerColor(2);
  AnglevsEnersum->SetMarkerStyle(20);
  AnglevsEnersum->SetMarkerSize(0.7);

  TH2F* AnglevsEnersumMC = new TH2F("AngleEvsEnersumMC","AngleEvsEnersumMC",1000,0,180.0,400,0,40);
  AnglevsEnersumMC->SetMarkerColor(2);
  AnglevsEnersumMC->SetMarkerStyle(20);
  AnglevsEnersumMC->SetMarkerSize(0.7);

  TH2F* VertexXvsEnersum = new TH2F("VertexXvsEnersum","VertexXvsEnersum",500,0,1000,400,0,40);
  VertexXvsEnersum->SetMarkerColor(2);
  VertexXvsEnersum->SetMarkerStyle(20);
  VertexXvsEnersum->SetMarkerSize(0.7);

  TH2D* Range_vs_AngleH = new TH2D("Range_vs_AngleH","Range_vs_AngleH",1000,0,180.0,500,0,1000);
  Range_vs_AngleH->SetMarkerColor(2);
  Range_vs_AngleH->SetMarkerStyle(20);
  Range_vs_AngleH->SetMarkerSize(0.7);

  TH2D* Range1_vs_Range2 = new TH2D("Range1_vs_Range2","Range1_vs_Range2",500,0,1000,500,0,1000);
  Range1_vs_Range2->SetMarkerColor(2);
  Range1_vs_Range2->SetMarkerStyle(20);
  Range1_vs_Range2->SetMarkerSize(0.7);

  TH2D* VertexH_vs_AngleH = new TH2D("VertexH_vs_AngleH","VertexH_vs_AngleH",1000,0,180.0,500,0,1000);
  VertexH_vs_AngleH->SetMarkerColor(2);
  VertexH_vs_AngleH->SetMarkerStyle(20);
  VertexH_vs_AngleH->SetMarkerSize(0.7);

  TH2F* AnglePhi_vs_AngleSum = new TH2F("AnglePhi_vs_AngleSum","AnglePhi_vs_AngleSum",1000,0,180.0,1000,0,180.0);
  AnglePhi_vs_AngleSum->SetMarkerColor(2);
  AnglePhi_vs_AngleSum->SetMarkerStyle(20);
  AnglePhi_vs_AngleSum->SetMarkerSize(0.7);


  FairRunAna* run = new FairRunAna();

  Int_t value;
  Int_t nEve=0;

  TChain *chain = new TChain("cbmsim");

  TString FileNameHead_num;
  TString FileNameHead_chain;
  TString FilePath = "/run/media/ayyadlim/DATA/data/Alpha/Analysis/";

  ofstream kine_file;
  kine_file.open("kinematics.txt");

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

                Double_t Eoffset = -0.0; //OFfset in MeV

                Double_t rad2deg = 180.0/TMath::Pi();

                if( (track_r.GetQuadrant()==0 && track_s.GetQuadrant()==2) || (track_r.GetQuadrant()==2 && track_s.GetQuadrant()==0)){


                  VertexH->Fill(fTrackingEvent->GetVertex());
                  AngleH->Fill(track_r.GetGeoTheta()*rad2deg);
                  AngleH->Fill(track_s.GetGeoTheta()*rad2deg);
                  AngleSum->Fill(track_s.GetGeoTheta()*rad2deg+track_r.GetGeoTheta()*rad2deg);
                  AngleSumMC->Fill(track_s.FitParameters.sThetaMin*rad2deg+track_r.FitParameters.sThetaMin*rad2deg);
                  Range_vs_AngleH->Fill(track_r.GetGeoTheta()*rad2deg,track_r.GetLinearRange(fTrackingEvent->GetGeoVertex()));
                  Range_vs_AngleH->Fill(track_s.GetGeoTheta()*rad2deg,track_s.GetLinearRange(fTrackingEvent->GetGeoVertex()));
                  RangevsEner->Fill(track_r.GetLinearRange(fTrackingEvent->GetGeoVertex()),track_r.GetGeoEnergy());
                  RangevsEner->Fill(track_s.GetLinearRange(fTrackingEvent->GetGeoVertex()),track_s.GetGeoEnergy());
                  VertexEvsEnersum->Fill(track_r.GetGeoEnergy()+track_s.GetGeoEnergy()+ Eoffset*2.0,fTrackingEvent->GetVertexEnergy());
                  AnglevsEnersum->Fill(track_r.GetGeoTheta()*rad2deg,track_r.GetGeoEnergy()/track_s.GetGeoEnergy());
                  AnglevsEnersumMC->Fill(track_r.FitParameters.sThetaMin*rad2deg,track_r.FitParameters.sEnerMin/track_s.FitParameters.sEnerMin);
                  VertexH_vs_AngleH->Fill(track_r.GetGeoTheta()*rad2deg,fTrackingEvent->GetVertex());
                  VertexH_vs_AngleH->Fill(track_s.GetGeoTheta()*rad2deg,fTrackingEvent->GetVertex());
                  VertexXvsEnersum->Fill(fTrackingEvent->GetVertex(),track_r.GetGeoEnergy()+track_s.GetGeoEnergy()+ Eoffset*2.0);
                  Chi2MC->Fill(track_s.FitParameters.sChi2Min);
                  VertexEvsEnersumMC->Fill(track_r.FitParameters.sEnerMin+track_s.FitParameters.sEnerMin+ Eoffset*2.0,track_r.FitParameters.sVertexEner);
                  Range1_vs_Range2->Fill(track_r.GetLinearRange(fTrackingEvent->GetGeoVertex()),track_s.GetLinearRange(fTrackingEvent->GetGeoVertex()));
                  E1vsE2->Fill(track_r.GetGeoEnergy()+ Eoffset,track_s.GetGeoEnergy()+ Eoffset);

                  AnglePhi_vs_AngleSum->Fill(track_s.GetGeoTheta()*rad2deg,track_s.GetGeoTheta()*rad2deg+track_r.GetGeoTheta()*rad2deg);

                  kine_file<<track_r.GetGeoTheta()*rad2deg<<" "<<track_s.GetGeoTheta()*rad2deg<<" "<<fTrackingEvent->GetVertex()<<std::endl;

                }else if( (track_r.GetQuadrant()==1 && track_s.GetQuadrant()==3) || (track_r.GetQuadrant()==3 && track_s.GetQuadrant()==1)){

                  VertexH->Fill(fTrackingEvent->GetVertex());
                  AngleH->Fill(track_r.GetGeoTheta()*rad2deg);
                  AngleH->Fill(track_s.GetGeoTheta()*rad2deg);
                  AngleSum->Fill(track_s.GetGeoTheta()*rad2deg+track_r.GetGeoTheta()*rad2deg);
                  AngleSumMC->Fill(track_s.FitParameters.sThetaMin*rad2deg+track_r.FitParameters.sThetaMin*rad2deg);
                  Range_vs_AngleH->Fill(track_r.GetGeoTheta()*rad2deg,track_r.GetLinearRange(fTrackingEvent->GetGeoVertex()));
                  Range_vs_AngleH->Fill(track_s.GetGeoTheta()*rad2deg,track_s.GetLinearRange(fTrackingEvent->GetGeoVertex()));
                  RangevsEner->Fill(track_r.GetLinearRange(fTrackingEvent->GetGeoVertex()),track_r.GetGeoEnergy());
                  RangevsEner->Fill(track_s.GetLinearRange(fTrackingEvent->GetGeoVertex()),track_s.GetGeoEnergy());
                  VertexEvsEnersum->Fill(track_r.GetGeoEnergy()+track_s.GetGeoEnergy()+ Eoffset*2.0,fTrackingEvent->GetVertexEnergy());
                  AnglevsEnersum->Fill(track_r.GetGeoTheta()*rad2deg,track_r.GetGeoEnergy()/track_s.GetGeoEnergy());
                  AnglevsEnersumMC->Fill(track_r.FitParameters.sThetaMin*rad2deg,track_r.FitParameters.sEnerMin/track_s.FitParameters.sEnerMin);
                  VertexH_vs_AngleH->Fill(track_r.GetGeoTheta()*rad2deg,fTrackingEvent->GetVertex());
                  VertexH_vs_AngleH->Fill(track_s.GetGeoTheta()*rad2deg,fTrackingEvent->GetVertex());
                  VertexXvsEnersum->Fill(fTrackingEvent->GetVertex(),track_r.GetGeoEnergy()+track_s.GetGeoEnergy()+ Eoffset*2.0);
                  Chi2MC->Fill(track_s.FitParameters.sChi2Min);
                  VertexEvsEnersumMC->Fill(track_r.FitParameters.sEnerMin+track_s.FitParameters.sEnerMin+ Eoffset*2.0,track_r.FitParameters.sVertexEner);
                  Range1_vs_Range2->Fill(track_r.GetLinearRange(fTrackingEvent->GetGeoVertex()),track_s.GetLinearRange(fTrackingEvent->GetGeoVertex()));
                  E1vsE2->Fill(track_r.GetGeoEnergy()+ Eoffset,track_s.GetGeoEnergy()+ Eoffset);

                  AnglePhi_vs_AngleSum->Fill(track_s.GetGeoTheta()*rad2deg,track_s.GetGeoTheta()*rad2deg+track_r.GetGeoTheta()*rad2deg);

                  kine_file<<track_r.GetGeoTheta()*rad2deg<<" "<<track_s.GetGeoTheta()*rad2deg<<" "<<fTrackingEvent->GetVertex()<<std::endl;

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
  Range_vs_AngleH->Draw("zcol");
  c1->cd(4);
  AngleSum->Draw();

  TCanvas *c1p = new TCanvas("c1p","c1p",200,10,700,700);
  c1p->cd();
  VertexH->GetXaxis()->SetTitle("Reaction Vertex (mm)");
  VertexH->Draw();
  TCanvas *c2p = new TCanvas("c2p","c2p",200,10,700,700);
  c2p->cd();
  AngleH->GetXaxis()->SetTitle("Scattering Angle");
  AngleH->Draw();
  TCanvas *c3p = new TCanvas("c3p","c3p",200,10,700,700);
  c3p->cd();
  Range_vs_AngleH->GetXaxis()->SetTitle("Scattering Angle");
  Range_vs_AngleH->GetYaxis()->SetTitle("Range (mm)");
  Range_vs_AngleH->Draw("zcol");
  TCanvas *c4p = new TCanvas("c4p","c4p",200,10,700,700);
  c4p->cd();
  AngleSum->GetXaxis()->SetTitle("Sum Scattering Angle");
  AngleSum->Draw();


  TCanvas *c2 = new TCanvas("c2","c2",200,10,700,700);
  VertexH_vs_AngleH->Draw("zcol");
  VertexH_vs_AngleH->GetXaxis()->SetTitle("Scattering Angle");
  VertexH_vs_AngleH->GetYaxis()->SetTitle("Reaction Vertex (mm)");

  TCanvas *c3 = new TCanvas("c3","c3",200,10,700,700);
  c3->Divide(2,2);
  c3->cd(1);
  RangevsEner->Draw();
  c3->cd(2);
  VertexEvsEnersum->Draw("zcol");
  c3->cd(3);
  AnglevsEnersum->Draw("zcol");
  c3->cd(4);
  VertexXvsEnersum->Draw("zcol");

  TCanvas *c5p = new TCanvas("c5p","c5p",200,10,700,700);
  c5p->cd();
  RangevsEner->Draw();
  TCanvas *c6p = new TCanvas("c6p","c6p",200,10,700,700);
  c6p->cd();
  VertexEvsEnersum->GetXaxis()->SetTitle("Sum Energy (MeV)");
  VertexEvsEnersum->GetYaxis()->SetTitle("Vertex Energy (MeV)");
  VertexEvsEnersum->Draw("zcol");
  TCanvas *c7p = new TCanvas("c7p","c7p",200,10,700,700);
  c7p->cd();
  AnglevsEnersum->GetXaxis()->SetTitle("Scattering Angle");
  AnglevsEnersum->GetYaxis()->SetTitle("Energy Ratio");
  AnglevsEnersum->Draw("zcol");
  TCanvas *c8p = new TCanvas("c8p","c8p",200,10,700,700);
  c8p->cd();
  VertexXvsEnersum->GetYaxis()->SetTitle("Sum Energy (MeV)");
  VertexXvsEnersum->GetXaxis()->SetTitle("Vertex (mm)");
  VertexXvsEnersum->Draw("zcol");


  TCanvas *cMC = new TCanvas("cMC","cMC",200,10,700,700);
  cMC->Divide(2,2);
  cMC->cd(1);
  AngleSumMC->Draw();
  cMC->cd(2);
  Chi2MC->Draw();
  cMC->cd(3);
  AnglevsEnersumMC->Draw("zcol");
  cMC->cd(4);
  VertexEvsEnersumMC->Draw("zcol");

  TCanvas *c9p = new TCanvas("c9p","c9p",200,10,700,700);
  c9p->cd();
  AngleSumMC->GetXaxis()->SetTitle("Sum Scattering Angle");
  AngleSumMC->Draw();
  TCanvas *c10p = new TCanvas("c10p","c10p",200,10,700,700);
  c10p->cd();
  Chi2MC->GetXaxis()->SetTitle("Chi2");
  Chi2MC->Draw();
  TCanvas *c11p = new TCanvas("c11p","c11p",200,10,700,700);
  c11p->cd();
  AnglevsEnersumMC->GetXaxis()->SetTitle("Scattering Angle");
  AnglevsEnersumMC->GetYaxis()->SetTitle("Energy Ratio");
  AnglevsEnersumMC->Draw("zcol");
  TCanvas *c12p = new TCanvas("c12p","c12p",200,10,700,700);
  c12p->cd();
  VertexEvsEnersumMC->GetXaxis()->SetTitle("Sum Energy (MeV)");
  VertexEvsEnersumMC->GetYaxis()->SetTitle("Vertex (mm)");
  VertexEvsEnersumMC->Draw("zcol");

  TCanvas *c4 = new TCanvas("c4","c4",200,10,700,700);
  AnglePhi_vs_AngleSum->Draw("zcol");
  AnglePhi_vs_AngleSum->GetXaxis()->SetTitle("Scattering Angle");
  AnglePhi_vs_AngleSum->GetYaxis()->SetTitle("Sum Scattering Angle");

  TCanvas *c5 = new TCanvas("c5","c5",200,10,700,700);
  Range1_vs_Range2->Draw("zcol");

  TCanvas *c6 = new TCanvas("c6","c6",200,10,700,700);
  E1vsE2->Draw("zcol");


}
