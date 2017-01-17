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


void run_ana_CO2(TString FileNameHead = "run_",
Int_t num_ev=100000000, Int_t file_ini=4, Int_t file_end=4, TString file="../Kinematics/Decay_kinematics/Kine.txt")
{

  TH1F* Energy = new TH1F("Energy","Energy",200,0,10);
  Energy->GetXaxis()->SetTitle("Energy (MeV)");

  TH1F* QEnergy = new TH1F("QEnergy","QEnergy",200,0,100000);
  QEnergy->GetXaxis()->SetTitle("Energy (a.u.)");

  TH2F* EvsQ = new TH2F("EvsQ","EvsQ",200,0,10,1000,0,100000);
  EvsQ->GetXaxis()->SetTitle("Energy (MeV)");
  EvsQ->GetYaxis()->SetTitle("Energy (a.u.)");


  TH2F* MultvsRange = new TH2F("MultvsRange","MultvsRange",200,0,400,200,0,600);
  MultvsRange->SetMarkerColor(2);
  MultvsRange->SetMarkerStyle(20);
  MultvsRange->SetMarkerSize(0.7);

  TH2F* AnglevsRange = new TH2F("AnglevsRange","AnglevsRange",400,0,4,200,0,600);





  FairRunAna* run = new FairRunAna();

  Int_t value;
  Int_t nEve=0;

  TChain *chain = new TChain("cbmsim");

  TString FileNameHead_num;
  TString FileNameHead_chain;
  TString FilePath = "~/data/CO2/Analysis/";

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

            for(Int_t i=0;i<tracks.size();i++)
            {

                    ATTrack track = tracks.at(i);

                    Double_t Eoffset = 0.168; //keV

                    if(tracks.size()==1){

                        //if(track.GetHitArray()->size()>200){
                           Energy->Fill(track.GetGeoEnergy()+Eoffset);
                           EvsQ->Fill(track.GetGeoEnergy()+Eoffset,track.GetGeoQEnergy());
                           QEnergy->Fill(track.GetGeoQEnergy());
			   MultvsRange->Fill(track.GetHitArray()->size(),track.GetLinearRange());
                           AnglevsRange->Fill(track.GetAngleZAxis(),track.GetLinearRange());
                        //}
                  }

            }


      }// Tree loop



    }// Open file if


  }// for files


  TCanvas *c1 = new TCanvas("c1","c1",200,10,700,700);
  c1->Divide(2,2);

  c1->cd(1);
  Energy->Draw();
  c1->cd(2);
  QEnergy->Draw();
  c1->cd(3);
  EvsQ->Draw("zcol");
  c1->cd(4);
  MultvsRange->Draw("zcol");

  TCanvas *c2 = new TCanvas("c2","c2",200,10,700,700);
  c2->Divide(1,2);
  c2->cd(1);
  MultvsRange->Draw("zcol");
  c2->cd(2);
  AnglevsRange->Draw("zcol");

  TCanvas *plot1 = new TCanvas("plot1","plot1",200,10,700,700);
  plot1->cd();
  Energy->Draw();

  TCanvas *plot2 = new TCanvas("plot2","plot2",200,10,700,700);
  plot2->cd();
  QEnergy->Draw();

  TCanvas *plot3 = new TCanvas("plot3","plot3",200,10,700,700);
  plot3->cd();
  EvsQ->Draw("zcol");
  



}
