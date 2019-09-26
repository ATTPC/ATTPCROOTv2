#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TH2.h"
#include "TH1.h"
#include "TH3.h"
#include "TClonesArray.h"
#include "TCanvas.h"
#include "TMath.h"

#include <iostream>
#include <fstream>

void ana(Int_t num_ev=10){


  //HISTOS
  TH2D *tracks = new TH2D("tracks","tracks",1000,-100,1000,1000,-300,300);

  TH2D *Eloss_vs_Range_Rec = new TH2D("Eloss_vs_Range_Rec","ELoss_vs_Range_Rec",1000,0,2000,1000,0,10);
  Eloss_vs_Range_Rec->SetMarkerStyle(20);
  Eloss_vs_Range_Rec->SetMarkerSize(0.5);
  Eloss_vs_Range_Rec->SetMarkerColor(2);

  TH2D *Eloss_vs_Angle_Rec = new TH2D("Eloss_vs_Angle_Rec","ELoss_vs_Angle_Rec",1000,0,180,1000,0,10);
  Eloss_vs_Angle_Rec->SetMarkerStyle(20);
  Eloss_vs_Angle_Rec->SetMarkerSize(0.5);
  Eloss_vs_Angle_Rec->SetMarkerColor(2);

  TH2D *KineRec = new TH2D("KineRec","KineRec",1000,0,180,1000,0,40);
  KineRec->SetMarkerStyle(20);
  KineRec->SetMarkerSize(0.5);
  KineRec->SetMarkerColor(2);


   TCanvas *c1 = new TCanvas();
   c1->Divide(2,2);
   c1->Draw();	


  TString mcFileNameHead = "../data/attpcsim";
  TString mcFileNameTail = ".root";
  TString mcFileName     = mcFileNameHead + mcFileNameTail;
  TString outFileNameHead = "../data/AT3PC_sim";
  TString outFileNameTail = ".root";
  TString outFileName     = outFileNameHead + outFileNameTail;

   Double_t RecAng,RecEner,RecRange,RecELoss;
  Double_t ScaAng,ScaEner,ScaRange,ScaELoss;

  
  TFile* file = new TFile(mcFileName.Data(),"READ");
  TTree* tree = (TTree*) file -> Get("cbmsim");
  
  
  AtTpcPoint* point = new AtTpcPoint();
  TClonesArray *pointArray=0;


  TFile* outfile = new TFile(outFileName.Data(),"RECREATE");
  TTree *treeOut = new TTree("treeOut","ATTPCSimTree");
  treeOut->Branch("RecAng",&RecAng,"RecAng/D");
  treeOut->Branch("RecEner",&RecEner,"RecEner/D");
  treeOut->Branch("RecRange",&RecRange,"RecRange/D");
  treeOut->Branch("RecELoss",&RecELoss,"RecELoss/D");
  treeOut->Branch("ScaAng",&ScaAng,"ScaAng/D");
  treeOut->Branch("ScaEner",&ScaAng,"ScaEner/D");
  treeOut->Branch("ScaRange",&ScaRange,"ScaRange/D");
  treeOut->Branch("ScaELoss",&ScaELoss,"ScaELoss/D");
  

  tree = (TTree*) file -> Get("cbmsim");
  tree -> SetBranchAddress("AtTpcPoint", &pointArray);
  Int_t nEvents = tree -> GetEntriesFast();
  
  Double_t vertex =0.0;
  
  if(nEvents>num_ev) nEvents=num_ev;

  	for(Int_t iEvent=0; iEvent<nEvents; iEvent++){

  		Double_t energyLoss_sca=0.0;
    	Double_t range_sca=0.0;
    	Double_t energyLoss_rec=0.0;
    	Double_t range_rec=0.0;
    	Double_t range_beam=0.0;
    	Double_t BeamEnergyLoss_IC=0.0;
    	Double_t EnergyRecoil = 0.0;
    	Double_t EnergyBeam=0.0;
    	Double_t EnergySca=0.0;
    	Double_t AngleRecoil = 0.0;
    	Double_t AngleSca = 0.0;
    	Double_t zpos = 0.0;
    	Double_t xpos = 0.0;
    	Double_t radius=0.0;
    	Double_t radmean=0.0;
    	Double_t thetamean=0.0;
    	Double_t theta=0.0;
    	Int_t n2=0;
    	Int_t nrad=0;


  		tree->GetEvent(iEvent);
    	// tree -> GetEntry(iEvent);
    	Int_t n = pointArray -> GetEntries();
    	std::cout<<" Event Number : "<<iEvent<<std::endl;


    	for(Int_t i=0; i<n; i++) {
      
		      point = (AtTpcPoint*) pointArray -> At(i);
		      TString VolName=point->GetVolName();
		      //std::cout<<" Volume Name : "<<VolName<<std::endl;
		      Int_t trackID = point -> GetTrackID();

			       //RECOIL INFORMATION
			   if(trackID==2 && VolName=="drift_volume"){ //RECOIL
				n2++;
				range_rec = point -> GetLength()*10; //mm
				RecRange = range_rec;
				energyLoss_rec+=( point -> GetEnergyLoss() )*1000;//MeV
				EnergyRecoil= point->GetEIni();
				RecEner = EnergyRecoil;
				AngleRecoil= point->GetAIni();
				RecAng = AngleRecoil;
				zpos=point->GetZ()*10;
				xpos=point->GetXIn()*10;
				tracks->Fill(zpos,xpos);
				}

		}//Points

		if(iEvent%2!=0){
     		 Eloss_vs_Range_Rec->Fill(range_rec,energyLoss_rec);
     		 KineRec->Fill(RecAng,RecEner);
     		 std::cout<<RecAng<<"	"<<RecEner<<"\n";
     		 Eloss_vs_Angle_Rec->Fill(RecAng,energyLoss_rec);
			 RecELoss =  energyLoss_rec;  

	    }

		treeOut->Fill();

  	}//Events

  	c1->cd(1);
  	tracks->Draw("col");
  	c1->cd(2);
  	Eloss_vs_Range_Rec->Draw("scat");
  	c1->cd(3);
  	KineRec->Draw("scat");
  	c1->cd(4);
  	Eloss_vs_Angle_Rec->Draw("scat");



  	//outfile->cd();
  	treeOut->Write();
  	outfile->Close();



}