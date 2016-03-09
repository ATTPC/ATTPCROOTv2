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

#include "../../include/ATEvent.hh"
#include "../../include/ATHit.hh"
#include "../../include/ATProtoEvent.hh"
#include "../../include/ATHoughSpaceLine.hh"
#include "../../include/ATHoughSpaceCircle.hh"

#include <ios>
#include <iostream>
#include <istream>
#include <limits>


std::pair<Double_t,Double_t> GetHoughParameters(TH2F* hist);
void myflush ( std::istream& in );
void mypause();

void run_ana_46Ar(TString FileNameHead = "run_",
Int_t num_ev=100000000, Int_t file_ini=87, Int_t file_end=89, Int_t runnum=250, TString file="../Kinematics/Decay_kinematics/Kine.txt")
{

	    //gStyle->SetCanvasPreferGL(1);
      //gStyle->SetPalette(1);
		  //if(!debug)	gErrorIgnoreLevel=kFatal; //Avoid printing Minuit errors
			FairRunAna* run = new FairRunAna();

	    TCanvas *c1 = new TCanvas("c1","c1",200,10,700,700);
	    c1->Divide(2,2);
	    /*TCanvas *c2 = new TCanvas("c2","c2",200,10,700,700);
	    c2->Divide(2,3);
      TCanvas *c3 = new TCanvas("c3","c3",200,10,700,700);
	    c3->Divide(2,1);
			TCanvas *c4 = new TCanvas("c4","c4",200,10,700,700);
	    c4->Divide(2,2);*/

			TH2D* EAKine = new TH2D("EAKine","EAKine",100,0,10,500,0,180);
	    EAKine->SetMarkerColor(2);
	    EAKine->SetMarkerStyle(20);
	    EAKine->SetMarkerSize(0.7);

			Int_t value;
			Int_t nEve=0;


	    TChain *chain = new TChain("cbmsim");
	    TFileCollection *filecol = new TFileCollection();
	    TString FileNameHead_num;
	    TString FileNameHead_chain;
			TString FilePath = "/data/ar46/analysis/46Ar/";

      for(Int_t i=file_ini;i<=file_end;i++){
				if(i<10) FileNameHead_num.Form("_000%i",i);
				else if(i<100) FileNameHead_num.Form("_00%i",i);
				else if(i<1000) FileNameHead_num.Form("_0%i",i);
				FileNameHead_chain = FilePath+"run"+FileNameHead_num+".root";
				std::cout<<" File : "<<FileNameHead_chain<<" added"<<std::endl;
				//filecol->Add(FileNameHead_chain);
				//chain->Add(FileNameHead_chain);

				TFile* file = new TFile(FileNameHead_chain.Data(),"READ");
				TTree* tree = (TTree*) file -> Get("cbmsim");
		    Int_t nEvents = tree -> GetEntriesFast();
		    std::cout<<" Number of events : "<<nEvents<<std::endl;
				TTreeReader Reader1("cbmsim", file);
				TTreeReaderValue<TClonesArray> houghArray(Reader1, "ATHough");

				ATHoughSpaceCircle* fHoughSpaceCircle;

					while (Reader1.Next() && nEve<num_ev) {
						nEve++;

						fHoughSpaceCircle  = dynamic_cast<ATHoughSpaceCircle*> (houghArray->At(0));
						Double_t EnerMin   = fHoughSpaceCircle->FitParameters.sEnerMin;
						Double_t NormChi2  = fHoughSpaceCircle->FitParameters.sNormChi2;
						Double_t ThetaMin  = 180.0*fHoughSpaceCircle->FitParameters.sThetaMin/TMath::Pi();
						Double_t VertexPos = fHoughSpaceCircle->FitParameters.sVertexPos.Z();
						//std::cout<<EnerMin<<"  "<<NormChi2<<std::endl;
						if(nEve%1000==0) std::cout<<" Event : "<<nEve<<"/"<<nEvents<<std::endl;
						if(NormChi2<4) EAKine->Fill(EnerMin,ThetaMin);

					}



	}//for files


        /*Double_t *ThetaCMS = new Double_t[20000];
       	Double_t *ThetaLabRec = new Double_t[20000];
				Double_t *EnerLabRec = new Double_t[20000];
				Double_t *ThetaLabSca = new Double_t[20000];
				Double_t *EnerLabSca = new Double_t[20000];

				std::ifstream *kineStr = new std::ifstream(file.Data());
				Int_t numKin=0;

				  if(!kineStr->fail()){
						while(!kineStr->eof()){
								*kineStr>>ThetaCMS[numKin]>>ThetaLabRec[numKin]>>EnerLabRec[numKin]>>ThetaLabSca[numKin]>>EnerLabSca[numKin];
								numKin++;
						}
					}else if(kineStr->fail()) std::cout<<" Warning : No Kinematics file found for this reaction! Please run the macro on $SIMPATH/macro/Kinematics/Decay_kinematics/Mainrel.cxx"<<std::endl;

					TGraph *Kine_AngRec_AngSca = new TGraph(numKin,ThetaLabRec,ThetaLabSca);
					TGraph *Kine_AngRec_AngSca_vert = new TGraph(numKin,ThetaLabSca,ThetaLabRec);*/

				//Kine_AngRec_AngSca->Draw("C");
				//Kine_AngRec_AngSca_vert->Draw("C");

				c1->cd(1);
				EAKine->Draw("zcol");

}



void myflush ( std::istream& in )
{
  in.ignore ( std::numeric_limits<std::streamsize>::max(), '\n' );
  in.clear();
}

void mypause()
{
  std::cout<<"Press [Enter] to continue . . .";
  std::cin.get();
}
