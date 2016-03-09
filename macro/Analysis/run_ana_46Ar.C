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
Int_t num_ev=100000000, Int_t file_ini=86, Int_t file_end=93, Int_t runnum=250, TString file="../Kinematics/Decay_kinematics/Kine.txt")
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

			Double_t tEnerMin;
			Double_t tNormChi2;
			Double_t tThetaMin;
			Double_t tVertexPos;

			TH2D* EAKine = new TH2D("EAKine","EAKine",500,0,180,100,0,10);
	    EAKine->SetMarkerColor(2);
	    EAKine->SetMarkerStyle(20);
	    EAKine->SetMarkerSize(0.7);

			Int_t value;
			Int_t nEve=0;

			TFile *f = new TFile("analysis.root","RECREATE");
			TTree *treeOut = new TTree("AnalysisTree","AnalysisTree");
			treeOut->Branch("tEnerMin",&tEnerMin,"tEnerMin/D");
			treeOut->Branch("tNormChi2",&tNormChi2,"tNormChi2/D");
			treeOut->Branch("tThetaMin",&tThetaMin,"tThetaMin/D");
			treeOut->Branch("tVertexPos",&tVertexPos,"tVertexPos/D");

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

			if(file->IsOpen()){
				std::cout<<" Analyzing"<<std::endl;

				TTree* tree = (TTree*) file -> Get("cbmsim");
		    Int_t nEvents = tree -> GetEntriesFast();
		    std::cout<<" Number of events : "<<nEvents<<std::endl;
				TTreeReader Reader1("cbmsim", file);
				TTreeReaderValue<TClonesArray> houghArray(Reader1, "ATHough");

				ATHoughSpaceCircle* fHoughSpaceCircle;

					while (Reader1.Next() && nEve<num_ev) {
						nEve++;

						fHoughSpaceCircle  = dynamic_cast<ATHoughSpaceCircle*> (houghArray->At(0));
						tEnerMin   = fHoughSpaceCircle->FitParameters.sEnerMin;
						tNormChi2  = fHoughSpaceCircle->FitParameters.sNormChi2;
						tThetaMin  = 180.0*fHoughSpaceCircle->FitParameters.sThetaMin/TMath::Pi();
						tVertexPos = fHoughSpaceCircle->FitParameters.sVertexPos.Z();
						//std::cout<<EnerMin<<"  "<<NormChi2<<std::endl;
						if(nEve%1000==0) std::cout<<" Event : "<<nEve<<"/"<<nEvents<<std::endl;
						if(tNormChi2<4) EAKine->Fill(tThetaMin,tEnerMin);


						treeOut->Fill();

					}

				}

				file->Close();

	}//for files


	Double_t *ThetaCMS = new Double_t[20000];
	Double_t *ThetaLabRec = new Double_t[20000];
	Double_t *EnerLabRec = new Double_t[20000];
	Double_t *ThetaLabSca = new Double_t[20000];
	Double_t *EnerLabSca = new Double_t[20000];

	Double_t *ThetaCMS2 = new Double_t[20000];
	Double_t *ThetaLabRec2 = new Double_t[20000];
	Double_t *EnerLabRec2 = new Double_t[20000];
	Double_t *ThetaLabSca2 = new Double_t[20000];
	Double_t *EnerLabSca2 = new Double_t[20000];

	Double_t *ThetaCMS3 = new Double_t[20000];
	Double_t *ThetaLabRec3 = new Double_t[20000];
	Double_t *EnerLabRec3 = new Double_t[20000];
	Double_t *ThetaLabSca3 = new Double_t[20000];
	Double_t *EnerLabSca3 = new Double_t[20000];

	TString fileKine="../Kinematics/Decay_kinematics/46Ar_p_4.1MeV.txt";
	std::ifstream *kineStr = new std::ifstream(fileKine.Data());
	Int_t numKin=0;

		if(!kineStr->fail()){
			while(!kineStr->eof()){
					*kineStr>>ThetaCMS[numKin]>>ThetaLabRec[numKin]>>EnerLabRec[numKin]>>ThetaLabSca[numKin]>>EnerLabSca[numKin];
					numKin++;
			}
		}else if(kineStr->fail()) std::cout<<" Warning : No Kinematics file found for this reaction! Please run the macro on $SIMPATH/macro/Kinematics/Decay_kinematics/Mainrel.cxx"<<std::endl;

		TGraph *Kine_AngRec_AngSca = new TGraph(numKin,ThetaLabRec,ThetaLabSca);
		TGraph *Kine_AngRec_AngSca_vert = new TGraph(numKin,ThetaLabSca,ThetaLabRec);
		TGraph *Kine_AngRec_EnerRec = new TGraph(numKin,ThetaLabRec,EnerLabRec);

		TString fileKine2="../Kinematics/Decay_kinematics/46Ar_p_2.64MeV.txt";
		std::ifstream *kineStr2 = new std::ifstream(fileKine2.Data());
	  numKin=0;

		if(!kineStr2->fail()){
			while(!kineStr2->eof()){
					*kineStr2>>ThetaCMS2[numKin]>>ThetaLabRec2[numKin]>>EnerLabRec2[numKin]>>ThetaLabSca2[numKin]>>EnerLabSca2[numKin];
					numKin++;
			}
		}else if(kineStr2->fail()) std::cout<<" Warning : No Kinematics file found for this reaction! Please run the macro on $SIMPATH/macro/Kinematics/Decay_kinematics/Mainrel.cxx"<<std::endl;

		TGraph *Kine_AngRec_AngSca2 = new TGraph(numKin,ThetaLabRec2,ThetaLabSca2);
		TGraph *Kine_AngRec_AngSca_vert2 = new TGraph(numKin,ThetaLabSca2,ThetaLabRec2);
		TGraph *Kine_AngRec_EnerRec2 = new TGraph(numKin,ThetaLabRec2,EnerLabRec2);

		/*TString fileKine3="../Kinematics/Decay_kinematics/46Ar_p_3.79MeV.txt";
		std::ifstream *kineStr3 = new std::ifstream(fileKine3.Data());
		numKin=0;

		if(!kineStr3->fail()){
			while(!kineStr3->eof()){
					*kineStr3>>ThetaCMS3[numKin]>>ThetaLabRec3[numKin]>>EnerLabRec3[numKin]>>ThetaLabSca3[numKin]>>EnerLabSca3[numKin];
					numKin++;
			}
		}else if(kineStr3->fail()) std::cout<<" Warning : No Kinematics file found for this reaction! Please run the macro on $SIMPATH/macro/Kinematics/Decay_kinematics/Mainrel.cxx"<<std::endl;

		TGraph *Kine_AngRec_AngSca3 = new TGraph(numKin,ThetaLabRec3,ThetaLabSca3);
		TGraph *Kine_AngRec_AngSca_vert3 = new TGraph(numKin,ThetaLabSca3,ThetaLabRec3);
		TGraph *Kine_AngRec_EnerRec3 = new TGraph(numKin,ThetaLabRec3,EnerLabRec3);*/


   	f->cd();
		Kine_AngRec_EnerRec->Write();
		Kine_AngRec_EnerRec2->Write();
    treeOut->Write();
		f->Close();


				//Kine_AngRec_AngSca_vert->Draw("C");

				c1->cd(1);
				EAKine->Draw("zcol");
				Kine_AngRec_EnerRec->Draw("C");
				Kine_AngRec_EnerRec2->Draw("C");
				//Kine_AngRec_EnerRec3->Draw("C");
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
