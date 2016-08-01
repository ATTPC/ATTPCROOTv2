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


#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

std::pair<Double_t,Double_t> GetHoughParameters(TH2F* hist);
void myflush ( std::istream& in );
void mypause();

void run_ana_8He_multi(TString FileNameHead = "8He_TRIUMF_run_",
Int_t num_ev=3000000, Int_t file_ini=200, Int_t file_end=210, Int_t runnum=1, Float_t HoughDist=2.0,
Bool_t debug=kTRUE, Bool_t stdhough=kFALSE, TString file="../Kinematics/Decay_kinematics/Kine.txt")
{

				//Elastic cut
				TCutG *cutg = new TCutG("CUTG",18);
				cutg->SetVarX("Q02_Kine");
				cutg->SetVarY("");
				cutg->SetTitle("Graph");
				cutg->SetFillColor(1);
				cutg->SetPoint(0,32.87305,35.17606);
				cutg->SetPoint(1,60.4343,22.1831);
				cutg->SetPoint(2,72.21047,12.35915);
				cutg->SetPoint(3,68.20156,10.14084);
				cutg->SetPoint(4,64.69376,10.14084);
				cutg->SetPoint(5,36.88196,23.7676);
				cutg->SetPoint(6,27.86191,23.1338);
				cutg->SetPoint(7,23.10134,25.98591);
				cutg->SetPoint(8,22.09911,33.90845);
				cutg->SetPoint(9,23.60245,39.61268);
				cutg->SetPoint(10,10.07238,67.1831);
				cutg->SetPoint(11,8.318484,72.88732);
				cutg->SetPoint(12,11.07461,77.95775);
				cutg->SetPoint(13,12.32739,76.05634);
				cutg->SetPoint(14,22.09911,59.57746);
				cutg->SetPoint(15,31.36971,42.78169);
				cutg->SetPoint(16,32.12138,34.85915);
				cutg->SetPoint(17,32.87305,35.17606);


	    //gStyle->SetCanvasPreferGL(1);
      //gStyle->SetPalette(1);
		  if(!debug)	gErrorIgnoreLevel=kFatal; //Avoid printing Minuit errors


	    TCanvas *c1 = new TCanvas("c1","c1",200,10,700,700);
	    //c1->Divide(2,2);
	    TCanvas *c2 = new TCanvas("c2","c2",200,10,700,700);
	    /*c2->Divide(2,3);
      TCanvas *c3 = new TCanvas("c3","c3",200,10,700,700);
	    c3->Divide(2,1);
			TCanvas *c4 = new TCanvas("c4","c4",200,10,700,700);
	    c4->Divide(2,2);*/


      TH2D* Q02_Kine = new TH2D("Q02_Kine","Q02_Kine",1000,0,180,1000,0,180);
	    Q02_Kine->SetMarkerColor(2);
	    Q02_Kine->SetMarkerStyle(20);
	    Q02_Kine->SetMarkerSize(0.7);
	    TH2D* Q13_Kine = new TH2D("Q13_Kine","Q13_Kine",1000,0,180,1000,0,180);
	    Q13_Kine->SetMarkerColor(2);
	    Q13_Kine->SetMarkerStyle(20);
	    Q13_Kine->SetMarkerSize(0.7);

			TH2D* Vertex_vs_Angle = new TH2D("Vertex_vs_Angle","Vertex_vs_Angle",1000,0,1000,200,0,180);



		Int_t    tTreeEv=0;
		Double_t tHoughPar0[4]={0};
		Double_t tHoughPar1[4]={0};
		Double_t tFitPar0[4]={0};
		Double_t tFitPar1[4]={0};
		Double_t tAngleHough[4]={0};
		Double_t tAngleFit[4]={0};
		Double_t tVertex[4]={0};
		Double_t tChi2[4]={0};
		Double_t tNDF[4]={0};

		TFile *f = new TFile("analysis_8He_TRIUMF.root","RECREATE");
		TTree *treeOut = new TTree("AnalysisTree","AnalysisTree");
		treeOut->Branch("tTreeEv",&tTreeEv,"tTreeEv/I");
		treeOut->Branch("tHoughPar0",&tHoughPar0,"tHoughPar0[4]/D");
		treeOut->Branch("tHoughPar1",&tHoughPar1,"tHoughPar1[4]/D");
		treeOut->Branch("tFitPar0",&tFitPar0,"tFitPar0[4]/D");
		treeOut->Branch("tFitPar1",&tFitPar1,"tFitPar1[4]/D");
		treeOut->Branch("tAngleHough",&tAngleHough,"tAngleHough[4]/D");
		treeOut->Branch("tAngleFit",&tAngleFit,"tAngleFit[4]/D");
		treeOut->Branch("tVertex",&tVertex,"tVertex[4]/D");
		treeOut->Branch("tChi2",&tChi2,"tChi2[4]/D");
		treeOut->Branch("tNDF",&tNDF,"tNDF[4]/D");





			TString workdir = getenv("VMCWORKDIR");
	    TChain *chain = new TChain("cbmsim");
	    TFileCollection *filecol = new TFileCollection();
	    TString FileNameHead_num;
	    TString FileNameHead_chain;
			TString FilePath = "/data/ND/2013/buffer/";
			TString FileNameTail = ".root";
			TString FileName     = FilePath + FileNameHead + FileNameTail;



      for(Int_t i=file_ini;i<=file_end;i++){
				if(i<10) FileNameHead_num.Form("_000%i",i);
				else if(i<100) FileNameHead_num.Form("_00%i",i);
				else if(i<1000) FileNameHead_num.Form("_0%i",i);
				FileNameHead_chain = "8He_TRIUMF_run"+FileNameHead_num;
				std::cout<<" File : "<<FileNameHead_chain<<" added"<<std::endl;
				TString FileName     = FilePath + FileNameHead_chain + FileNameTail;
				//filecol->Add(FileNameHead_chain);
				//chain->Add(FileNameHead_chain);
	    //TString outFileNameHead = "data/";
	    //TString outFileNameTail = ".root";
	    //TString outFileName     = outFileNameHead + reactionName + outFileNameTail;
      //std::cout<<" Opening File : "<<FileName.Data()<<std::endl;

	     TFile* file = new TFile(FileName.Data(),"READ");
	    //TFile* file = new TFile(FileNameHead_chain.Data(),"READ");
      TTree* tree = (TTree*) file -> Get("cbmsim");
	    Int_t nEvents = tree -> GetEntriesFast();
	    std::cout<<" Number of events : "<<nEvents<<std::endl;

	    TTreeReader Reader1("cbmsim", file);
	    //TTreeReaderValue<TClonesArray> eventArray(Reader1, "ATEventH");
	    //TTreeReaderValue<TClonesArray> protoeventArray(Reader1, "ATProtoEvent");
	    //TTreeReaderValue<TClonesArray> houghArray(Reader1, "ATHough");
      TTreeReaderValue<TClonesArray> protoeventanaArray(Reader1, "ATProtoEventAna");
	    Bool_t fIsLinear=kFALSE;
	    Bool_t fIsCircular=kFALSE;

	    Int_t value;
	    Int_t nEve=0;

		while (Reader1.Next() && nEve<num_ev) {
			tTreeEv=nEve;
			if(nEve%100000==0)std::cout<<cRED<<" Event Number : "<<nEve<<cNORMAL<<std::endl;


		 ATProtoEventAna* protoeventana = (ATProtoEventAna*) protoeventanaArray->At(0);
		 std::vector<Double_t>* AngleFit =  protoeventana->GetAngleFit();
		 std::vector<Double_t>* vertex = protoeventana->GetVertex();
     std::vector<Double_t>* Chi2   = protoeventana->GetChi2();
     std::vector<Int_t>*    NDF    = protoeventana->GetNDF();


		 for(Int_t i=0;i<4;i++){
			 				tAngleFit[i] = AngleFit->at(i);
							tVertex[i]   = vertex->at(i);
							tChi2[i]     = Chi2->at(i);
							tNDF[i]      = NDF->at(i);

		 }

		 if( TMath::Abs(vertex->at(0) - vertex->at(2))<20 && (vertex->at(0)>0 && vertex->at(2)>0))
			 if( Chi2->at(0)/NDF->at(0)<10.0  && Chi2->at(2)/NDF->at(2)<10.0  ) Q02_Kine->Fill(AngleFit->at(0),AngleFit->at(2));

		 if( TMath::Abs(vertex->at(1) - vertex->at(3))<20 && (vertex->at(1)>0 && vertex->at(3)>0))
						 if( Chi2->at(1)/NDF->at(1)<10.0  && Chi2->at(3)/NDF->at(3)<10.0  ) Q02_Kine->Fill(AngleFit->at(1),AngleFit->at(3));


						if(cutg->IsInside(AngleFit->at(0),AngleFit->at(2)) ) Vertex_vs_Angle->Fill(vertex->at(0),AngleFit->at(0));
		 				if(cutg->IsInside(AngleFit->at(1),AngleFit->at(3)) ) Vertex_vs_Angle->Fill(vertex->at(1),AngleFit->at(1));


					nEve++;
					treeOut->Fill();


		}//While

			file->Close();

	}//for files

	      f->cd();
	      treeOut->Write();
      	f->Close();


				c1->cd();
				Q02_Kine->Draw("zcol");
				cutg->Draw("l");

				c2->cd();
				Vertex_vs_Angle->Draw("zcol");



        Double_t *ThetaCMS = new Double_t[20000];
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
					TGraph *Kine_AngRec_AngSca_vert = new TGraph(numKin,ThetaLabSca,ThetaLabRec);









}

/*std::pair<Double_t,Double_t> GetHoughParameters(TH2F* hist){

		std::pair<Double_t,Double_t> HoughPar;
		Int_t locmaxx,locmaxy,locmaxz;
                hist->GetMaximumBin(locmaxx,locmaxy,locmaxz);
                Double_t xpos = hist->GetXaxis()->GetBinCenter(locmaxx);
                Double_t ypos = hist->GetYaxis()->GetBinCenter(locmaxy);
		//std::cout<<" X Hough Position : "<<xpos<<std::endl;
		//std::cout<<" Y Hough Position : "<<ypos<<std::endl;
		HoughPar.first= xpos;
		HoughPar.second= ypos;
		return HoughPar;

}*/

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
