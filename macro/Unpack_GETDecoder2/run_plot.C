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
#include "../../include/ATAnalysis.hh"
#include "../../include/ATHoughSpaceLine.hh"
#include "../../include/ATHoughSpaceCircle.hh"

#include <ios>
#include <iostream>
#include <istream>
#include <limits>

void run_plot(TString FileNameHead = "output_proto_ana")
{

  TString workdir = getenv("VMCWORKDIR");
  TString FilePath = workdir + "/macro/Unpack_GETDecoder2/";
  TString FileNameTail = ".root";
  TString FileName     = FilePath + FileNameHead + FileNameTail;
  std::cout<<" Opening File : "<<FileName.Data()<<std::endl;
  TFile* file = new TFile(FileName.Data(),"READ");
  //TFile* file = new TFile(FileNameHead_chain.Data(),"READ");
  TTree* tree = (TTree*) file -> Get("cbmsim");
  Int_t nEvents = tree -> GetEntriesFast();
  std::cout<<" Number of events : "<<nEvents<<std::endl;

  TCanvas *c3 = new TCanvas("c3","c3",200,10,700,700);
  c3->Divide(2,1);

  TH2D* Q02_Kine = new TH2D("Q02_Kine","Q02_Kine",1000,0,180,1000,0,180);
  Q02_Kine->SetMarkerColor(2);
  Q02_Kine->SetMarkerStyle(20);
  Q02_Kine->SetMarkerSize(0.7);
  TH2D* Q13_Kine = new TH2D("Q13_Kine","Q13_Kine",1000,0,180,1000,0,180);
  Q13_Kine->SetMarkerColor(2);
  Q13_Kine->SetMarkerStyle(20);
  Q13_Kine->SetMarkerSize(0.7);

  TTreeReader Reader1("cbmsim", file);
  TTreeReaderValue<TClonesArray> analysisArray(Reader1, "ATAnalysis");


        		while (Reader1.Next()) {

              ATProtoAnalysis* analysis = (ATProtoAnalysis*) analysisArray->At(0);
              //Double_t ATProtoAnalysis::*HoughDist = &ATProtoAnalysis::fHoughDist;
              std::vector<Double_t> *AngleFit = analysis->GetAngleFit();
              Q02_Kine->Fill(AngleFit->at(0),AngleFit->at(2));
              Q02_Kine->Fill(AngleFit->at(1),AngleFit->at(3));


            }

            c3->cd(1);
            Q02_Kine->Draw("");
            //Kine_AngRec_AngSca->Draw("C");
            //Kine_AngRec_AngSca_vert->Draw("C");


            c3->cd(2);
            Q13_Kine->Draw("");
            //Kine_AngRec_AngSca->Draw("C");
            //Kine_AngRec_AngSca_vert->Draw("C");



}
