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
#include "../../include/ATProtoEventAna.hh"
#include "../../include/ATAnalysis.hh"
#include "../../include/ATHoughSpaceLine.hh"
#include "../../include/ATHoughSpaceCircle.hh"

#include <ios>
#include <iostream>
#include <istream>
#include <limits>

void run_plot(TString FileNameHead = "output_proto",TString fileKine="../Kinematics/Decay_kinematics/10Be_4He_19MeV.txt")
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

  TCanvas *c2 = new TCanvas("c2","c2",200,10,700,700);
  c2->Divide(2,1);
  TCanvas *c3 = new TCanvas("c3","c3",200,10,700,700);
  //c3->Divide(2,1);

  TH2D* Q02_Kine = new TH2D("Q02_Kine","Q02_Kine",1000,0,180,1000,0,180);
  Q02_Kine->SetMarkerColor(2);
  Q02_Kine->SetMarkerStyle(20);
  Q02_Kine->SetMarkerSize(0.7);
  TH2D* Q13_Kine = new TH2D("Q13_Kine","Q13_Kine",1000,0,180,1000,0,180);
  Q13_Kine->SetMarkerColor(2);
  Q13_Kine->SetMarkerStyle(20);
  Q13_Kine->SetMarkerSize(0.7);

  TH1D* Vertex = new TH1D("Vertex","Vertex",100,0,1000);
  TH2D* Vertex_vs_Angle = new TH2D("Vertex_vs_Angle","Vertex_vs_Angle",1000,0,1000,200,0,180);


                TCutG *cutg = new TCutG("CUTG",27);
                cutg->SetVarX("Q02_Kine");
                cutg->SetVarY("");
                cutg->SetTitle("Graph");
                cutg->SetFillColor(1);
                cutg->SetPoint(0,13.91253,83.41335);
                cutg->SetPoint(1,16.43826,70.76698);
                cutg->SetPoint(2,29.69832,47.3185);
                cutg->SetPoint(3,32.22404,36.51639);
                cutg->SetPoint(4,49.90412,29.13934);
                cutg->SetPoint(5,68.21562,18.86417);
                cutg->SetPoint(6,79.79186,15.70258);
                cutg->SetPoint(7,86.73761,10.96019);
                cutg->SetPoint(8,86.52713,6.217797);
                cutg->SetPoint(9,77.05566,5.427399);
                cutg->SetPoint(10,69.26801,8.325526);
                cutg->SetPoint(11,55.79747,15.17564);
                cutg->SetPoint(12,50.53555,20.9719);
                cutg->SetPoint(13,35.80215,25.97775);
                cutg->SetPoint(14,31.17165,21.23536);
                cutg->SetPoint(15,24.43639,19.3911);
                cutg->SetPoint(16,19.80589,24.39695);
                cutg->SetPoint(17,20.01637,30.98361);
                cutg->SetPoint(18,22.75257,36.25293);
                cutg->SetPoint(19,21.91066,46.79157);
                cutg->SetPoint(20,16.0173,54.16862);
                cutg->SetPoint(21,8.229653,66.02459);
                cutg->SetPoint(22,6.545836,73.13817);
                cutg->SetPoint(23,7.177267,86.04801);
                cutg->SetPoint(24,11.59729,88.15574);
                cutg->SetPoint(25,13.70206,83.41335);
                cutg->SetPoint(26,13.91253,83.41335);

  TTreeReader Reader1("cbmsim", file);
  TTreeReaderValue<TClonesArray> analysisArray(Reader1, "ATProtoEventAna");


        		while (Reader1.Next()) {

              ATProtoEventAna* analysis = (ATProtoEventAna*) analysisArray->At(0);
              //Double_t ATProtoAnalysis::*HoughDist = &ATProtoAnalysis::fHoughDist;
              std::vector<Double_t> *AngleFit = analysis->GetAngleFit();
              std::vector<Double_t> *Par0     = analysis->GetPar0();
              std::vector<Double_t> *vertex   = analysis->GetVertex();
              if( TMath::Abs(vertex->at(0) - vertex->at(2))<20 && (vertex->at(0)>0 && vertex->at(2)>0)) Q02_Kine->Fill(AngleFit->at(0),AngleFit->at(2));
              if( TMath::Abs(vertex->at(1) - vertex->at(3))<20 && (vertex->at(1)>0 && vertex->at(3)>0)) Q02_Kine->Fill(AngleFit->at(1),AngleFit->at(3));

              if(cutg->IsInside(AngleFit->at(0),AngleFit->at(2)) ) {Vertex->Fill(Par0->at(0));Vertex_vs_Angle->Fill(Par0->at(0),AngleFit->at(0));}
              if(cutg->IsInside(AngleFit->at(1),AngleFit->at(3)) ) {Vertex->Fill(Par0->at(1));Vertex_vs_Angle->Fill(Par0->at(1),AngleFit->at(1));}


            }

            Double_t *ThetaCMS = new Double_t[20000];
            Double_t *ThetaLabRec = new Double_t[20000];
            Double_t *EnerLabRec = new Double_t[20000];
            Double_t *ThetaLabSca = new Double_t[20000];
            Double_t *EnerLabSca = new Double_t[20000];

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


                c2->cd(1);
                Vertex->Draw();
                c2->cd(2);
                Vertex_vs_Angle->Draw("zcol");

                c3->cd();
                Q02_Kine->Draw("");
                Kine_AngRec_AngSca->Draw("C");
                Kine_AngRec_AngSca_vert->Draw("C");
                cutg->Draw("l");


              //  c3->cd(2);
              //  Q13_Kine->Draw("");
                //Kine_AngRec_AngSca->Draw("C");
                //Kine_AngRec_AngSca_vert->Draw("C");


}
