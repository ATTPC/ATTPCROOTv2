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

void run_plot(TString FileNameHead = "output_proto",TString fileKine="../Kinematics/Decay_kinematics/10Be_4He_19MeV.txt",TString fileKine2="../Kinematics/Decay_kinematics/10Be_12C_19MeV.txt")
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

                TCutG *cutg2 = new TCutG("CUTG2",6);
                cutg2->SetVarX("Q02_Kine");
                cutg2->SetVarY("");
                cutg2->SetTitle("Graph");
                cutg2->SetFillColor(1);
                cutg2->SetPoint(0,54.85507,28.95589);
                cutg2->SetPoint(1,64.59005,23.76482);
                cutg2->SetPoint(2,64.52764,10.0358);
                cutg2->SetPoint(3,54.81763,12.22152);
                cutg2->SetPoint(4,54.86755,29.0242);
                cutg2->SetPoint(5,54.85507,28.95589);

                TCutG *cutg3 = new TCutG("CUTG2",6);
                cutg3->SetVarX("Q02_Kine");
                cutg3->SetVarY("");
                cutg3->SetTitle("Graph");
                cutg3->SetFillColor(1);
                cutg3->SetPoint(0,27.46418,21.43491);
                cutg3->SetPoint(1,37.45702,16.44231);
                cutg3->SetPoint(2,37.13467,2.463016);
                cutg3->SetPoint(3,27.46418,4.792898);
                cutg3->SetPoint(4,27.78653,21.43491);
                cutg3->SetPoint(5,27.46418,21.43491);


  TTreeReader Reader1("cbmsim", file);
  TTreeReaderValue<TClonesArray> analysisArray(Reader1, "ATProtoEventAna");
  Int_t evnt = 0;


        		while (Reader1.Next()) {

              evnt++;
              ATProtoEventAna* analysis = (ATProtoEventAna*) analysisArray->At(0);
              //Double_t ATProtoAnalysis::*HoughDist = &ATProtoAnalysis::fHoughDist;
              std::vector<Double_t> *AngleFit = analysis->GetAngleFit();
              std::vector<Double_t> *Par0     = analysis->GetPar0();
              std::vector<Double_t> *vertex   = analysis->GetVertex();
              std::vector<Double_t> *Chi2     = analysis->GetChi2();
              std::vector<Int_t> *NDF         = analysis->GetNDF();

              if( TMath::Abs(vertex->at(0) - vertex->at(2))<20 && (vertex->at(0)>0 && vertex->at(2)>0))
                if( Chi2->at(0)/NDF->at(0)<1000.0  && Chi2->at(2)/NDF->at(2)<1000.0  ) Q02_Kine->Fill(AngleFit->at(0),AngleFit->at(2));

              if( TMath::Abs(vertex->at(1) - vertex->at(3))<20 && (vertex->at(1)>0 && vertex->at(3)>0))
                      if( Chi2->at(1)/NDF->at(1)<1000.0  && Chi2->at(3)/NDF->at(3)<1000.0  ) Q02_Kine->Fill(AngleFit->at(1),AngleFit->at(3));

              if(cutg->IsInside(AngleFit->at(0),AngleFit->at(2)) ) {Vertex->Fill(Par0->at(0));Vertex_vs_Angle->Fill(Par0->at(0),AngleFit->at(0));}
              if(cutg->IsInside(AngleFit->at(1),AngleFit->at(3)) ) {Vertex->Fill(Par0->at(1));Vertex_vs_Angle->Fill(Par0->at(1),AngleFit->at(1));}

              if(cutg3->IsInside(AngleFit->at(0),AngleFit->at(2)) ) std::cout<<evnt<<std::endl;


            }

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



              std::ifstream *kineStr2 = new std::ifstream(fileKine2.Data());
              Int_t numKin2=0;

              if(!kineStr2->fail()){
                while(!kineStr2->eof()){
                    *kineStr2>>ThetaCMS2[numKin2]>>ThetaLabRec2[numKin2]>>EnerLabRec2[numKin2]>>ThetaLabSca2[numKin2]>>EnerLabSca2[numKin2];
                    numKin2++;
                }
              }else if(kineStr2->fail()) std::cout<<" Warning : No Kinematics file found for this reaction! Please run the macro on $SIMPATH/macro/Kinematics/Decay_kinematics/Mainrel.cxx"<<std::endl;

              TGraph *Kine_AngRec_AngSca2 = new TGraph(numKin2,ThetaLabRec2,ThetaLabSca2);
              TGraph *Kine_AngRec_AngSca_vert2 = new TGraph(numKin2,ThetaLabSca2,ThetaLabRec2);


                c2->cd(1);
                Vertex->Draw();
                c2->cd(2);
                Vertex_vs_Angle->Draw("zcol");

                c3->cd();
                Q02_Kine->Draw("");
                Kine_AngRec_AngSca->Draw("C");
                Kine_AngRec_AngSca_vert->Draw("C");
                Kine_AngRec_AngSca2->Draw("C");
                Kine_AngRec_AngSca_vert2->Draw("C");
                cutg3->Draw("l");


              //  c3->cd(2);
              //  Q13_Kine->Draw("");
                //Kine_AngRec_AngSca->Draw("C");
                //Kine_AngRec_AngSca_vert->Draw("C");


}
