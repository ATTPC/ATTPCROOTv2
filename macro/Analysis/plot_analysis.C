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

#include <ios>
#include <iostream>
#include <istream>
#include <limits>


#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

void plot_analysis(TString FileName="analysis.root")
{
  gROOT->ProcessLine(".x ~/Dropbox/rooton.C");
  TFile* file = new TFile(FileName.Data(),"READ");
  TTree* tree = (TTree*) file -> Get("AnalysisTree");
  Int_t nEvents = tree -> GetEntriesFast();
  std::cout<<" Number of events : "<<nEvents<<std::endl;

  TString fileKine2="../Kinematics/Decay_kinematics/46Ar_p_2.64MeV.txt";
  std::ifstream *kineStr2 = new std::ifstream(fileKine2.Data());
  numKin=0;

  Double_t *ThetaCMS2 = new Double_t[20000];
  Double_t *ThetaLabRec2 = new Double_t[20000];
  Double_t *EnerLabRec2 = new Double_t[20000];
  Double_t *ThetaLabSca2 = new Double_t[20000];
  Double_t *EnerLabSca2 = new Double_t[20000];

  if(!kineStr2->fail()){
    while(!kineStr2->eof()){
        *kineStr2>>ThetaCMS2[numKin]>>ThetaLabRec2[numKin]>>EnerLabRec2[numKin]>>ThetaLabSca2[numKin]>>EnerLabSca2[numKin];
        numKin++;
    }
  }else if(kineStr2->fail()) std::cout<<" Warning : No Kinematics file found for this reaction! Please run the macro on $SIMPATH/macro/Kinematics/Decay_kinematics/Mainrel.cxx"<<std::endl;

  TGraph *Kine_AngRec_EnerRec2 = new TGraph(numKin,ThetaLabRec2,EnerLabRec2);

  TCanvas *c1=new TCanvas();
  c1->Divide(2,2);
  c1->cd(1);
  tree->Draw("tVertexPos>>h1(100,5,100)","tNormChi2<4 && tThetaMin>45 && tThetaMin<55 && tVertexEner>3","");
  c1->cd(2);
  tree->Draw("tVertexPos>>h2(100,5,100)","tNormChi2<4 && tThetaMin>30 && tThetaMin<40 && tVertexEner>3","");
  c1->cd(3);
  tree->Draw("tVertexPos>>h3(100,5,100)","tNormChi2<4 && tThetaMin>65 && tThetaMin<75 && tVertexEner>3","");
  c1->cd(4);
  gPad->SetLogz();
  tree->Draw("tVertexEner:tThetaMin>>h4(200,0,180,200,0,10)","tNormChi2<4","zcol");

  TCanvas *c2=new TCanvas();
  //c2->Divide(2,2);
  //c2->cd(1);
  tree->Draw("tVertexEner:tThetaMin>>h4(200,0,180,200,0,10)","tNormChi2<4 && tVertexPos<64 && tVertexPos>62","");
  Kine_AngRec_EnerRec2->Draw("C");


  TCanvas *c22=new TCanvas();
  tree->Draw("tThetaMin","tNormChi2<4 && tVertexPos<64 && tVertexPos>62","");

  TCanvas *c3=new TCanvas();
  c3->Divide(2,2);
  c3->cd(1);
  tree->Draw("tVertexPos>>h11(100,5,100)","tNormChi2<1 && tNormChi2>0 && tThetaMin>45 && tThetaMin<55 && tVertexEner>0","");
  c3->cd(2);
  tree->Draw("tVertexPos>>h12(100,5,100)","tNormChi2<1 && tNormChi2>0 && tThetaMin>45 && tThetaMin<55 && tVertexEner>1","");
  c3->cd(3);
  tree->Draw("tVertexPos>>h13(100,5,100)","tNormChi2<1 && tNormChi2>0 && tThetaMin>45 && tThetaMin<55 && tVertexEner>2","");
  c3->cd(4);
  tree->Draw("tVertexPos>>h14(100,5,100)","tNormChi2<1 && tNormChi2>0 && tThetaMin>45 && tThetaMin<55 && tVertexEner>3","");

  TCanvas *c4=new TCanvas();
  c4->Divide(2,2);
  c4->cd(1);
  tree->Draw("tVertexPos>>h21(100,5,100)","tNormChi2<1 && tNormChi2>0 && tThetaMin>55 && tThetaMin<60 && tVertexEner>0","");
  c4->cd(2);
  tree->Draw("tVertexPos>>h22(100,5,100)","tNormChi2<1 && tNormChi2>0 && tThetaMin>55 && tThetaMin<60 && tVertexEner>1","");
  c4->cd(3);
  tree->Draw("tVertexPos>>h23(100,5,100)","tNormChi2<1 && tNormChi2>0 && tThetaMin>55 && tThetaMin<60 && tVertexEner>2","");
  c4->cd(4);
  tree->Draw("tVertexPos>>h24(100,5,100)","tNormChi2<1 && tNormChi2>0 && tThetaMin>55 && tThetaMin<60 && tVertexEner>3","");

  TCanvas *c5=new TCanvas();
  c5->Divide(2,2);
  c5->cd(1);
  tree->Draw("tVertexPos>>h31(100,5,100)","tNormChi2<1 && tNormChi2>0 && tThetaMin>30 && tThetaMin<40 && tVertexEner>0","");
  c5->cd(2);
  tree->Draw("tVertexPos>>h32(100,5,100)","tNormChi2<1 && tNormChi2>0 && tThetaMin>30 && tThetaMin<40 && tVertexEner>1","");
  c5->cd(3);
  tree->Draw("tVertexPos>>h33(100,5,100)","tNormChi2<1 && tNormChi2>0 && tThetaMin>30 && tThetaMin<40 && tVertexEner>2","");
  c5->cd(4);
  tree->Draw("tVertexPos>>h34(100,5,100)","tNormChi2<1 && tNormChi2>0 && tThetaMin>30 && tThetaMin<40 && tVertexEner>3","");

  TCanvas *c6=new TCanvas();
  c6->Divide(1,1);
  c6->cd(1);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2):tThetaMin>>h61(200,0,180,1000,-10,200)","tNormChi2<3 && tNormChi2>0","zcol");



  TCanvas *c7=new TCanvas();
  c7->Divide(4,2);
  //gPad->SetLogy();
  c7->cd(1);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h71(100,1,20)","tNormChi2<5 && tNormChi2>0 && tThetaMin>85 && tThetaMin<90 && tVertexPos>0 && tVertexPos<100","zcol");
  c7->cd(2);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h72(100,1,20)","tNormChi2<5 && tNormChi2>0 && tThetaMin>80 && tThetaMin<85 && tVertexPos>0 && tVertexPos<100","zcol");
  c7->cd(3);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h73(100,1,20)","tNormChi2<5 && tNormChi2>0 && tThetaMin>75 && tThetaMin<80 && tVertexPos>0 && tVertexPos<100","zcol");
  c7->cd(4);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h74(100,1,20)","tNormChi2<5 && tNormChi2>0 && tThetaMin>70 && tThetaMin<75 && tVertexPos>0 && tVertexPos<100","zcol");
  c7->cd(5);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h75(100,1,20)","tNormChi2<5 && tNormChi2>0 && tThetaMin>65 && tThetaMin<70 && tVertexPos>0 && tVertexPos<100","zcol");
  c7->cd(6);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h76(100,1,20)","tNormChi2<5 && tNormChi2>0 && tThetaMin>60 && tThetaMin<65 && tVertexPos>0 && tVertexPos<100","zcol");
  c7->cd(7);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h77(100,1,20)","tNormChi2<5 && tNormChi2>0 && tThetaMin>55 && tThetaMin<60 && tVertexPos>0 && tVertexPos<100","zcol");
  c7->cd(8);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h78(100,1,20)","tNormChi2<5 && tNormChi2>0 && tThetaMin>50 && tThetaMin<55 && tVertexPos>0 && tVertexPos<100","zcol");

  TCanvas *c8=new TCanvas();
  c8->Divide(4,2);
  //gPad->SetLogy();
  c8->cd(1);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h81(100,1,20)","tNormChi2<1 && tNormChi2>0 && tThetaMin>45 && tThetaMin<50 && tVertexPos>0 && tVertexPos<100","zcol");
  c8->cd(2);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h82(100,1,20)","tNormChi2<1 && tNormChi2>0 && tThetaMin>40 && tThetaMin<45 && tVertexPos>0 && tVertexPos<100","zcol");
  c8->cd(3);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h83(100,1,20)","tNormChi2<1 && tNormChi2>0 && tThetaMin>35 && tThetaMin<40 && tVertexPos>0 && tVertexPos<100","zcol");
  c8->cd(4);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h84(100,1,20)","tNormChi2<1 && tNormChi2>0 && tThetaMin>30 && tThetaMin<35 && tVertexPos>0 && tVertexPos<100","zcol");
  c8->cd(5);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h85(100,1,20)","tNormChi2<1 && tNormChi2>0 && tThetaMin>25 && tThetaMin<30 && tVertexPos>0 && tVertexPos<100","zcol");
  c8->cd(6);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h86(100,1,20)","tNormChi2<1 && tNormChi2>0 && tThetaMin>20 && tThetaMin<25 && tVertexPos>0 && tVertexPos<100","zcol");
  c8->cd(7);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h87(100,1,20)","tNormChi2<1 && tNormChi2>0 && tThetaMin>15 && tThetaMin<20 && tVertexPos>0 && tVertexPos<100","zcol");
  c8->cd(8);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h88(100,1,20)","tNormChi2<1 && tNormChi2>0 && tThetaMin>10 && tThetaMin<15 && tVertexPos>0 && tVertexPos<100","zcol");


  TCanvas *c9=new TCanvas();
  c9->Divide(4,2);
  //gPad->SetLogy();
  c9->cd(1);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h91(100,1,20)","tNormChi2<4 && tNormChi2>0 && tThetaMin>80 && tThetaMin<90","zcol");
  c9->cd(2);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h92(100,1,20)","tNormChi2<4 && tNormChi2>0 && tThetaMin>70 && tThetaMin<80","zcol");
  c9->cd(3);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h93(100,1,20)","tNormChi2<4 && tNormChi2>0 && tThetaMin>60 && tThetaMin<70","zcol");
  c9->cd(4);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h94(100,1,20)","tNormChi2<4 && tNormChi2>0 && tThetaMin>50 && tThetaMin<60","zcol");
  c9->cd(5);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h95(100,1,20)","tNormChi2<4 && tNormChi2>0 && tThetaMin>40 && tThetaMin<50","zcol");
  c9->cd(6);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h96(100,1,20)","tNormChi2<4 && tNormChi2>0 && tThetaMin>30 && tThetaMin<40","zcol");
  c9->cd(7);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h97(100,1,20)","tNormChi2<4 && tNormChi2>0 && tThetaMin>20 && tThetaMin<30","zcol");
  c9->cd(8);
  tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h98(100,1,20)","tNormChi2<4 && tNormChi2>0 && tThetaMin>10 && tThetaMin<20","zcol");


   TCanvas *c10=new TCanvas();
   tree->Draw("tVertexEner/TMath::Power(TMath::Cos(tThetaMin*TMath::Pi()/180.0),2)>>h77(200,1,20)","tNormChi2<4 && tNormChi2>0 && tThetaMin>55 && tThetaMin<60 && tVertexPos>0 && tVertexPos<100","zcol");








}
