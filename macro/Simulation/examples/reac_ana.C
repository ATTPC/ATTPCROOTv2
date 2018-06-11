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

 TH2D *Angle_sca_vs_Angle_rec_1 = new TH2D("Angle_sca_vs_Angle_rec_1","Angle_sca_vs_Angle_rec_1",200,0,180,200,0,180);
 TH2D *Angle_sca_vs_Angle_rec_2 = new TH2D("Angle_sca_vs_Angle_rec_2","Angle_sca_vs_Angle_rec_2",200,0,180,200,0,180);

 TH2D *Range_vs_Angle_rec_1 = new TH2D("Range_vs_Angle_rec_1","Range_vs_Angle_rec_1",200,0,600,200,0,180);
 TH2D *Range_vs_Angle_rec_2 = new TH2D("Range_vs_Angle_rec_2","Range_vs_Angle_rec_2",200,0,600,200,0,180);

 TH2D *Radius_vs_Vertex_rec_1 = new TH2D("Radius_vs_Vertex_rec_1","Radius_vs_Vertex_rec_1",200,0,300,200,0,600);
 TH2D *Radius_vs_Vertex_rec_2 = new TH2D("Radius_vs_Vertex_rec_2","Radius_vs_Vertex_rec_2",200,0,300,200,0,600);

 TH2D *Ener_vs_Vertex_rec_1 = new TH2D("Ener_vs_Vertex_rec_1","Ener_vs_Vertex_rec_1",1000,0,50,1000,0,600);
 TH2D *Ener_vs_Vertex_rec_2 = new TH2D("Ener_vs_Vertex_rec_2","Ener_vs_Vertex_rec_2",1000,0,50,1000,0,600);

 TH2D *Ener_vs_Angle_rec_1 = new TH2D("Ener_vs_Angle_rec_1","Ener_vs_Angle_rec_1",1000,0,180,1000,0,50);
 TH2D *Ener_vs_Angle_sca_1 = new TH2D("Ener_vs_Angle_sca_1","Ener_vs_Angle_sca_1",1000,0,180,1000,0,50);

 TH2D *Ener_vs_Angle_rec_2 = new TH2D("Ener_vs_Angle_rec_2","Ener_vs_Angle_rec_2",1000,0,180,1000,0,50);
 TH2D *Ener_vs_Angle_sca_2 = new TH2D("Ener_vs_Angle_sca_2","Ener_vs_Angle_sca_2",1000,0,180,1000,0,50);


void reac_ana(TString reactionName1="8He_elas",TString reactionName2="8He_6He")
{



       TString inFileNameHead1 = "data/";
       TString inFileNameTail1 = ".root";
       TString inFileName1     = inFileNameHead1 + reactionName1 + inFileNameTail1;
       std::cout << " Simulated reaction file : " << inFileName1 << std::endl;

       TString inFileNameHead2 = "data/";
       TString inFileNameTail2 = ".root";
       TString inFileName2     = inFileNameHead2 + reactionName2 + inFileNameTail2;
       std::cout << " Simulated reaction file : " << inFileName2 << std::endl;

       //Double_t RecAng=0.0;

       TFile* file1 = new TFile(inFileName1.Data(),"READ");
       TTreeReader Reader1("treeOut", file1);
       TTreeReaderValue<Double_t> RecAng1(Reader1, "RecAng");
       TTreeReaderValue<Double_t> ScaAng1(Reader1, "ScaAng");
       TTreeReaderValue<Double_t> RecEner1(Reader1, "RecEner");
       TTreeReaderValue<Double_t> ScaEner1(Reader1, "ScaEner");
       TTreeReaderValue<Double_t> RecRange1(Reader1, "RecRange");
       TTreeReaderValue<Double_t> ScaRange1(Reader1, "ScaRange");
       TTreeReaderValue<Double_t> RecRad1(Reader1, "RecRad");
       TTreeReaderValue<Double_t> ScaRad1(Reader1, "ScaRad");
       TTreeReaderValue<Double_t> VertexPos1(Reader1, "VertexPos");
       TTreeReaderValue<Double_t> BeamEner1(Reader1, "BeamEner");
       TTreeReaderValue<Double_t> BeamRange1(Reader1, "BeamRange");

       TFile* file2 = new TFile(inFileName2.Data(),"READ");
       TTreeReader Reader2("treeOut", file2);
       TTreeReaderValue<Double_t> RecAng2(Reader2, "RecAng");
       TTreeReaderValue<Double_t> ScaAng2(Reader2, "ScaAng");
       TTreeReaderValue<Double_t> RecEner2(Reader2, "RecEner");
       TTreeReaderValue<Double_t> ScaEner2(Reader2, "ScaEner");
       TTreeReaderValue<Double_t> RecRange2(Reader2, "RecRange");
       TTreeReaderValue<Double_t> ScaRange2(Reader2, "ScaRange");
       TTreeReaderValue<Double_t> RecRad2(Reader2, "RecRad");
       TTreeReaderValue<Double_t> ScaRad2(Reader2, "ScaRad");
       TTreeReaderValue<Double_t> VertexPos2(Reader2, "VertexPos");
       TTreeReaderValue<Double_t> BeamEner2(Reader2, "BeamEner");
       TTreeReaderValue<Double_t> BeamRange2(Reader2, "BeamRange");

       while (Reader1.Next()) {

         Angle_sca_vs_Angle_rec_1->Fill(*ScaAng1,*RecAng1);
         Range_vs_Angle_rec_1->Fill(*RecRange1,*RecAng1);
         Radius_vs_Vertex_rec_1->Fill(*RecRad1,*VertexPos1);
         Ener_vs_Vertex_rec_1->Fill(*RecEner1,*VertexPos1);
         Ener_vs_Angle_rec_1->Fill(*RecAng1,*RecEner1);
         Ener_vs_Angle_sca_1->Fill(*ScaAng1,*ScaEner1);

       }

       while (Reader2.Next()) {

         Angle_sca_vs_Angle_rec_2->Fill(*ScaAng2,*RecAng2);
         Range_vs_Angle_rec_2->Fill(*RecRange2,*RecAng2);
         Radius_vs_Vertex_rec_2->Fill(*RecRad2,*VertexPos2);
         Ener_vs_Vertex_rec_2->Fill(*RecEner2,*VertexPos2);
         Ener_vs_Angle_rec_2->Fill(*RecAng2,*RecEner2);
         Ener_vs_Angle_sca_2->Fill(*ScaAng2,*ScaEner2);

       }

       /*TTree* tree1 = (TTree*) file1 -> Get("treeOut");
       if(!tree1) std::cout<<" No tree found! "<<std::endl;
       tree1 -> Branch("RecAng", &RecAng,"RecAng/D");
       Int_t nEvents1 = tree1 -> GetEntriesFast();

            for(Int_t i=0;i<nEvents1;i++){
              tree1->GetEntry(i);
              std::cout<<RecAng<<std::endl;

            }*/

        TCanvas *c1 = new TCanvas();
        c1->Draw();
        c1->Divide(2,2);
        c1->cd(1);
        Angle_sca_vs_Angle_rec_1->SetMarkerStyle(20);
        Angle_sca_vs_Angle_rec_1->SetMarkerSize(0.5);
        Angle_sca_vs_Angle_rec_1->SetMarkerColor(2);
        Angle_sca_vs_Angle_rec_1->Draw("");
        Angle_sca_vs_Angle_rec_2->SetMarkerStyle(20);
        Angle_sca_vs_Angle_rec_2->SetMarkerSize(0.5);
        Angle_sca_vs_Angle_rec_2->Draw("SAMES");
        c1->cd(2);
        Ener_vs_Vertex_rec_1->SetMarkerStyle(20);
        Ener_vs_Vertex_rec_1->SetMarkerSize(0.5);
        Ener_vs_Vertex_rec_1->SetMarkerColor(2);
        Ener_vs_Vertex_rec_1->Draw("");
        Ener_vs_Vertex_rec_2->SetMarkerStyle(20);
        Ener_vs_Vertex_rec_2->SetMarkerSize(0.5);
        Ener_vs_Vertex_rec_2->Draw("SAMES");
        c1->cd(3);
        Range_vs_Angle_rec_1->SetMarkerStyle(20);
        Range_vs_Angle_rec_1->SetMarkerSize(0.5);
        Range_vs_Angle_rec_1->SetMarkerColor(2);
        Range_vs_Angle_rec_1->Draw("");
        Range_vs_Angle_rec_2->SetMarkerStyle(20);
        Range_vs_Angle_rec_2->SetMarkerSize(0.5);
        Range_vs_Angle_rec_2->Draw("SAMES");
        c1->cd(4);
        Radius_vs_Vertex_rec_1->SetMarkerStyle(20);
        Radius_vs_Vertex_rec_1->SetMarkerSize(0.5);
        Radius_vs_Vertex_rec_1->SetMarkerColor(2);
        Radius_vs_Vertex_rec_1->Draw("");
        Radius_vs_Vertex_rec_2->SetMarkerStyle(20);
        Radius_vs_Vertex_rec_2->SetMarkerSize(0.5);
        Radius_vs_Vertex_rec_2->Draw("SAMES");

        TCanvas *c2 = new TCanvas();
        c2->Draw();
        c2->Divide(2,2);
        c2->cd(1);
        Ener_vs_Angle_rec_1->SetMarkerStyle(20);
        Ener_vs_Angle_rec_1->SetMarkerSize(0.5);
        Ener_vs_Angle_rec_1->SetMarkerColor(2);
        Ener_vs_Angle_rec_1->Draw("zcol");
        Ener_vs_Angle_rec_2->SetMarkerStyle(20);
        Ener_vs_Angle_rec_2->SetMarkerSize(0.5);
        Ener_vs_Angle_rec_2->Draw("zcolSAMES");
        c2->cd(2);
        Ener_vs_Angle_sca_1->SetMarkerStyle(20);
        Ener_vs_Angle_sca_1->SetMarkerSize(0.5);
        Ener_vs_Angle_sca_1->SetMarkerColor(2);
        Ener_vs_Angle_sca_1->Draw("zcol");
        Ener_vs_Angle_sca_2->SetMarkerStyle(20);
        Ener_vs_Angle_sca_2->SetMarkerSize(0.5);
        Ener_vs_Angle_sca_2->Draw("zcolSAMES");






}
