#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TClonesArray.h"

#include <iostream>
#include <fstream>

void run_sim_ana()
{
    
    TH2D *Eloss_vs_Range_Sca = new TH2D("Eloss_vs_Range_Sca","ELoss_vs_Range_Sca",100,0,1000,300,0,300);
    Eloss_vs_Range_Sca->SetMarkerStyle(20);
    Eloss_vs_Range_Sca->SetMarkerSize(0.5);
    
    TH2D *Eloss_vs_Range_Rec = new TH2D("Eloss_vs_Range_Rec","ELoss_vs_Range_Rec",1000,0,1000,1000,0,10);
    Eloss_vs_Range_Rec->SetMarkerStyle(20);
    Eloss_vs_Range_Rec->SetMarkerSize(0.5);
    Eloss_vs_Range_Rec->SetMarkerColor(2);
    
    TH1D *ELossRatio = new TH1D("ElossRatio","ELossRatio",1000,0,1000);
    
    TH1D *ICELoss = new TH1D("ICELoss","ICELoss",1000,0,100);
    
    
    
    TCanvas *c1 = new TCanvas();
    c1->Divide(2,2);
    c1->Draw();
    
    
    TString mcFileNameHead = "data/attpcsim";
    TString mcFileNameTail = ".root";
    TString mcFileName     = mcFileNameHead + mcFileNameTail;
    std:cout << " Analysis of simulation file  " << mcFileName << endl;
    
    AtTpcPoint* point;
    TClonesArray *pointArray;
    
    TFile* file = new TFile(mcFileName.Data(),"READ");
    TTree* tree = (TTree*) file -> Get("cbmsim");
    tree -> SetBranchAddress("AtTpcPoint", &pointArray);
    Int_t nEvents = tree -> GetEntriesFast();
    
    
    
    for(Int_t iEvent=0; iEvent<nEvents; iEvent++)
    {
        Double_t energyLoss_sca=0.0;
        Double_t range_sca=0.0;
        Double_t energyLoss_rec=0.0;
        Double_t range_rec=0.0;
        Double_t BeamEnergyLoss_IC=0.0;
        TString VolName;
        tree -> GetEntry(iEvent);
        Int_t n = pointArray -> GetEntries();
        std::cout<<" Event Number : "<<iEvent<<std::endl;
        for(Int_t i=0; i<n; i++) {
            
            point = (AtTpcPoint*) pointArray -> At(i);
            VolName=point->GetVolName();
            //std::cout<<" Volume Name : "<<VolName<<std::endl;
            Int_t trackID = point -> GetTrackID();

             //std::cout<<" Track ID : "<<trackID<<std::endl;
             //std::cout<<" Point number : "<<i<<std::endl;
            if(trackID==0 && VolName=="IC_drift_volume"){
                
                BeamEnergyLoss_IC+=( point -> GetEnergyLoss() )*1000;//MeV
                
            }
            
            if(trackID==2 && VolName=="drift_volume"){
                range_rec = point -> GetLength()*10; //mm
                energyLoss_rec+=( point -> GetEnergyLoss() )*1000;//MeV
                //std::cout<<" Track ID : "<<trackID<<std::endl;
                
                //std::cout<<" Point number : "<<i<<std::endl;
                //std::cout<<" Event Number : "<<iEvent<<std::endl;
                //std::cout<<" Range_rec : "<<range_rec<<std::endl;
                //std::cout<<" energyLoss_rec : "<<energyLoss_rec<<std::endl;
                
            }//TrackID == 2
                if(trackID==1 && VolName=="drift_volume"){
                    range_sca = point -> GetLength()*10; //mm
                    energyLoss_sca+=( point -> GetEnergyLoss() )*1000;//MeV
                   // std::cout<<" Track ID : "<<trackID<<std::endl;
                   // std::cout<<" Range_sca : "<<range_sca<<std::endl;
                   // std::cout<<" energyLoss_sca : "<<energyLoss_sca<<std::endl;
                }//TrackID == 1
            
            
        }//n number of points
        
        
        if(iEvent%2!=0){
            std::cout<<" Range_rec : "<<range_rec<<std::endl;
            std::cout<<" energyLoss_rec : "<<energyLoss_rec<<std::endl;
            Eloss_vs_Range_Rec->Fill(range_rec,energyLoss_rec);
            

            std::cout<<" Range_sca : "<<range_sca<<std::endl;
            std::cout<<" energyLoss_sca : "<<energyLoss_sca<<std::endl;
            Eloss_vs_Range_Sca->Fill(range_sca,energyLoss_sca);
            ELossRatio->Fill(energyLoss_sca/energyLoss_rec);
            
        }else if(iEvent%2==0) ICELoss->Fill(BeamEnergyLoss_IC);
        
    }// Events
    
    c1->cd(1);
    Eloss_vs_Range_Sca->Draw("scat");
    c1->cd(2);
    Eloss_vs_Range_Rec->Draw("scat");
    c1->cd(3);
    ELossRatio->Draw();
    c1->cd(4);
    ICELoss->Draw();
    
    
}