#include <iostream>
#include <string>
#include <map>
#include <vector>
#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TClonesArray.h"
#include "TH1D.h"
#include "TCanvas.h"
#include "TRandom3.h"
#include "TMath.h"


void Simp_gamma_analysis(Double_t momentum,Int_t num_ev = 50000)
{
   
    std::string fileName = "test";
    
    

    TString mcFileNameHead = "./DeGAi_";
    TString mcFileNameTail = ".root";
    TString mcFileName = mcFileNameHead + TString(fileName.c_str()) + mcFileNameTail;
    TString outFileNameHead = "./data/DeGAiana";
    TString outFileNameTail = ".root";
    TString outFileName = outFileNameHead + outFileNameTail;

    AtMCPoint* point = new AtMCPoint();
    TClonesArray* pointArray = 0;

    TFile* file = new TFile(mcFileName.Data(), "READ");
    TTree* tree = (TTree*)file->Get("cbmsim");

    tree->SetBranchAddress("AtMCPoint", &pointArray);
    Int_t nEvents = tree->GetEntriesFast();

    if (nEvents > num_ev)
        nEvents = num_ev;

    // Histograms
    Int_t Bins = 11000;
    Int_t MeV = 11;
    TH1D* Energy_loss = new TH1D("Energy_loss", "Photopeak Efficiency: ", Bins, 0, MeV);

    Double_t Count = 0.0;
    Double_t PhotopeakCount = 0.0;

    TRandom3* gRandom = new TRandom3();

    for (Int_t iEvent = 0; iEvent < nEvents; iEvent++)
    {
        tree->GetEvent(iEvent);
        Int_t n = pointArray->GetEntries();
        Double_t energyLoss = 0.0;

        for (Int_t i = 0; i < n; i++) {

            point = (AtMCPoint*)pointArray->At(i);
            auto VolName = point->GetVolName();

            auto trackID = point->GetTrackID();
            if (VolName.Contains("Crystal_")) {

                // Gaussian smearing
                Float_t fResolutionGe = .30;
                Double_t inputEnergy = point->GetEnergyLoss();
                Double_t randomIs = gRandom->Gaus(0, inputEnergy * fResolutionGe * 1000 / (235 * sqrt(inputEnergy * 1000)));
                energyLoss += (inputEnergy + randomIs / 1000) * 1000; // MeV
                Count++;

            

                // Check if energyLoss is within the photopeak range for specific isotopes
                if (energyLoss >= momentum * 0.95 && energyLoss <= momentum * 1.05) {
                    PhotopeakCount++;
                }
         }
}
        
    }

    // Calculate photopeak efficiency
    Double_t photopeakEfficiency = (PhotopeakCount / Count) * 100.0;
    Double_t Err = (TMath::Sqrt(PhotopeakCount)/Count) *100.0;


    std::cout << "Photopeak Efficiency : " << photopeakEfficiency << "%" << std::endl;
    std::cout<<"Error:" << Err<< std::endl;
}

