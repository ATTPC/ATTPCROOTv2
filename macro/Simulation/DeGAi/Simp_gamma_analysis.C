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


void Simp_gamma_analysis(Double_t momentum,Int_t num_ev)
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
    TH1D* Energy_loss = new TH1D("Energy_loss", "Photopeak Efficency: ", Bins, 0, MeV);

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
            if (VolName.Contains("Crystal_") && !VolName.Contains("41")) {

                // Gaussian smearing
                //Float_t fResolutionGe = .30;
                Double_t inputEnergy = point->GetEnergyLoss();
                //Double_t randomIs = gRandom->Gaus(0, inputEnergy * fResolutionGe * 1000 / (235 * sqrt(inputEnergy * 1000)));
                //energyLoss += (inputEnergy + randomIs / 1000) * 1000; // MeV
                energyLoss += inputEnergy * 1000;
                Count++;

            

                
         }
         else if(VolName.Contains("41")) {
                Count++;
            }
}
        
    }

    // Calculate photopeak efficency
    PhotopeakCount = Energy_loss->GetXaxis()->FindBin(momentum);
    Double_t photopeakEfficency = (PhotopeakCount / Count) * 100.0;
    Double_t Err = (TMath::Sqrt(PhotopeakCount)/PhotopeakCount) *100.0;

    std::cout << "Total number of events : " << Count << std::endl;
    std::cout << "Number of events in photopeak : " << PhotopeakCount << std::endl;
    std::cout << "Photopeak Efficency : " << photopeakEfficency << "%" << std::endl;
    std::cout<<"Error:" << Err<< std::endl;
}

