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

bool doesNotEqualAny(const std::string& str, const std::vector<std::string>& values) {
    for (const auto& value : values) {
        if (str == value) {
            return false;  // String matches a value in the list
        }
    }
    return true;  // String does not match any value in the list
}

void gamma_analysis(Int_t num_ev = 50000)
{
    // Ask user for file name
    std::cout << "Enter the file name for analysis: ";
    std::string fileName;
    std::cin >> fileName;
    Double_t momentum;
    std::vector<std::string> isotopes = { "60Co", "173Cs", "22Na" };
    if (doesNotEqualAny(fileName, isotopes)) {

        std::cout << "momentum: (MeV)";
        std::cin >> momentum;
    }

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
    TCanvas* c1 = new TCanvas();
    c1->Draw();

    Double_t Count = 0.0;
    Double_t PhotopeakCount = 0.0;
    std::map<std::string, int> crystalHits; // Map to store hit count for each VolName

    TRandom3* gRandom = new TRandom3();

    for (Int_t iEvent = 0; iEvent < nEvents; iEvent++)
    {
        tree->GetEvent(iEvent);
        Int_t n = pointArray->GetEntries();
        Double_t energyLoss = 0.0;
        Double_t GausenergyLoss = 0.0;

        for (Int_t i = 0; i < n; i++) {

            point = (AtMCPoint*)pointArray->At(i);
            auto VolName = point->GetVolName();

            auto trackID = point->GetTrackID();
            if (VolName.Contains("Crystal_")) {

                // Gaussian smearing
                Float_t fResolutionGe = .30;
                Double_t inputEnergy = point->GetEnergyLoss();
                Double_t randomIs = gRandom->Gaus(0, inputEnergy * fResolutionGe * 1000 / (235 * sqrt(inputEnergy * 1000)));
                GausenergyLoss += (inputEnergy + randomIs / 1000) * 1000; // MeV
                energyLoss += inputEnergy * 1000; // MeV
                
                Count++;

                // Update hit count for VolName
                crystalHits[VolName.Data()]++;

                
            }
        }

        if(energyLoss >= momentum-0.02 && energyLoss <= momentum+0.02){
            PhotopeakCount++;
        }
        if (GausenergyLoss != 0.0) {
            std::cout << "energyLoss: " << energyLoss << std::endl;
            Energy_loss->Fill(GausenergyLoss);
        }
    }

   
    Double_t photopeakEfficiency = ( PhotopeakCount/ num_ev) * 100.0;
    Double_t Err = (TMath::Sqrt(PhotopeakCount)/PhotopeakCount) *100.0;

    // Print the tally board for each crystal volume
    for (const auto& crystal : crystalHits) {
        std::cout << "VolName: " << crystal.first << " had " << crystal.second << " hits." << std::endl;
    }

    std::cout << "Photopeak Efficiency : " << photopeakEfficiency << "%" << std::endl;
    std::cout << PhotopeakCount << std::endl;
    std::cout <<"Error: "<< Err << std::endl;
    c1->cd(1);
    Energy_loss->SetTitle(Form("Photopeak Efficiency: Energy Loss Spectrum (%.2f%%)", photopeakEfficiency));
    Energy_loss->Draw();
}

