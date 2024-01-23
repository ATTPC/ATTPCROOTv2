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


void Spectrum(TString isotopeName = "Cs137",Int_t num_ev = 100000)
{   
    
    
    

    TString mcFileNameHead = "./data/DeGAi_";
    TString mcFileNameTail = ".root";
    TString mcFileName = mcFileNameHead + isotopeName + mcFileNameTail;
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

        
        if (GausenergyLoss != 0.0) {
        
            Energy_loss->Fill(GausenergyLoss);
        }
    }
    std::vector<double> energies;
    if (isotopeName == "Co60") {
            energies = {1.173, 1.332}; // Cobalt-60 energies
        } else if (isotopeName == "Cs137") {
            energies = {0.662}; // Cesium-137 energy
        } else if (isotopeName == "I131") {
            energies = {0.364}; // Iodine-131 energy
        } else if (isotopeName == "Te99") {
            energies = {0.140}; // Technetium-99m energy
        } else if (isotopeName == "Na22") {
            energies = {1.275}; // Sodium-22 energy
        } else if (isotopeName == "Am241") {
            energies = {0.060}; // Americium-241 energy
        } else if (isotopeName == "Th208") {
            energies = {0.583, 0.860, 2.614}; // Thallium-208 energies
        } else if (isotopeName == "U238") {
            energies = {0.186}; // Uranium-238 energy
        } else {
            std::cerr << "Isotope not recognized: " << isotopeName << std::endl;
            return;
        }
   
for (auto energy : energies) {
            double lowerBound = energy - 0.01;
            double upperBound = energy + 0.01;
            for (Int_t bin = Energy_loss->GetXaxis()->FindBin(lowerBound);
                 bin <= Energy_loss->GetXaxis()->FindBin(upperBound); bin++) {
                PhotopeakCount += Energy_loss->GetBinContent(bin);
            }
}
    Double_t photopeakEfficiency = (PhotopeakCount / num_ev) * 100.0;
    Double_t Err = (TMath::Sqrt(PhotopeakCount)/PhotopeakCount) *photopeakEfficiency;
    // Print the tally board for each crystal volume
    for (const auto& crystal : crystalHits) {
        std::cout << "VolName: " << crystal.first << " had " << crystal.second << " hits." << std::endl;
    }

    std::cout << "Photopeak Efficiency : " << photopeakEfficiency << "%" << std::endl;
    std::cout <<"Photopeak Count"<< PhotopeakCount << std::endl;
    std::cout <<"Error: "<< Err << std::endl;
    c1->cd(1);
    Energy_loss->SetTitle(Form("Photopeak Efficiency: Energy Loss Spectrum (%.2f%%)", photopeakEfficiency));
    Energy_loss->Draw();
}

