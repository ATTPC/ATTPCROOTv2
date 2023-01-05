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


void Analysis(TString isotopeName,Int_t num_ev=100000)
{
   
    std::string fileName = "test";
    
    

    TString mcFileNameHead = "./data/DeGAi_";
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
            

                
         }
          if (energyLoss != 0.0) {
          
            Energy_loss->Fill(energyLoss);
        }
        }
         

        
    

    // Calculate photopeak efficency
        std::vector<double> energies;
    if (isotopeName == "Co60") {
            energies = {1.173, 1.332}; // Cobalt-60 energies
        } else if (isotopeName == "Ce137") {
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
            double lowerBound = energy - 0.02;
            double upperBound = energy + 0.02;
            for (Int_t bin = energyLossHistogram->GetXaxis()->FindBin(lowerBound);
                 bin <= energyLossHistogram->GetXaxis()->FindBin(upperBound); bin++) {
                photopeakCount += energyLossHistogram->GetBinContent(bin);
            }
}
    Double_t photopeakEfficency = (PhotopeakCount / num_ev) * 100.0;
    Double_t Err = (TMath::Sqrt(PhotopeakCount)/PhotopeakCount) *photopeakEfficency;

    std::cout << "Total number of events : " << num_ev << std::endl;
    std::cout << "Number of events in photopeak : " << PhotopeakCount << std::endl;
    std::cout << "Photopeak Efficency : " << photopeakEfficency << "%" << std::endl;
    std::cout<<"Error: " << Err << std::endl;
}

