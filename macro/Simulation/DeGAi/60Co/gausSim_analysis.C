#include <map>
#include <set>
#include <TMath.h>
#include <TString.h>

void gausSim_analysis(Int_t num_ev = 100000)
{
    // File names
    TString mcFileNameHead = "./DeGAi";
    TString mcFileNameTail = ".root";
    TString mcFileName = mcFileNameHead + mcFileNameTail;
    TString outFileNameHead = "./DeGAiana";
    TString outFileNameTail = ".root";
    TString outFileName = outFileNameHead + outFileNameTail;

    // Create objects
    AtMCPoint* point = new AtMCPoint();
    TClonesArray* pointArray = 0;

    // Open file and get tree
    TFile* file = new TFile(mcFileName.Data(), "READ");
    TTree* tree = (TTree*)file->Get("cbmsim");

    tree->SetBranchAddress("AtMCPoint", &pointArray);
    Int_t nEvents = tree->GetEntriesFast();

    if (nEvents > num_ev)
        nEvents = num_ev;

    // Histograms
    TH1D* Energy_loss = new TH1D("Energy_loss", "Energy_loss", 10000, 0, 10);
    TCanvas* c1 = new TCanvas();
    c1->Draw();
    Double_t Count = 0.0;

    std::set<TString> uniqueCrystals; // Set to store unique crystal volume names
    std::map<TString, Int_t> crystalCount; // Tally for crystal volume names

    for (Int_t iEvent = 0; iEvent < nEvents; iEvent++)
    {
        tree->GetEvent(iEvent);
        Int_t n = pointArray->GetEntries();

        double energyLoss = 0.0;

        for (Int_t i = 0; i < n; i++)
        {
            point = (AtMCPoint*)pointArray->At(i);
            auto VolName = point->GetVolName();
            auto trackID = point->GetTrackID();

            if (VolName.Contains("Crystal_"))
            {
                uniqueCrystals.insert(VolName);
                crystalCount[VolName]++;

                Float_t fResolutionGe = .30;
                Double_t inputEnergy = point->GetEnergyLoss();
                Double_t randomIs = gRandom->Gaus(0, inputEnergy * fResolutionGe * 1000 / (235 * sqrt(inputEnergy * 1000)));
                energyLoss += (inputEnergy + randomIs / 1000) * 1000; // MeV
                Count += 1;
            }
        }

        if (energyLoss != 0.0)
        {
            Energy_loss->Fill(energyLoss);
        }
    }

    // Print crystal volume names and tallies
    std::cout << "Crystal Volume Names and Number of hits:" << std::endl;
    for (const auto& crystal : uniqueCrystals)
    {
        std::cout << crystal << ": " << crystalCount[crystal] << std::endl;
    }

    // Bin content for 60Co gamma-ray energies
    std::map<Double_t, Double_t> binContent;

    // Energy of 60Co gamma-ray lines (1.17 MeV and 1.33 MeV)
    Double_t energy1 = 1.17;
    Double_t energy2 = 1.33;

    Int_t bin1 = Energy_loss->GetXaxis()->FindBin(energy1);
    Int_t bin2 = Energy_loss->GetXaxis()->FindBin(energy2);

    binContent[energy1] = Energy_loss->GetBinContent(bin1);
    binContent[energy2] = Energy_loss->GetBinContent(bin2);



    Double_t PhotopeakEfficency = ((binContent[energy1] + binContent[energy2]) /Count)*100; ;
    std::cout << "Total number of events: " << Count<< "Total bin: "<< binContent[energy1] +binContent[energy2] << std::endl;
    std::cout << "Photopeak Efficency: " << PhotopeakEfficency << std::endl;

    c1->cd(1);
    Energy_loss->Draw();
}
