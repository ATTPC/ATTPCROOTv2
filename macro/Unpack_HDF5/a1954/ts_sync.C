#include <TMath.h>
#include <TGraph2D.h>
#include <TRandom2.h>
#include <TStyle.h>
#include <TCanvas.h>
#include <TF2.h>
#include <TH1.h>
#include <Math/Functor.h>
#include <TPolyLine3D.h>
#include <Math/Vector3D.h>
#include <Fit/Fitter.h>
#include "AtTrackTransformer.h"
#include <cassert>

void ts_sync() {
    // Define the file pairs
    std::vector<std::pair<std::string, std::string>> filepairs;
    filepairs.push_back(std::make_pair("run_0079.root", "run_0079_FRIB_sorted.root"));
    //filepairs.push_back(std::make_pair("run_0105.root", "run_0105_FRIB_sorted.root"));

    for (const auto& filepair : filepairs) {
        // Open the ROOT files
        TFile *file1 = TFile::Open(filepair.first.c_str(), "UPDATE");
        TFile *file2 = TFile::Open(filepair.second.c_str(), "UPDATE");

        if (!file1 || !file2) {
            std::cerr << "Failed to open one of the files: " << filepair.first << " or " << filepair.second << std::endl;
            continue;
        }

        // Get the trees from the files
        TTree *tree1 = (TTree*)file1->Get("cbmsim");
        TTree *tree2 = (TTree*)file2->Get("FRIB_output_tree");

        if (!tree1 || !tree2) {
            std::cerr << "Failed to get the trees from the files: " << filepair.first << " or " << filepair.second << std::endl;
            file1->Close();
            file2->Close();
            continue;
        }

        // Get the branches for the timestamps
        TBranch *branch2 = tree2->GetBranch("timestamp");

        if (!branch2) {
            std::cerr << "Failed to get the timestamp branches from the trees: " << filepair.first << " or " << filepair.second << std::endl;
            file1->Close();
            file2->Close();
            continue;
        }

        // Variables to hold the timestamps
        Long64_t timestamp1, timestamp2, timestamp1_next;
        branch2->SetAddress(&timestamp2);
        Long64_t tstampreference1 = 0;
        Long64_t tstampreference2 = 0;

        // Vectors to store timestamp differences
       std::vector<std::tuple<Long64_t, Long64_t, Long64_t>> timestampDiffs1, timestampDiffs2;
       std::vector<double> remove_index_1, remove_index_2;

        // Loop over the events in the first tree and store the timestamp differences and corresponding timestamps
        TClonesArray *eventArray = nullptr;
        tree1->SetBranchAddress("AtEventH", &eventArray);

       
Long64_t j=0;

    for (Long64_t i = 0; i < tree1->GetEntries() - 1; i++){
        tree1->GetEntry(i);
        AtEvent *event1 = (AtEvent*)eventArray->At(0);
        timestamp1 = event1->GetTimestamp(1);
        tree1->GetEntry(i+1);
        AtEvent *event2 = (AtEvent*)eventArray->At(0);
        timestamp1_next = event2->GetTimestamp(1);
        branch2->GetEntry(j);
        Long64_t ts2 = timestamp2;
        branch2->GetEntry(j+1);
        Long64_t ts2_next = timestamp2;
        
        
        std::cout << "Timestamp1: " << timestamp1 << " Timestamp1_next: " << timestamp1_next << std::endl;
        std::cout << "Timestamp2: " << ts2 << " Timestamp2_next: " << ts2_next << std::endl;

        Long64_t timestampDiff1 = timestamp1 - timestamp1_next;                
            
        Long64_t timestampDiff2 = ts2 - ts2_next;

        if (std::abs(timestampDiff1 - timestampDiff2) > 5 ) {
            remove_index_1.push_back(i+1);
            remove_index_2.push_back(j+1);
            remove_index_2.push_back(j+2);
            j = j + 3;
            i = i + 1;   
        } else {
            j = j + 1;
        }
    }

    std::cout << "Number of events to be removed from tree1: " << remove_index_1.size() << std::endl;
    std::cout << "Number of events to be removed from tree2: " << remove_index_2.size() << std::endl;
        // Sets to hold the timestamps of events to be kept

        // Create new trees to keep the matching events
        TTree *newTree1 = tree1->CloneTree(0);
        TTree *newTree2 = tree2->CloneTree(0);

        
        

        // Close the files
        file1->Close();
        file2->Close();
    }
}