#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TClonesArray.h"

#include <iostream>
#include <fstream>
#include <vector>


void run_sim_ana(Int_t num_ev=100000)
{
    //std::ofstream energyRecoilFile("alphaenergy_sim.txt");

    TH1F* vertex_dist = new TH1F("vertex_dist","vertex_dist",100,0,1000);

    TH1F* EnergyRecoil_h= new TH1F("EnergyRecoil","EnergyRecoil",1000,0,300);
    TH2D* dE_vs_E = new TH2D("dE_vs_E","dE_vs_E",200,0,0.2,200,0,0.2);
    TH2D* zpos_vs_angle = new TH2D("zpos_vs_angle","zpos_vs_angle",18,0,180,100,0,1000);
    TH2D* pEnergy_vs_angle = new TH2D("pEnergy_vs_angle","pEnergy_vs_angle",18,0,180,400,0,80);
    
   std::vector<TString> mcFileNames = {
        "rcnp_e565_10deg_300torr.root",
        "rcnp_e565_20deg_300torr.root",
        "rcnp_e565_30deg_300torr.root",
        "rcnp_e565_40deg_300torr.root",
        "rcnp_e565_50deg_300torr.root",
        "rcnp_e565_60deg_300torr.root",
        "rcnp_e565_70deg_300torr.root",
        "rcnp_e565_80deg_300torr.root",
        "rcnp_e565_90deg_300torr.root",
        "rcnp_e565_100deg_300torr.root",
        "rcnp_e565_110deg_300torr.root",
        "rcnp_e565_120deg_300torr.root",
        "rcnp_e565_130deg_300torr.root",
        "rcnp_e565_140deg_300torr.root",
        "rcnp_e565_150deg_300torr.root",
        "rcnp_e565_160deg_300torr.root",
        "rcnp_e565_170deg_300torr.root",
        "rcnp_e565_180deg_300torr.root", 

       /* "rcnp_e565_10deg_450torr.root",
        "rcnp_e565_20deg_450torr.root",
        "rcnp_e565_30deg_450torr.root",
        "rcnp_e565_40deg_450torr.root",
        "rcnp_e565_50deg_450torr.root",
        "rcnp_e565_60deg_450torr.root",
        "rcnp_e565_70deg_450torr.root",
        "rcnp_e565_80deg_450torr.root",
        "rcnp_e565_90deg_450torr.root",
        "rcnp_e565_100deg_450torr.root",
        "rcnp_e565_110deg_450torr.root",
        "rcnp_e565_120deg_450torr.root",
        "rcnp_e565_130deg_450torr.root",
        "rcnp_e565_140deg_450torr.root",
        "rcnp_e565_150deg_450torr.root",
        "rcnp_e565_160deg_450torr.root",
        "rcnp_e565_170deg_450torr.root",
        "rcnp_e565_180deg_450torr.root",*/

        /*"rcnp_e565_10deg_600torr.root",
        "rcnp_e565_20deg_600torr.root",
        "rcnp_e565_30deg_600torr.root",
        "rcnp_e565_40deg_600torr.root",
        "rcnp_e565_50deg_600torr.root",
        "rcnp_e565_60deg_600torr.root",
        "rcnp_e565_70deg_600torr.root",
        "rcnp_e565_80deg_600torr.root",
        "rcnp_e565_90deg_600torr.root",
        "rcnp_e565_100deg_600torr.root",
        "rcnp_e565_110deg_600torr.root",
        "rcnp_e565_120deg_600torr.root",
        "rcnp_e565_130deg_600torr.root",
        "rcnp_e565_140deg_600torr.root",
        "rcnp_e565_150deg_600torr.root",
        "rcnp_e565_160deg_600torr.root",
        "rcnp_e565_170deg_600torr.root",
        "rcnp_e565_180deg_600torr.root",*/



    };
for (const auto& mcFileName : mcFileNames) {
    TString name = mcFileName;
    Ssiz_t pos_deg = name.Index("deg");
    int angle = -1;
    if (pos_deg != kNPOS) {
        // Find the start of the number (scan backwards from pos_deg)
        int start = pos_deg - 1;
        while (start >= 0 && isdigit(name[start])) --start;
        TString angleStr = name(start+1, pos_deg - (start+1));
        angle = angleStr.Atoi();
        std::cout << "Angle extracted from file name: " << angle << std::endl;
    }
    std:cout << " Analysis of simulation file  " << mcFileName << endl;

    AtMCPoint* point = new AtMCPoint();
    AtMCPoint* pointSi = new AtMCPoint();
    AtMCPoint* pointMC = new AtMCPoint();
    AtMCPoint* point_forw = new AtMCPoint();
    AtMCPoint* point_back = new AtMCPoint();
    AtMCPoint* react_point = new AtMCPoint();
    Int_t nSiHits = 0;
    
    TClonesArray *pointArray=0;
    TClonesArray *pointSiArray=0;
    TClonesArray *pointMCArray=0;
    TFile* file = new TFile(mcFileName.Data(),"READ");
    TTree* tree = (TTree*) file -> Get("cbmsim");


    tree = (TTree*) file -> Get("cbmsim");
    //TBranch *branch = tree->GetBranch("AtTpcPoint");
    tree -> SetBranchAddress("AtTpcPoint", &pointArray);
    tree -> SetBranchAddress("MCTrack", &pointMCArray);
    tree -> SetBranchAddress("AtSiArrayPoint", &pointSiArray);
    Int_t nEvents = tree -> GetEntriesFast();

    Double_t vertex = 0.0;

    if(nEvents>num_ev) nEvents=num_ev;

    for(Int_t iEvent=0; iEvent<nEvents; iEvent++)
    {

        
        
        TString VolName;
        tree->GetEvent(iEvent);
        // tree -> GetEntry(iEvent);
        Int_t n = pointArray -> GetEntries();
        Int_t nSi = pointSiArray -> GetEntries();
        Int_t nMC = pointMCArray -> GetEntries();
        //std::cout<<" Event Number : "<<iEvent<<std::endl;
	   
        if (iEvent % 2 == 0) {
            react_point = (AtMCPoint*) pointArray -> At(n-1);
            vertex = react_point->GetZ()*10;
        }

        if (nSi != 0 && iEvent % 2 != 0) {
            Int_t nSi1Hits = 0;
            Int_t nSi2Hits = 0;
            Double_t Eloss1 = 0.0;
            Double_t Eloss2 = 0.0;
            
            for (Int_t i=0; i<nSi; i++) {
                pointSi = (AtMCPoint*) pointSiArray -> At(i);
                VolName = pointSi->GetVolName();
                Int_t trackID = pointSi -> GetTrackID();
                

                if (trackID == 0 && VolName.Contains("silicon1")) {
                    Eloss1 += pointSi->GetEnergyLoss();
                    nSi1Hits++;
                }

                if (trackID == 0 && VolName.Contains("silicon2")) {
                    Eloss2 += pointSi->GetEnergyLoss();
                    nSi2Hits++; 
                       
                }

                if (trackID == 0 && iEvent % 2 != 0 && VolName.Contains("silicon2")) {
                    vertex_dist->Fill(vertex);
                    zpos_vs_angle->Fill(angle, vertex);
                    for (Int_t imc = 0; imc < nMC; ++imc) {
                        AtMCTrack* mcTrack = (AtMCTrack*) pointMCArray->At(imc); // Use your MCTrack class here
                        //std::cout << "Processing MCTrack with PDG: " << mcTrack->GetMass() << std::endl;
                        if (mcTrack->GetMass() == 0.938272) {
                           
                            bool hasPointOutside3cm = false;
                            for (Int_t ipt = 0; ipt < n; ++ipt) {
                                AtMCPoint* pt = (AtMCPoint*) pointArray->At(ipt);
                                Int_t mcTrackID = pt->GetTrackID();
                                if (mcTrackID == 0) continue;
                                if (pt->GetTrackID() == mcTrackID) {
                                    double x = pt->GetX();
                                    double y = pt->GetY();
                                    if (std::sqrt(x*x + y*y) >= 3.0) {
                                        hasPointOutside3cm = true;
                                        break;
                                    }
                                }
                            }

   
                          if (hasPointOutside3cm == true) {  
                                double energy = mcTrack->GetEnergy();
                                pEnergy_vs_angle->Fill(angle-10, (energy - mcTrack->GetMass())*1000);
                                break; // Remove break if you expect more than one such particle per event
                            }
                        }
                    }


                    break;
                }

                

                
            }

            if(nSi1Hits > 0 && nSi2Hits > 0) {
                nSiHits++;
                dE_vs_E->Fill(Eloss2 + Eloss1, Eloss1);

            }

            
        }

        

        
        //std::cout<<" Number of points in Si : "<<nSi<<std::endl;
        
        
        /*for(Int_t i=0; i<nSi; i++) {

            pointSi = (AtMCPoint*) pointSiArray -> At(i);
            VolName=pointSi->GetVolName();
            //std::cout<<" Volume Name : "<<VolName<<std::endl;
            Int_t trackID = pointSi -> GetTrackID();
            //std::cout << trackID << std::endl;

            if(trackID==0 && VolName.Contains("silicon2") && iEvent % 2!=0){

            nSiHits++;

            }

        }*/

       // 
        //}else if(iEvent%2==0) ICELoss->Fill(BeamEnergyLoss_IC);
    }//number of events
}

    //zpos_vs_angle->Draw("colz");
    pEnergy_vs_angle->Draw("colz");


}