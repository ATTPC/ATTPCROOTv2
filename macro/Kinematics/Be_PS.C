#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <sstream>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include <string>
#include <ctime>
#include <time.h>


// ROOT header
#include "TLorentzVector.h"
#include "TLorentzRotation.h"
#include "TVector3.h"
#include "TGraph.h"
#include "TCanvas.h"
#include "TH2F.h"
#include "TStyle.h"
#include "TGenPhaseSpace.h"
#include "TRandom3.h"
#include "TTree.h"
#include "TFile.h"

#include "Be_PS.h"

#define CLOCKS_PER_SEC  1000000l

using namespace std;

void Be_PS(double Energy = 35.0)//BeamEnergy in MeV
{


    TString file_name = Form("PSTree_10Be_%d.root",(int)Energy);
    //--------------- Tree definition ---------------//
    TFile *file = new TFile(file_name,"RECREATE");
    TTree *PSTree = new TTree("PSTree","Phase Space Tree");

    PSTree->Branch("KineticEnergy6He",&KineticEnergy6He,"KineticEnergy6He/D");
    PSTree->Branch("ThetaLab_6He",&ThetaLab_6He,"ThetaLab_6He/D");
    PSTree->Branch("KineticEnergy4He_1",&KineticEnergy4He_1,"KineticEnergy4He_1/D");
    PSTree->Branch("ThetaLab_4He_1",&ThetaLab_4He_1,"ThetaLab_4He_1/D");
    PSTree->Branch("KineticEnergy4He_2",&KineticEnergy4He_2,"KineticEnergy4He_2/D");
    PSTree->Branch("ThetaLab_4He_2",&ThetaLab_4He_2,"ThetaLab_4He_2/D");



    //-----------------------------------------------//

    Energy = Energy/1000.0; //GeV


    fImpulsionLab_beam = TVector3(0,0,sqrt(Energy*Energy + 2*Energy*mass_10Be));

    fEnergyImpulsionLab_beam = TLorentzVector(fImpulsionLab_beam,mass_10Be+Energy);

    fEnergyImpulsionLab_target = TLorentzVector(TVector3(0,0,0),mass_4He);

    fEnergyImpulsionLab_Total = fEnergyImpulsionLab_beam + fEnergyImpulsionLab_target;
    s = fEnergyImpulsionLab_Total.M2();
    beta = fEnergyImpulsionLab_Total.Beta();

    int nentries = 10000;
    clock_t begin = clock();
    clock_t end = begin;
    int good_entry = 0;

    cout << "********************************************" << endl;
    cout << "Energy  = " << Energy*1000 << " MeV" << endl;
    cout << "Energy available in the CM = " << sqrt(s)*1000 << " MeV" << endl;
    cout << "Beta = " << beta << endl;
    cout << "********************************************" << endl;


    for (int i=0;i<nentries;i++) {

        if (i%10000 == 0 && i!=0) 	{
            cout.precision(5);
            end = clock();
            double TimeElapsed = (end-begin) / CLOCKS_PER_SEC;
            double percent = (double)i/nentries ;
            double TimeToWait = (TimeElapsed/percent) - TimeElapsed;
            cout  << "                                                                                                "<< flush;
            cout	<< "\r Progression:" << percent*100 << " % \t | \t Remaining time : ~" <<  TimeToWait <<"s"<< flush;
        }
        else if (i==nentries-1) 	cout << "\r Progression:" << " 100% " <<endl;

        InitOutput();



        mass_1[0] = mass_6He;
        mass_1[1] = mass_4He;
	      mass_1[2] = mass_4He;

        std::cout<<" S : "<<s<<" Pow(M) "<<pow(mass_1[0]+mass_1[1]+mass_1[2],2)<<std::endl;

        if(s>pow(mass_6He+mass_4He+mass_4He,2)){


            event1.SetDecay(fEnergyImpulsionLab_Total, 3, mass_1);
            Double_t weight1 = event1.Generate();

            p6He    = event1.GetDecay(0);
            p4He_1  = event1.GetDecay(1);
	          p4He_2  = event1.GetDecay(2);

	    KineticEnergy6He  = (p6He->E() - mass_6He)*1000; //MeV
	    ThetaLab_6He     = p6He->Theta()*180./TMath::Pi();

	    KineticEnergy4He_1  = (p4He_1->E() - mass_4He)*1000; //MeV
	    ThetaLab_4He_1     = p4He_1->Theta()*180./TMath::Pi();

	    KineticEnergy4He_2  = (p4He_2->E() - mass_4He)*1000; //MeV
	    ThetaLab_4He_2     = p4He_2->Theta()*180./TMath::Pi();

            PSTree->Fill();


        }

    }

    file->Write();
    file->Close();

   // cout << "Percentage of event that satisifies the kinematics : " << (double)good_entry/nentries*100 << "%" << endl;

}



//************************************************************************
void InitOutput()
{

    KineticEnergy6He  = -100;
    KineticEnergy4He_1  = -100;
    KineticEnergy4He_2  = -100;
    ThetaLab_6He        = -100;
    ThetaLab_4He_1        = -100;
    ThetaLab_4He_2        = -100;

   /* KineticEnergyDelta  = -100;
    KineticEnergyN1     = -100;
    KineticEnergyN2     = -100;
    KineticEnergyPi_CM  = -100;
    KineticEnergyPi_Lab = -100;
    ThetaLab_Pi         = -100;
    ThetaCM_Pi          = -100;
    mass_delta          = -100;
    ThetaLab_N1         = -100;
    ThetaLab_N2         = -100;
    ThetaLab_Delta      = -100;*/

}
