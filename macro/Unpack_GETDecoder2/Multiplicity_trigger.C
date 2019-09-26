
//Trigger Simulation for the ATTPCRoot Framework 
//Adapted script for root
//javier.diaz@usc.es
//07/24/2017

#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TH2.h"
#include "TH1.h"
#include "TH3.h"
#include "TClonesArray.h"
#include "TCanvas.h"
#include "TMath.h"
#include "TClonesArray.h"
#include "TStyle.h"
#include "TTreeReader.h"
#include "TChain.h"
#include "TFileCollection.h"
#include "TError.h"
#include "TMinuit.h"

#include <iostream>
#include <istream>
#include <limits>

void Multiplicity_trigger(){

	//This script contains a simulation of the trigger multiplicity for the AT-TPC

	//*************************************************************************************************
	//Setting all the parameters and variables for the simulation
	//*************************************************************************************************

	//Parameters, all of this parameters are found in the config file.

	double write_clock = 12.5e6; 			//The CoBo write clock in Hz
	double read_clock =25e6;			//The CoBo read clock in Hz
	double master_clock =100e6;			//The CoBo Global Master Clock in Hz
	double pad_thresh_MSB = 0;			//Global Threshold Value
	double pad_thresh_LSB = 0;			//LSB Threshold Value
	double trigger_signal_width = 235e-9; 		//Width of the trigger signal pulse
	double trigger_discriminator_fraction = 0.175;	//as a decimal, not a percent
	double multiplicity_threshold = 14000;		//Certain threshold for the multiplicity	
	double multiplicity_window = 300;		//The width of the time window
	double trigger_height = 48;			//The width of the time window
	double time_factor =  read_clock/write_clock;
	int Cobo_rate =1; 				//Cobo sampling rate 

	//*************************************************************************************************
	//Calculation of the pad threshold in ADC time buckets.
	//First, we transform what is written in the config file for the trigger threshold for each channel to a more useful value.
	//*************************************************************************************************

	int trigger_width = trigger_signal_width * write_clock;
	double pad_threshold = ((pad_thresh_MSB*pow(2,4)  + pad_thresh_LSB)/128)*trigger_discriminator_fraction*4096;
	double time_window = multiplicity_window / read_clock*write_clock;
	std::cerr << "Setting the pad threshold -> " <<pad_threshold << std::endl;
	std::cerr << "Setting multiplicity threshold -> " <<multiplicity_threshold <<std::endl;
	std::cerr << "Setting the sliding time window -> " <<time_window  << " (units of time buckets)" <<std::endl;
	
	//Some variables definition.

	int tbIdx;			int cobo;
	int CoboNumArray[10240];	int PadNumArray[10241];
	double multiplicity[512];	double Mutant_multiplicity;	double MuTanT_multiplicity[1000];
            	
	TMatrixD triggerSignal(10,512);// The trigger signal is a vector with 522 positions. The first 10 one will be filled with CoBo number and the rest will be the trigger signal. [(0,10),(0,512)]
	TMatrixD result(10,512);

	int count =0.0;
	int MaxRawADC = 0.0;
	vector<TH2I *> hRawPulse;
	vector<TH2I> CoboNumber;
	
	int minIdx;	int maxIdx;	int accum;
	Bool_t trigger;

	//*************************************************************************************************
	//Only for Root users. Importing data files and root files
	//First, we import a map of the pad number, cobos etc.
	//Then, we read the root file.
	//*************************************************************************************************
		
	//Importing map from dat file
  	TString dir = gSystem->UnixPathName(__FILE__);
   	dir.ReplaceAll("Multiplicity_trigger.C","");
   	dir.ReplaceAll("/./","/");
   	ifstream in;
   	in.open(Form("%sLookup20150611.dat",dir.Data()));
	TNtuple *ntuple = new TNtuple("ntuple","data from ascii file","CoboNum:asad:aget:channel:pad");

  	Float_t CoboNum, asad, aget, channel, pad;
   	Int_t nlines = 0;

	 while (1) {
     	 in >> CoboNum >> asad >> aget >> channel >>pad;
     	 if (!in.good()) break;
     	 if (nlines < 5) printf("CoboNum=%8f, asad=%8f, aget=%8f, channel=%8f, pad=%8f\n",CoboNum, asad, aget, channel, pad);
	 CoboNumArray[nlines] = CoboNum;
	 PadNumArray[nlines] = pad;
     	 ntuple->Fill(CoboNum, asad, aget, channel, pad);
     	 nlines++;
  	 }
  	 printf(" found %d points\n",nlines);

  	 in.close();
		
	//We get the Event information from a real event to check the trigger.

	TH1D* h_PadNum = new TH1D("h_PadNum","h_PadNum",512,0,11000);
	TH1D* h_RawADC = new TH1D("h_RawADC","h_RawADC",512,0,5120);
	TH1D* h_Mult = new TH1D("h_Mult","Multiplicity",512,0,512);
	TH1D* h_trigger = new TH1D("h_trigger","Trigger Multiplicity Signal",512,0,512);
	TH1D* h_MuTant_Multiplicity= new TH1D("h_MuTant_Multiplicity","MuTANT Multiplicity Signal",512,0,20);
	TString FileNameHead = "run0218_output";
   	TString workdir = getenv("VMCWORKDIR");
   	TString FilePath = "/mnt/analysis/attpcroot/alpha_scattering/analyzedData/";
   	TString FileNameTail = ".root";
    	TString FileName     = FilePath + FileNameHead + FileNameTail;
   	std::cout<<" Opening File : "<<FileName.Data()<<std::endl;
   	TChain *chain = new TChain("cbmsim");
    	chain->Add(FileName);

	TClonesArray* fATRawEventArray= NULL;	
	chain->SetBranchAddress("ATRawEvent", &fATRawEventArray);
	TClonesArray* fATEventArray= NULL;	
	chain->SetBranchAddress("ATEventH", &fATEventArray);

	Int_t nentries = chain->GetEntries();


	//*************************************************************************************************
	//Only for Root users. Starting of the event readout.
	//*************************************************************************************************

	//LOOP over the number of EVENTS
  	
   	 for(int i=7 ; i < 8 ; i++){
		chain->GetEntry(i);

		ATEvent *fEvent;
		fEvent = (ATEvent*) fATEventArray->At(0);
		Int_t numHits = fEvent->GetNumHits();

		ATRawEvent *fRawEvent;
		fRawEvent = (ATRawEvent*) fATRawEventArray->At(0);
		Int_t numPads = fRawEvent->GetNumPads();

		MaxRawADC = 0;

		//Loop over the NUMBER of SIGNALS in the EVENT

		for(Int_t iHit=0; iHit < numHits; iHit++){
			ATHit fHit = fEvent ->GetHitArray()->at(iHit);
			Int_t PadNumHit = fHit.GetHitPadNum();

			Bool_t fValidPad;

			ATPad *fPad = fRawEvent -> GetPad(PadNumHit,fValidPad);
			Double_t *fRawAdc = fPad->GetADC();			
			Int_t fPadNum = fPad->GetPadNum();
			
			std::cout<< "Pad hit -> " << PadNumHit << std::endl;

			if (i==0 && iHit<100){hRawPulse.push_back( new TH2I(Form("hRawPulse_Pad%i",iHit),Form("hRawPulse_Pad%i",iHit),512,0,512,500,0,4100));}

			//LOOP over the TIME distribution of each signal of the EVENT

			for (int j = 0; j<512; j++){
				if(MaxRawADC <= fRawAdc[j]) MaxRawADC =fRawAdc[j];
				if (i == 0 && iHit<100){ hRawPulse.back()->Fill(j,fRawAdc[j]);}
				}

			  //**************************************************************************************************************************************
 			  //Calculation of the trigger signal for the given event.
 			  //Any time the input signal rises above the threshold value, a trigger signal is generated by the AGET for the corresponding channel.
  			  //**************************************************************************************************************************************

				double trigSignal[512];			//Empty Trigger Signal for each signal of the event
				tbIdx = 0;
				memset(trigSignal, 0, sizeof(trigSignal));
						
				while (tbIdx < 512){		
					if (fRawAdc[tbIdx] > pad_threshold) {
						for (Int_t ii = tbIdx; ii < min(tbIdx+trigger_width,512); ii++){trigSignal[ii] += trigger_height;}		
						tbIdx+= trigger_width;
						
						}

					else tbIdx +=1;
			
			
			}//While
	
				for(Int_t j = 0; j < 512; j++){
					
					if (trigSignal[j] > 0 ){
								
						
								for (Int_t l = 0; l <10240; l++){
									
									if (PadNumArray[l] == PadNumHit){cobo = CoboNumArray[l];}
										}//Loop to check with Cobo is hitting	
									triggerSignal(cobo,j) = trigSignal[j];
														
									}//Positive signal if
								
								}//3rd foor loop
					triggerSignal(cobo,0) = cobo;
					std::cout<< "CoBo hit -> " << cobo << std::endl;


			//*******************************************************************************************************************************
 			//Calculation of the multiplicity signals from the trigger signals.
  			//Each AGET sums the trigger signals from its channels and outputs this trigger multiplicity to the ADC.
  			//Then the CoBo sums this digitalized multiplicity signals from the AGETs and integrates this signal over a sliding time window.
  			//The input of this part will be the triggerSignals calculated before, and the output will be the multiplicity signals.
  			//********************************************************************************************************************************

				for (Int_t l =0; l < 512; l++){
					Int_t triggerWindow = l - time_window;
					minIdx = max(0,triggerWindow);
					maxIdx = l;

						for (Int_t ll =0; ll <10; ll++){
								accum = 0;
								if (ll == cobo ){result(ll,0)= cobo;
							for(Int_t jj = minIdx; jj < maxIdx; jj++){
								accum += triggerSignal(ll,jj);								
									}
									result(ll,l) = accum * time_factor;
									multiplicity[l] = result(ll,l);}
									
								}//ll for
			//*******************************************************************************************************************************
			//The CoBo Multiplicity is sent to the MuTanT.
			//Each CoBo Multiplicity is built every 40 ns, so we have time buckets of 40 ns.
			//The CoBo sends the multiplicity every 40 ns to the MuTanT.
			//*******************************************************************************************************************************				
								
							MuTanT_multiplicity[l] = multiplicity[l];
							h_MuTant_Multiplicity ->SetBinContent(l,MuTanT_multiplicity[l]);

							}//l for
			//*******************************************************************************************************************************
			//Just some plots
			//*******************************************************************************************************************************				
							
							if (cobo == 0){
									for (Int_t l =0; l < 512; l++){
										h_Mult->SetBinContent(l,result(0,l));
										h_trigger->SetBinContent(l,triggerSignal(0,l));
												}
											}

			
			//*************************************************************************************************
			//Determines if the given multiplicity signals rose above the multiplicity threshold in any CoBo.
			//*************************************************************************************************

				for (Int_t kk =0; kk < 10; kk++){
						if (kk == cobo){
							for (Int_t k =0; k < 512; k++){
								if(MuTanT_multiplicity[k] >  multiplicity_threshold) {trigger = kTRUE;}

									}

								}
							}
						std :: cout << "Max Multiplicity for the CoBo -> "<< TMath::MaxElement(512,multiplicity) << std::endl;
				
						}//Loop on entries of the event
				count++;
				std::cerr << "Event triggered if trigger =1 -> trigger = " << trigger << std::endl;
					
				}//loop on events
			
				std::cerr << "******************************************** " << std::endl;
				std::cerr << "Macro finished succesfully " << std::endl;
				std::cerr << "Number of Events -> = " << count << std::endl;
				std::cerr << "******************************************** " <<std::endl;
				
				TCanvas* can1 = new TCanvas("Signals","Signals");
				can1 ->Divide(2,1);
				can1->cd(2); h_MuTant_Multiplicity->Draw("E PMC");
				TLine *line2 = new TLine(0,multiplicity_threshold,20,multiplicity_threshold);
				line2->SetLineWidth(2);
				line2->SetLineStyle(1);
				line2->Draw();
				h_MuTant_Multiplicity->SetMarkerStyle(kFullCircle);
				can1->cd(1); h_Mult->Draw();
				TLine *line1 = new TLine(0,multiplicity_threshold,512,multiplicity_threshold);
				line1->SetLineWidth(2);
				line1->SetLineStyle(1);
				line1->Draw();

}

		




			
