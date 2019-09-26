//  -------------------------------------------------------------------------
//
//   ----- General Macro for ATTPCRootv2 Analysis results
//         Author: Hector Alvarez <hector.alvarez@usc.es>
//         Comments:
//			Checks the variables
//
//  -------------------------------------------------------------------------
//
//   Usage:
//     > root -l 
//     root[0] .L AT.C
//     root[1] AT(0)    ... or AT(number_of_event) 
//
//  -------------------------------------------------------------------------

void AT(Int_t eventNum=1) {
  
  char title1[250];
  char name[250];
  
  sprintf(title1,"%s","output_digi.root");
  TFile *file1 = TFile::Open(title1);
  
  gROOT->SetStyle("Default");
  gStyle->SetOptStat(0);
  gStyle->SetOptFit(0);
  
  //HISTOGRAMS DEFINITION
  TCanvas *c1 = new TCanvas();
  c1->Divide(2);
  
  TH1F** h1_ADC;
  TH2F* h2_POS = new TH2F("pos","pos",600,-300,300,600,-300,300);
  
  TTree* ATTree = (TTree*)file1->Get("cbmsim");
  
  //Raw Hits (input)
  TClonesArray* ATRawEventCA;
  ATRawEvent** rawEvent;
  ATRawEventCA = new TClonesArray("ATRawEvent",1);
  TBranch *branchRawEvent = ATTree->GetBranch("ATRawEvent");
  branchRawEvent->SetAddress(&ATRawEventCA);
  
  std::vector<ATPad>* pads;
  ATPad * aPad;
  
  Long64_t nevents = ATTree->GetEntries();
  
  Int_t ATRawEventPerEvent=0;
  Int_t padsFired=0;
  
  for(Int_t i=0;i<nevents;i++){
    if(i%1000 == 0) printf("Event:%i\n",i);
    ATRawEventCA->Clear();
    if(i==eventNum){
      ATTree->GetEvent(i);
      ATRawEventPerEvent = ATRawEventCA->GetEntries();
      
      if(ATRawEventPerEvent>0) {
	rawEvent = new ATRawEvent*[ATRawEventPerEvent];
	for(Int_t j=0;j<ATRawEventPerEvent;j++){
	  rawEvent[j] = new ATRawEvent;
	  rawEvent[j] = (ATRawEvent*) ATRawEventCA->At(j);
	}
      }
      
      for(Int_t j=0;j<ATRawEventPerEvent;j++){
	padsFired = rawEvent[j]->GetNumPads();
	pads = rawEvent[j]->GetPads();
	cout << padsFired <<" pads fired."<<endl;
	h1_ADC = new TH1F*[padsFired];
	Int_t counter=0;
	for(std::vector<ATPad>::iterator it = pads->begin(); it != pads->end(); ++it){
	  //if((*it).GetPadNum()==147){
          sprintf(name,"h%i",(*it).GetPadNum());
          h1_ADC[counter] = new TH1F(name,name,512,0,512);
          aPad = &(*it);
          h2_POS->Fill(aPad->GetPadXCoord(),aPad->GetPadYCoord());
          for(Int_t bin=0;bin<512;bin++)
	    h1_ADC[counter]->SetBinContent(bin,aPad->GetADC(bin));
          c1->cd(1);
          h1_ADC[counter]->SetMaximum(4096);
          gStyle->SetHistLineColor(counter);
          if(counter==0) h1_ADC[counter]->Draw();
          else h1_ADC[counter]->Draw("SAME");
          c1->cd(2);
          h2_POS->Draw("");
          counter++;
	  //}
	}
      }
    }
  }
}
