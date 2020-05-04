double GetMaximum(double *adc,int &maxTB)
{

	double max = 0;
	int TBmax = 0;

	for(int indTB=1;indTB<511;++indTB)
	{
	   // std::cout<<" TB "<<indTB<<" adc "<<adc[indTB]<<"\n";
	    if(adc[indTB]>max){
		 max   = adc[indTB];
                 TBmax = indTB; 
	    }
	}

	std::cout<<" max "<<max<<" TBmax "<<TBmax<<"\n";  

	maxTB=TBmax;           	   			 

	return max;

}

void scan_aux(){

    gSystem->Load("libATTPCReco.so"); 
    gSystem->Load("libAtTpcMap.so");

    FairRunAna* run = new FairRunAna(); //Forcing a dummy run

    TString workdir = getenv("VMCWORKDIR");
    TString FileNameHead = "output";
    TString FilePath = "/mnt/analysis/attpcroot/";
    TString FileNameTail = ".root";
    TString FileNameOut  = "_analysis";
    TString FileName     = FilePath + FileNameHead + FileNameTail;
    TString OutputFileName = FilePath + FileNameHead + FileNameOut + FileNameTail;

	TH1F* IC_spec = new TH1F("IC_spec","IC_spec",4096,0,4096);
	TH2F* Q_vs_T = new TH2F("Q_vs_T","Q_vs_T",4096,0,4096,512,0,511);
	TH1F* IC_waveform =  new TH1F("IC_waveform","IC_waveform",512,0,512);

    std::cout<<" Opening File : "<<FileName.Data()<<std::endl;
    TFile* file = new TFile(FileName.Data(),"READ");

		TTree* tree = (TTree*) file -> Get("cbmsim");
    		Int_t nEvents = tree -> GetEntries();
    		std::cout<<" Number of events : "<<nEvents<<std::endl;

    		TTreeReader Reader1("cbmsim", file);
    		//TTreeReaderValue<TClonesArray> raweventArray(Reader1, "ATRawEvent");
    		TTreeReaderValue<TClonesArray> eventArray(Reader1, "ATEventH");
    	

      for(Int_t i=0;i<nEvents;i++){
          //while (Reader1.Next()) {

              Reader1.Next();

              ATEvent* event = (ATEvent*) eventArray->At(0);
              //ATRawEvent *rawEvent = (ATRawEvent*) raweventArray->At(0);

	      if(event!=NULL)
              {
            
	              Int_t nHits = event->GetNumHits();
	              std::vector<ATHit>* hitArray = event->GetHitArray();
	              event->GetHitArrayObj();
	              std::cout<<" 	**** Event Number : "<<i<<" Event Q : "<<event->GetEventCharge()<<std::endl;
	      

	              //std::vector<ATPad> *padArray = rawEvent->GetPads();
	              std::vector<ATPad> *auxPadArray = event->GetAuxPadArray();


	              //std::cout<<" Number of pads : "<<padArray->size()<<" - Number of auxiliary pads : "<<auxPadArray->size()<<"\n";

	              for(auto auxpad : *auxPadArray)
	              {
			if(auxpad.GetAuxName().compare(std::string("unknown_4"))==0)
	              	{
	              		//std::cout<<" Auxiliary pad name "<<auxpad.GetAuxName()<<"\n";
				int maxTB=-100;
	              		Double_t *adc = auxpad.GetADC();
	              		float max = GetMaximum(adc,maxTB);
    				std::cout<<" Max ADC "<<max<<"\n";
			       if(maxTB==0) 
			       {
					std::cout<<" Event with MAXTB "<<i<<"\n";
					for(int indTB=1;indTB<512;++indTB){
						IC_waveform->SetBinContent(indTB,adc[indTB-1]);	
						//std::cout<<indTB<<"	"<<adc[indTB]<<"\n";
					}	
			       }
	
				IC_spec->Fill(max);
				Q_vs_T->Fill(max,maxTB);
			     
			}
		      }

	     }


       }

	TCanvas* c1 = new TCanvas();
	c1->Divide(2,2);
	c1->cd(1);
	Q_vs_T->Draw();
	c1->cd(2);
	IC_spec->Draw();
	c1->cd(3);
	IC_waveform->Draw();

}
