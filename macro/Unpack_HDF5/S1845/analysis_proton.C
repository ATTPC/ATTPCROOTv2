void analysis_proton()
{

	FairRunAna* run = new FairRunAna(); //Forcing a dummy run

	TString workdir = getenv("VMCWORKDIR");
    TString FileNameHead = "run_0168";
    TString FilePath = workdir + "/macro/Unpack_HDF5/S1845/";
    TString FileNameTail = ".root";
    TString FileName     = FilePath + FileNameHead + FileNameTail;

    std::ofstream outputFileEvent("proton_bragg_events.txt");

    std::cout<<" Opening File : "<<FileName.Data()<<std::endl;
    TFile* file = new TFile(FileName.Data(),"READ");

    TTree* tree = (TTree*) file -> Get("cbmsim");
    Int_t nEvents = tree -> GetEntries();
    std::cout<<" Number of events : "<<nEvents<<std::endl;

    TTreeReader Reader1("cbmsim", file);
    TTreeReaderValue<TClonesArray> raweventArray(Reader1, "ATRawEvent");

    TH2F* Q1_vs_Q2_int = new TH2F("Q1_vs_Q2_int","Q1_vs_Q2_int",500,0,500000,500,0,500000);

    TH1F* hQ = new TH1F("hQ","hQ",500,0,500000);

    TH1F* mesh = new TH1F("mesh","mesh",512,0,511);

    int numevt = 10000;

    for(Int_t i=0;i<numevt;i++){
          //while (Reader1.Next()) {

              Reader1.Next();

              mesh->Reset();

              
              ATRawEvent *rawEvent = (ATRawEvent*) raweventArray->At(0);

              outputFileEvent<<"		"<<"\n";
              outputFileEvent<<" Event "<<i<<" "<<"\n";

              if(rawEvent!=NULL)
              {

              	  std::vector<ATPad> *padArray = rawEvent->GetPads();

              	  std::cout<<" Raw event number : "<<i<<"\n";
	              std::cout<<" Number of pads : "<<padArray->size()<<"\n";

	              bool isValid = true;

	              int firstTBOT = 1000;
	              int lastTBOT  = -10;
	              double Qint_nearFirst = 0;
	              double Qint_nearLast = 0;
	              double Qtotal = 0;

	              for(auto padInd=0;padInd<padArray->size();++padInd)
	              {
	              		Double_t *adc =  padArray->at(padInd).GetADC();

	              		//std::cout<<" Pad Index "<<padInd<<"	Pad Number "<<padArray->at(padInd).GetPadNum()<<"\n";

	              	if(padArray->at(padInd).GetPadNum()>-1){	

	              		for(int indTB=0;indTB<512;++indTB)
	              	    {

	              	    				if(adc[indTB]>40.0){

	              	    					//std::cout<<" TB "<<indTB<<" ADC "<<adc[indTB]<<"\n";
	              	    					if(indTB<firstTBOT) firstTBOT = indTB;
	              	    					if(indTB>lastTBOT)  lastTBOT = indTB;

	              	    					mesh->Fill(indTB,adc[indTB]);

	              	    					Qtotal+=adc[indTB];

	              	    				}

	              	    					
	              	    }

	              	    //std::cout<<" First TBOT "<<firstTBOT<<" Last TBOT "<<lastTBOT<<"\n";			
	             	}//if
	             	
	             }//for	

	             int midTBOT = (lastTBOT - firstTBOT)/2.0 + firstTBOT; 

	             std::cout<<" First TBOT "<<firstTBOT<<" Last TBOT "<<lastTBOT<<" Middle TBOT "<<midTBOT<<"\n";

	             std::cout<<" Total Q "<<Qtotal<<"\n";

	             float Qtotalcal = Qtotal;

	             //if(Qtotal>1.68017e+05) Qtotalcal = gRandom->Gaus(Qtotal-10000,500);
	             //if(Qtotal<1.68017e+05) 

	             Qtotalcal = gRandom->Gaus(Qtotal/2.0,500)+1.68017e+05/2.0;
;


	             hQ->Fill(Qtotalcal);

	             for(auto padInd=0;padInd<padArray->size();++padInd)
	             {

	             	Double_t *adc =  padArray->at(padInd).GetADC();

	             	for(int indTB = firstTBOT;indTB<=midTBOT;++indTB)
	             		if(adc[indTB]>40.0) Qint_nearFirst+=adc[indTB];

	             	for(int indTB = midTBOT;indTB<lastTBOT;++indTB)	
	             		if(adc[indTB]>40.0) Qint_nearLast+=adc[indTB];

	             }

	             Q1_vs_Q2_int->Fill(Qint_nearFirst,Qint_nearLast);


              }//Rawevent	

              for(int indTB=0;indTB<512;++indTB)
			  {
				outputFileEvent<<indTB<<"	"<<mesh->GetBinContent(indTB)<<"	"<<mesh->GetBinError(indTB)<<"\n";	
			  }
    }

    mesh->Scale(1.0/numevt);

    std::ofstream outputFile("proton_bragg.txt");

    for(int indTB=0;indTB<512;++indTB)
	{
		outputFile<<indTB<<"	"<<mesh->GetBinContent(indTB)<<"	"<<mesh->GetBinError(indTB)<<"\n";
	
	}

	outputFile.close();	



    TCanvas *c1 = new TCanvas();
     c1->Divide(2,2);
     c1->cd(1);
     Q1_vs_Q2_int->Draw("zcol");
     c1->cd(2);
     mesh->Draw("hist");

     TCanvas *c2 = new TCanvas();
     hQ->Draw();


}