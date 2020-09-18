
void ana_Hits(int  rnumb=243)
{



  // -----   Timer   --------------------------------------------------------
 TStopwatch timer;
 timer.Start();
 // ------------------------------------------------------------------------


	gSystem->Load("libATTPCReco.so");
	gSystem->Load("libAtTpcMap.so");

  // -----------------------------------------------------------------


	  FairRunAna* run = new FairRunAna(); //Forcing a dummy run

    TString workdir = getenv("VMCWORKDIR");
    std::string strnumb = std::to_string(rnumb);
    //TString FileNameHead = "output_run_224";
    //TString FileNameHead = "output_run_"+strnumb;
    //TString FilePath = "/home/juan/proyectos/ND_2019/pAT-TPC_17F_Sep2019/hits/";
    //TString FilePath = "/home/juan/proyectos/ND_2019/pAT-TPC_14O_Sep2019/hits/";
    //TString FileNameHead = "output_run0037";
    //TString FilePath = "/home/juan/FairRoot/ATTPCROOTv2_2020/ATTPCROOTv2/macro/Unpack_HDF5/ND2019/";
    //TString FileNameTail = ".root";
    //TString FileNameOut  = "_digi";
    //TString FileName     = "attpcdigi_d2He_1000.root";
    //TString OutputFileName = "salida_attpcdigi_d2He_1000.root";
    TString FileName     = "attpcdigi_d2He_1000_ran.root";
    TString OutputFileName = "salida_attpcdigi_d2He_1000_ran.root";
    //TString FileName     = "attpcdigi_d2He_1e6pps_lmed.root";
    //TString OutputFileName = "salida_attpcdigi_d2He_1e6pps_lmed.root";



    std::cout<<" Opening File : "<<FileName.Data()<<std::endl;
    TFile* file = new TFile(FileName.Data(),"READ");

    TTree* tree = (TTree*) file -> Get("cbmsim");
    Int_t nEvents = tree -> GetEntries();
    std::cout<<" Number of events : "<<nEvents<<std::endl;

    TTreeReader Reader1("cbmsim", file);
    //TTreeReaderValue<TClonesArray> raweventArray(Reader1, "ATRawEvent");
    TTreeReaderValue<TClonesArray> eventArray(Reader1, "ATEventH");
    TTreeReaderValue<TClonesArray> fRansacArray(Reader1, "ATRansac");

	/*
    TString scriptfile = "LookupProtoND.xml";
    TString protomapfile = "proto20181201.map";
    TString geo = "ATTPC_Proto_v1.0.root";
	*/
    TString scriptfile = "Lookup20150611.xml";
    //TString protomapfile = "proto20181201.map";
    TString geo = "ATTPC_v1.1_geomanager.root";
    TString scriptdir = workdir + "/scripts/"+ scriptfile;
    //TString protomapdir = workdir + "/scripts/"+ protomapfile;

	/*
    AtTpcMap *fAtMapPtr = new AtTpcProtoMap();
    fAtMapPtr->ParseXMLMap(scriptdir.Data());
    fAtMapPtr->SetGeoFile(geo);
    fAtMapPtr->SetProtoMap(protomapdir.Data());
	*/

    //AtTpcMap *fAtMapPtr = new AtTpcMap();
    //fAtMapPtr->ParseXMLMap(scriptdir.Data());

    //TH2Poly* fPadPlane = fAtMapPtr->GetATTPCPlane("ATTPC_Proto");

    // Histograms


    std::vector<double> Vpx;
    std::vector<double> Vpy;
    std::vector<double> Vpz;
    std::vector<double> Vpt;
    std::vector<double> Vpe;
    std::vector<int> Vhits;
    std::vector<pair<double,double>> AuxTac;
    std::vector<pair<double,double>> AuxProton;
    std::vector<pair<double,double>> AuxIC;
    std::vector<pair<double,double>> AuxUnkown;
    std::vector<pair<double,double>> AuxAlphas;
    double Verz =0;
    int Ntracks = 0;

    TFile *analysisFile = new TFile(OutputFileName,"RECREATE");
    TTree *analysisTree = new TTree("analysisTree","analysis");
    analysisTree->Branch("vertz",&Verz);
    analysisTree->Branch("tracksN",&Ntracks);


    TH1F* WSignals[10];
    TH1F* WBg[10];
    TSpectrum * fspec = new TSpectrum();

     for(Int_t i=0;i<nEvents;i++){
       //for(Int_t i=0;i<100;i++){
          //while (Reader1.Next()) {

        Reader1.Next();

        ATEvent* event = (ATEvent*) eventArray->At(0);
        //ATRawEvent *rawEvent = (ATRawEvent*) raweventArray->At(0);
	      //auto fRansac = dynamic_cast<ATRANSACN::ATRansac*> (fRansacArray->At(0));
        auto fRansac = dynamic_cast<ATRansacMod*> (fRansacArray->At(0));
        //auto fRansac = dynamic_cast<ATLmedsMod*> (fRansacArray->At(0));


        if(event!=NULL)
        {
		if(i%2!=0) continue;
		//std::cout<<" Hola mundo!!"<<std::endl;
		auto TrackCand = fRansac->GetTrackCand();
            	TVector3 Vertex1    = fRansac->GetVertex1();
            	TVector3 Vertex2    = fRansac->GetVertex2();
		Verz = 0.5*(Vertex1.Z()+Vertex2.Z());
    Ntracks = TrackCand.size();
		//std::cout<<i<<"  "<<TrackCand.size()<<std::endl;
		//std::cout<<i<<"  "<<Vertex1.Z()<<"  "<<Vertex2.Z()<<"  "<<0.5*(Vertex1.Z()+Vertex2.Z())<<std::endl;
	         //Int_t nHits = event->GetNumHits();
	         //std::vector<ATHit>* hitArray = event->GetHitArray();
	         //event->GetHitArrayObj();
	         //if(i%1000==0)std::cout<<" 	**** Event Number : "<<i<<" Hits : "<<hitArray->size()<<std::endl;
	         if(Verz>0 && sqrt(Vertex1.X()*Vertex1.X() + Vertex1.Y()*Vertex1.Y())< 50 &&  Verz<1000 ) analysisTree->Fill();

	    } //if Event!=null



       //if(event!=NULL) analysisTree->Fill();
     }//for event loop

     analysisTree->Write();
     analysisFile->Close();


	std::cout << std::endl << std::endl;
  std::cout << "Macro finished succesfully."  << std::endl << std::endl;
  std::cout << "- Output file : " << OutputFileName << std::endl << std::endl;
  // -----   Finish   -------------------------------------------------------
  timer.Stop();
  Double_t rtime = timer.RealTime();
  Double_t ctime = timer.CpuTime();
  cout << endl << endl;
  cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
  cout << endl;
  // ------------------------------------------------------------------------


}
