void plot_analysis()
{

		std::vector<TString> files{"run_0080_analysis.root"
		,"run_0141_analysis.root"
		,"run_0142_analysis.root"
		,"run_0144_analysis.root"
		,"run_0145_analysis.root"};


		//Histograms 
		TH1F* chi2 = new TH1F("chi2","chi2",1000,0,10000);
		TH2F* chi2_stretch = new TH2F("chi2_stretch","chi2_stretch",1000,0,10000,100,-1.0,1.0);

		for(auto ifile : files)
		{

			std::cout<<" Processing file "<<ifile<<"\n";

			TFile* file = new TFile(ifile,"READ");

			double chi2_tree = 0;
   		    double Qref_tree = 0;
    		double Qexp_tree = 0;
    		double stretch_tree = 0;
    		double angle_tree = 0;
    		double range_tree = 0;
    		double shift_tree = 0;
    		double protonTrigger_tree = 0;
    		double alphaTrigger_tree = 0;
    		double    eventNum_tree = 0;

		    TTree* analysisTree = (TTree*) file -> Get("analysisTree");
            Int_t nEvents = analysisTree -> GetEntries();
            std::cout<<" Number of events : "<<nEvents<<std::endl;

            analysisTree->SetBranchAddress("chi2_tree",&chi2_tree);
    		analysisTree->SetBranchAddress("Qref_tree",&Qref_tree);
    		analysisTree->SetBranchAddress("Qexp_tree",&Qexp_tree);
    		analysisTree->SetBranchAddress("angle_tree",&angle_tree);
    		analysisTree->SetBranchAddress("stretch_tree",&stretch_tree);
    		analysisTree->SetBranchAddress("range_tree",&range_tree);
    		analysisTree->SetBranchAddress("shift_tree",&shift_tree);
    		analysisTree->SetBranchAddress("protonTrigger_tree",&protonTrigger_tree);
    		analysisTree->SetBranchAddress("alphaTrigger_tree",&alphaTrigger_tree);
    		analysisTree->SetBranchAddress("eventNum_tree",&eventNum_tree);


            for(auto iEvent =0;iEvent<nEvents;++iEvent)
            {
            	analysisTree->GetEntry(iEvent);
            	if(chi2_tree>0 && alphaTrigger_tree<100){
            		chi2->Fill(chi2_tree);
            		chi2_stretch->Fill(chi2_tree,stretch_tree);
            	}	


            }

            file->Close();

	   }	

	   TCanvas *c1 = new TCanvas();
	   c1->Divide(3,3);
	   c1->cd(1);
	   chi2->Draw();
	   c1->cd(2);
	   chi2_stretch->Draw();

}