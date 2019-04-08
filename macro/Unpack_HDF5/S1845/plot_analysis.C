void plot_analysis()
{

		std::vector<TString> files{"run_0080_analysis.root"
		,"run_0141_analysis.root"
		,"run_0142_analysis.root"
		,"run_0144_analysis.root"
		,"run_0145_analysis.root"
		,"run_0149_analysis.root"
		,"run_0150_analysis.root"
		,"run_0152_analysis.root"
		,"run_0153_analysis.root"
		,"run_0154_analysis.root"
		,"run_0156_analysis.root"
		,"run_0157_analysis.root"
		,"run_0158_analysis.root"
		,"run_0159_analysis.root"};

		TString path = "analysis/";


		//Histograms 
		TH1F* chi2 = new TH1F("chi2","chi2",1000,0,10000);
		TH2F* chi2_stretch = new TH2F("chi2_stretch","chi2_stretch",1000,0,10000,100,-1.0,1.0);
		TH2F* chi2_Qratio = new TH2F("chi2_Qratio","chi2_Qratio",1000,0,10000,100,0,10.0);
		TH2F* stretch_Qratio = new TH2F("stretch_Qratio","stretch_Qratio",100,-1.0,1.0,100,0,10.0);
		TH1F* alphaTrigger = new TH1F("alphaTrigger","alphaTrigger",10000,0,4000);
		TH2F* alphaTrigger_angle = new TH2F("alphaTrigger_angle","alphaTrigger_angle",1000,0,4000,100,0,180);

		TH2F* alphaTrigger_event = new TH2F("alphaTrigger_event","alphaTrigger_event",1000,0,4000,100,0,10000000);

		TH2F* alphaTrigger_Qevent = new TH2F("alphaTrigger_Qevent","alphaTrigger_Qevent",1000,0,4000,100,0,10000000);

		TH2F* alpha_proton = new TH2F("alpha_proton","alpha_proton",1000,0,4000,1000,0,4000);

		TH1F* Qexp = new TH1F("Qexp","Qexp",1000,0,2000000);



		std::size_t global_events = 0;


		for(auto ifile : files)
		{

			std::cout<<" Processing file "<<ifile<<"\n";

			TFile* file = new TFile(path+ifile,"READ");

			double chi2_tree = 0;
   		    double Qref_tree = 0;
    		double Qexp_tree = 0;
    		double stretch_tree = 0;
    		double angle_tree = 0;
    		double range_tree = 0;
    		double shift_tree = 0;
    		double protonTrigger_tree = 0;
    		double alphaTrigger_tree = 0;
    		double eventNum_tree = 0;

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

            	alphaTrigger_event->Fill(alphaTrigger_tree,global_events);
            	alphaTrigger_Qevent->Fill(alphaTrigger_tree,Qexp_tree);
            	alpha_proton->Fill(protonTrigger_tree,alphaTrigger_tree);
            	//if( protonTrigger_tree>300 ) 
            		//Qexp->Fill(Qexp_tree);

            	++global_events;

            	//if(alphaTrigger_tree>0 && protonTrigger_tree<300)
            	//{ 
            		alphaTrigger->Fill(alphaTrigger_tree);
            		alphaTrigger_angle->Fill(alphaTrigger_tree,angle_tree);
            	//}


            	if(chi2_tree>0 && alphaTrigger_tree<100){
            		chi2->Fill(chi2_tree);
            		chi2_stretch->Fill(chi2_tree,stretch_tree);
            		chi2_Qratio->Fill(chi2_tree,Qexp_tree/Qref_tree);
            		if(chi2_tree<1500)stretch_Qratio->Fill(stretch_tree,Qexp_tree/Qref_tree);
            		
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
	   c1->cd(3);
	   chi2_Qratio->Draw();
	   c1->cd(4);
	   stretch_Qratio->Draw();
	   c1->cd(5);
	   alphaTrigger->Draw();
	   c1->cd(6);
	   alphaTrigger_angle->Draw();
	   c1->cd(7);
	   alphaTrigger_event->Draw();
	   c1->cd(8);
	   alphaTrigger_Qevent->Draw();
	   c1->cd(9);
	   alpha_proton->Draw();

	   TCanvas *c2 = new TCanvas();
	   Qexp->Draw();


}