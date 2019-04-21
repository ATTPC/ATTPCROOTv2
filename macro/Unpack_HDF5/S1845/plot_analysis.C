void plot_analysis()
{

		std::vector<TString> files{"run_0080_analysis.root"};
			//"run_0081_analysis.root",
		//"run_0082_analysis.root",
		//"run_0083_analysis.root"};
		/*,"run_0141_analysis.root"
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
		,"run_0159_analysis.root"};*/

		TString path = "analysis/";
		//TString path = "";


		//Histograms 
		TH1F* chi2 = new TH1F("chi2","chi2",1000,0,10000);
		TH2F* chi2_chi2Li = new TH2F("chi2_chi2Li","chi2_chi2Li",1000,0,10000,1000,0,10000);
		TH2F* chi2_chi2Li_cond = new TH2F("chi2_chi2Li_cond","chi2_chi2Li_cond",1000,0,10000,1000,0,10000);
		TH2F* chi2_stretch = new TH2F("chi2_stretch","chi2_stretch",1000,0,10000,100,-2.0,2.0);
		TH2F* chi2Li_stretchLi = new TH2F("chi2Li_stretchLi","chi2Li_stretchLi",1000,0,10000,100,-2.0,2.0);
		TH2F* chi2_range = new TH2F("chi2_range","chi2_range",1000,0,10000,1000,0.0,200.0);
		TH2F* chi2_Qratio = new TH2F("chi2_Qratio","chi2_Qratio",1000,0,10000,100,0,10.0);
		TH2F* chi2_shift = new TH2F("chi2_shift","chi2_shift",1000,0,10000,100,-300,300.0);
		TH2F* stretch_Qratio = new TH2F("stretch_Qratio","stretch_Qratio",1000,-1.0,1.0,1000,0,10.0);
		TH1F* alphaTrigger = new TH1F("alphaTrigger","alphaTrigger",10000,0,4000);
		TH2F* alphaTrigger_angle = new TH2F("alphaTrigger_angle","alphaTrigger_angle",1000,0,4000,100,0,180);

		TH2F* alphaTrigger_event = new TH2F("alphaTrigger_event","alphaTrigger_event",1000,0,4000,100,0,10000000);

		TH2F* alphaTrigger_Qevent = new TH2F("alphaTrigger_Qevent","alphaTrigger_Qevent",1000,0,4000,100,0,10000000);

		TH2F* alpha_proton = new TH2F("alpha_proton","alpha_proton",1000,0,4000,1000,0,4000);

		TH1F* Qexp = new TH1F("Qexp","Qexp",1000,0,2000000);
		TH1F* stretch = new TH1F("stretch","stretch",1000,-1.0,1.0);
		TH1F* Qtot = new TH1F("Qtot","Qtot",1000,0,2000000);

		TH2F* Qexp_range = new TH2F("Qexp_range","Qexp_range",1000,0,2000000,1000,0,200.0);

		//Histograms 
		TH1F* chi2_cond = new TH1F("chi2_cond","chi2_cond",1000,0,10000);
		TH2F* chi2_cond_range = new TH2F("chi2_cond_range","chi2_cond_range",1000,0,10000,1000,0,200.0);

		TH1F* exp_curve = new TH1F("exp_curve","exp_curve",512,0,511);
		TH1F* ref_curve = new TH1F("ref_curve","ref_curve",512,0,511);



		std::size_t global_events = 0;


		for(auto ifile : files)
		{

			std::cout<<" Processing file "<<ifile<<"\n";

			TFile* file = new TFile(path+ifile,"READ");

			double chi2_tree = 0;
			double chi2Li_tree = 0;
   		    double Qref_tree = 0;
    		double Qexp_tree = 0;
    		double stretch_tree = 0;
    		double stretchLi_tree = 0;
    		double angle_tree = 0;
    		double range_tree = 0;
    		double shift_tree = 0;
    		double protonTrigger_tree = 0;
    		double alphaTrigger_tree = 0;
    		double Qtot_tree = 0;
    		int eventNum_tree = 0;

    		std::vector<double>* exp_curve_tree = new std::vector<double>();
    		std::vector<double>* ref_curve_tree = new std::vector<double>();

		    TTree* analysisTree = (TTree*) file -> Get("analysisTree");
            Int_t nEvents = analysisTree -> GetEntries();
            std::cout<<" Number of events : "<<nEvents<<std::endl;

            analysisTree->SetBranchAddress("chi2_tree",&chi2_tree);
            analysisTree->SetBranchAddress("chi2Li_tree",&chi2Li_tree);
    		analysisTree->SetBranchAddress("Qref_tree",&Qref_tree);
    		analysisTree->SetBranchAddress("Qexp_tree",&Qexp_tree);
    		analysisTree->SetBranchAddress("angle_tree",&angle_tree);
    		analysisTree->SetBranchAddress("stretch_tree",&stretch_tree);
    		analysisTree->SetBranchAddress("stretchLi_tree",&stretchLi_tree);
    		analysisTree->SetBranchAddress("range_tree",&range_tree);
    		analysisTree->SetBranchAddress("shift_tree",&shift_tree);
    		analysisTree->SetBranchAddress("protonTrigger_tree",&protonTrigger_tree);
    		analysisTree->SetBranchAddress("alphaTrigger_tree",&alphaTrigger_tree);
    		analysisTree->SetBranchAddress("eventNum_tree",&eventNum_tree);
    		//analysisTree->SetBranchAddress("exp_curve_tree",&exp_curve_tree);
    		//analysisTree->SetBranchAddress("ref_curve_tree",&ref_curve_tree);
    		analysisTree->SetBranchAddress("Qtot_tree",&Qtot_tree);


            for(auto iEvent =0;iEvent<nEvents;++iEvent)
            {
            	analysisTree->GetEntry(iEvent);


			  	//exp_curve_tree->clear();
    		  	//ref_curve_tree->clear();

            	alphaTrigger_event->Fill(alphaTrigger_tree,eventNum_tree);
            	alphaTrigger_Qevent->Fill(alphaTrigger_tree,Qexp_tree);
            	alpha_proton->Fill(protonTrigger_tree,alphaTrigger_tree);
            	//if( protonTrigger_tree>300 ) 
            		//Qexp->Fill(Qexp_tree);

            	++global_events;

            
            		alphaTrigger->Fill(alphaTrigger_tree);
            		alphaTrigger_angle->Fill(alphaTrigger_tree,angle_tree);

            		Qexp_range->Fill(Qexp_tree,range_tree);

            		double Qratio = Qexp_tree/Qref_tree;


            	if(chi2_tree>0 && alphaTrigger_tree<200){
            		chi2->Fill(chi2_tree);
            		chi2_stretch->Fill(chi2_tree,stretch_tree);
            		chi2Li_stretchLi->Fill(chi2Li_tree,stretchLi_tree);
            		chi2_Qratio->Fill(chi2_tree,Qexp_tree/Qref_tree);
            		chi2_range->Fill(chi2_tree,range_tree);
            		chi2_shift->Fill(chi2_tree,shift_tree);
            		chi2_chi2Li->Fill(chi2_tree,chi2Li_tree);



            		if(stretch_tree>-0.02 && stretch_tree<0.5 && stretchLi_tree>-0.2 && stretchLi_tree<0.5 && angle_tree>10 && angle_tree<60 && Qratio>0.6)
            		{

            			chi2_chi2Li_cond->Fill(chi2_tree,chi2Li_tree);
            			if(chi2_tree<1000 && chi2Li_tree>1000) Qexp->Fill(Qexp_tree);

            		}



            		   //if(chi2_tree<600) stretch_Qratio->Fill(stretch_tree,Qexp_tree/Qref_tree);

            		/*if(chi2_tree<600&&chi2_tree>100&&stretch_tree<0.04&&stretch_tree>0.01){
            			 
            			 Qexp->Fill(Qexp_tree);
            			 stretch->Fill(stretch_tree);
            		}*/

            		/*double Qratio = Qexp_tree/Qref_tree;

            		if(Qratio<1.1&&Qratio>0.8&&stretch_tree<0.04&&stretch_tree>0.01){
            			chi2_cond->Fill(chi2_tree);
            			chi2_cond_range->Fill(chi2_tree,range_tree);
            		}*/
            		
            	}

            	/*if(chi2_tree<300&&chi2_tree>50)
            	{


	            	for(int ivec = 0;ivec<exp_curve_tree->size();++ivec)
	            	{
	            		exp_curve->SetBinContent(ivec,exp_curve_tree->at(ivec));
	            		
	            	}

	            	for(int ivec = 0;ivec<ref_curve_tree->size();++ivec)
	            	{
	            		ref_curve->SetBinContent(ivec,ref_curve_tree->at(ivec));	
	            	}
	            }*/	

            }

            file->Close();

	   }	

	   TCanvas *c1 = new TCanvas();
	   c1->Divide(2,2);
	   c1->cd(1);
	   chi2->Draw();
	   c1->cd(2);
	   chi2_stretch->Draw("zcol");
	   c1->cd(3);
	   chi2Li_stretchLi->Draw("zcol");
	   c1->cd(4);
	   chi2_range->Draw("zcol");

	   TCanvas *c3 = new TCanvas();
	   c3->Divide(2,2);
	   c3->cd(1);
	   chi2_chi2Li->Draw("zcol");
	   c3->cd(2);
	   chi2_Qratio->Draw("zcol");
	   /*c3->cd(3);
	   alphaTrigger_angle->Draw();
	   c3->cd(4);
	   alphaTrigger_event->Draw();*/
	   
	   

	   TCanvas *c2 = new TCanvas();
	   c2->Divide(2,2);
	   c2->cd(1);
	   chi2_chi2Li_cond->Draw();
	   c2->cd(2);
	   Qexp->Draw();
	   /*chi2_range->Draw("zcol");
	   c2->cd(3);
	   chi2_shift->Draw("zcol");
	   c2->cd(4);
	   alpha_proton->Draw();


	   TCanvas *c4 = new TCanvas();
	   c4->Divide(2,2);
	   c4->cd(1);
	   stretch->Draw("zcol");
	   c4->cd(2);
	   chi2_cond->Draw();
	   c4->cd(3);
	   Qexp_range->Draw("zcol");
	   c4->cd(4);
	   alphaTrigger_Qevent->Draw();

	    TCanvas *c5 = new TCanvas();
	    chi2_cond_range->Draw();

	    TCanvas *c6 = new TCanvas();
	    exp_curve->Draw();
	    ref_curve->Draw("SAME");*/

}