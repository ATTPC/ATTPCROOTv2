void plot_analysis()
{

		std::vector<TString> files//{"run_0158_analysis.root"};
		{"run_0080_analysis.root",
		"run_0081_analysis.root",
		"run_0082_analysis.root",
		"run_0083_analysis.root",
		"run_0085_analysis.root",
		"run_0086_analysis.root",
		"run_0087_analysis.root",
		"run_0091_analysis.root",
		"run_0092_analysis.root",
	    "run_0093_analysis.root",
		"run_0100_analysis.root",
		"run_0101_analysis.root",
		"run_0102_analysis.root",
		"run_0103_analysis.root",
		"run_0104_analysis.root",
		"run_0105_analysis.root",
		"run_0106_analysis.root",
		"run_0107_analysis.root",
		"run_0108_analysis.root",
		"run_0109_analysis.root",
		"run_0110_analysis.root",
		"run_0111_analysis.root",
		"run_0112_analysis.root",
	    "run_0113_analysis.root",
	    "run_0142_analysis.root", 
		"run_0144_analysis.root",
		"run_0145_analysis.root",
		"run_0149_analysis.root",
		"run_0150_analysis.root",
		"run_0152_analysis.root",
		"run_0153_analysis.root",
		"run_0154_analysis.root",
		"run_0156_analysis.root",
		"run_0157_analysis.root",
		"run_0158_analysis.root"};

		std::vector<float> Qcorr_fact
		{
		  1.208,
		  1.208,
		  1.208,
		  1.208,
		  1.208,
		  1.208,
		  1.208,
		  1.208,
		  1.208,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000,
		  1.000
		};

		TString path = "analysis_May2019/";
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

		TH2F* chi2ratio_charge = new TH2F("chi2ratio_charge","chi2ratio_charge",100,0,10,250,0,2000000);

		TH2F* alphaTrigger_event = new TH2F("alphaTrigger_event","alphaTrigger_event",1000,0,4000,100,0,10000000);

		TH2F* alphaTrigger_Qevent = new TH2F("alphaTrigger_Qevent","alphaTrigger_Qevent",1000,0,4000,100,0,10000000);

		TH2F* alpha_proton = new TH2F("alpha_proton","alpha_proton",1000,0,4000,1000,0,4000);

		TH1F* Qexp = new TH1F("Qexp","Qexp",250,0,2000000);
		TH1F* Qexp_low = new TH1F("Qexp_low","Qexp_low",250,0,2000000);
		TH1F* Qexp_cond = new TH1F("Qexp_cond","Qexp_cond",250,0,2000000);
		Qexp_low->SetLineColor(kRed);

		TH1F* Qexp_nocond = new TH1F("Qexp_nocond","Qexp_nocond",1000,0,1000000);
		TH1F* stretch = new TH1F("stretch","stretch",1000,-1.0,1.0);
		TH1F* Qtot = new TH1F("Qtot","Qtot",1000,0,2000000);

		TH2F* Qexp_range = new TH2F("Qexp_range","Qexp_range",1000,0,2000000,1000,0,200.0);

		//Histograms 
		TH1F* chi2_cond = new TH1F("chi2_cond","chi2_cond",1000,0,10000);
		TH2F* chi2_cond_range = new TH2F("chi2_cond_range","chi2_cond_range",1000,0,10000,1000,0,200.0);

		TH1F* exp_curve = new TH1F("exp_curve","exp_curve",512,0,511);
		TH1F* ref_curve = new TH1F("ref_curve","ref_curve",512,0,511);

		TH2F* Qexp_angle = new TH2F("Qexp_angle","Qexp_angle",1000,0,2000000,1000,0,90);

		TH1F* hEnergy = new TH1F("hEnergy","hEnergy",100,0,1000);



		std::size_t global_events = 0;

		int qfacInd = 0;


		for(auto ifile : files)
		{



			std::cout<<" Processing file "<<ifile<<"\n";

			TString QtotoutputFileName = path + "Qtot_" + ifile + ".txt";

			//std::ofstream QtotoutputFile(QtotoutputFileName.Data());

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

            	//QtotoutputFile<<Qtot_tree<<"\n";

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


            		//Gain correction and calibration
            		//std::cout<<"=============="<<"\n";
            		//std::cout<<Qexp_tree<<"\n";
            		Qexp_tree*=Qcorr_fact[qfacInd];
            		//std::cout<<Qexp_tree<<"\n";
            		double Energy = 198.0 +  (Qexp_tree-170000)/(1.87*1000); //kev
            		//std::cout<<Energy<<"\n";
            		//std::cout<<"=============="<<"\n";



            		Qexp_range->Fill(Qexp_tree,range_tree);

            		double Qratio = Qexp_tree/Qref_tree;

            		Qexp_nocond->Fill(Qexp_tree);


            	if(chi2_tree>0 && alphaTrigger_tree<200){

            		//std::cout<<" Event "<<eventNum_tree<<" chi2 proton "<<chi2_tree<<" - chi2 7Li "<<chi2Li_tree<<" - stretch proton "<<stretch_tree<<" - stretch 7Li "<<stretchLi_tree<<"\n";

            		chi2->Fill(chi2_tree);
            		chi2_stretch->Fill(chi2_tree,stretch_tree);
            		chi2Li_stretchLi->Fill(chi2Li_tree,stretchLi_tree);
            		chi2_Qratio->Fill(chi2_tree,Qexp_tree/Qref_tree);
            		chi2_range->Fill(chi2_tree,range_tree);
            		chi2_shift->Fill(chi2_tree,shift_tree);

            		//Qexp->Fill(Qexp_tree);
            		if(chi2_tree<2000 && chi2Li_tree>1000)
            			Qexp_angle->Fill(Qexp_tree,angle_tree);


            		



            		if(stretch_tree>-0.2 && stretch_tree<0.5 && 
            			stretchLi_tree>-0.2 && stretchLi_tree<0.5 
            			&& angle_tree>30 && angle_tree<70 && shift_tree<-25
            			
            			)
            		{

            			chi2_chi2Li_cond->Fill(chi2_tree,chi2Li_tree);
            			if(chi2_tree<2000 && chi2Li_tree>1000) 
            				Qexp->Fill(Qexp_tree);

            			if(chi2_tree<350 && chi2Li_tree>1000) 
            				Qexp_low->Fill(Qexp_tree);

            			if(Qexp_tree<180000 && Qexp_tree>165000)
            				chi2_chi2Li->Fill(chi2_tree,chi2Li_tree);

            			chi2ratio_charge->Fill((chi2_tree/chi2Li_tree),Qexp_tree);

            			if((chi2_tree/chi2Li_tree)>3.5)
            			{
            				Qexp_cond->Fill(Qexp_tree);
            				hEnergy->Fill(Energy);
            			}

            			

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

            //QtotoutputFile.close();
            file->Close();
            ++qfacInd;

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
	   chi2_chi2Li->Draw();
	   c3->cd(2);
	   chi2_Qratio->Draw("zcol");
	   c3->cd(3);
	   chi2ratio_charge->Draw("zcol");
	   c3->cd(4);
	   Qexp_cond->Draw();
	   
	    Qexp->Scale(1.0/Qexp->GetEntries());
	    Qexp_low->Scale(1.0/Qexp_low->GetEntries());


	   TCanvas *c2 = new TCanvas();
	   c2->Divide(2,2);
	   c2->cd(1);
	   chi2_chi2Li_cond->Draw();
	   c2->cd(2);
	   Qexp_low->Draw("hist");
	   Qexp->Draw("hist sames");
	   //chi2_range->Draw("zcol");
	   c2->cd(3);
	   Qexp_angle->Draw();
	   c2->cd(4);
	   Qexp_nocond->Draw();


	   TCanvas *c4 = new TCanvas();
	   hEnergy->Draw();
	   /*c4->Divide(2,2);
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