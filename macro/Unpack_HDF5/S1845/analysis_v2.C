Double_t fitref(int x)
{
	double c4=x-100.;
		
	double c6=1580.*exp(-TMath::Power(((c4-197.)/16.),2)) + 1550.*exp(-TMath::Power(((c4-224.)/11.),2)) 
		   + 650.*exp(-TMath::Power(((c4-211.)/10.),2)) - 150.*exp(-TMath::Power(((c4-221.)/6.),2)) 
		   + 70.*exp(-TMath::Power(((c4-214.)/5.),2));
	
	double fitref=c6*2.0;

	return fitref;


}

double fitLi(int x)
{
          // ! to calculate the form of Li excited recoil
          double anal=0.;
             if(x>280.)
             {
                 anal=(x-280.)*110.;// !*233./200. !renormalized
                 anal=anal*cos(45.*0.01745); //!refrence was around 45deg
                 int dxx=345-x;// !end of triangle
                 anal=anal*1./(1.+exp(-dxx/3.));
                 double fitLi=anal*1.27;
                 return fitLi;
             }else{
                return 0;
             }
}           

Double_t chi2fitImp(int ioptionpart, TH1F* mesh, double &sumref, double sumexp, double angle,int iTb, double &chimin,double &shiftmin, double &stretchmin, std::vector<double>& ref_curve, std::vector<double>& exp_curve)
{

           int    dzero=110;
           double dstretch=3.;
           double stretch0=0.;  //!form start the calc to exp for ibegin

           int shift0= 0;
           
           if(ioptionpart==1) shift0=0.;
           if(ioptionpart==2) shift0=0.;

           int istretchmax=100;
           int izeromax=100;
           int itb0=300;//  !bricolage, to be replaced by center of gravity but probably not important

           double stretch = 0;
           int    shift = 0;
           double sumrefmin = 0;

           chimin=1.e6;

           double sumrefold = sumref;

           for(int iter=0;iter<1;++iter)
           {
              for(int istretch=0;istretch<istretchmax;++istretch)
              {

                stretch=dstretch*(float(istretch-istretchmax/2.0)/float(istretchmax));
                sumref=0.;

                  for(int itb=200;itb<400;++itb)
                  {
                    stretch0=1.;
                    int tbref=itb+(stretch0+stretch)*(itb-320)/cos(angle*0.01745);
                    double fitfit = 0;
                    if(ioptionpart==1) fitfit = fitref(tbref); //!function fitted at point bref corrected for projection on exp axis
                    if(ioptionpart==2) fitfit = fitLi(tbref);
                    sumref=sumref+fitfit;
                  } //TB

                  double ratios=sumref/sumrefold;

                  for(int izero=0;izero<izeromax;++izero)
                  {
                      double zerohalf=float(izeromax)/2.;
                      shift = dzero*(float(izero)-zerohalf)/float(izeromax);
                      shift = shift+shift0;

                      double chi2=0.;
                      double nchi2=0;

                        for(int itb=200;itb<400;++itb)
                        {
                         stretch0=1.;
                         int tbref= itb +(stretch0+stretch)*(itb-320)/cos(angle*0.01745)+shift;
                         double fitfit = 0;
                         if(ioptionpart==1) fitfit = fitref(tbref);// !function fitted at point bref corrected for projection on exp axis
                         if(ioptionpart==2) fitfit = fitLi(tbref);

                         fitfit=(sumexp/sumref)*fitfit;
                         double sigma=(mesh->GetBinContent(itb)+fitfit)*0.005+100.;
                         if(sigma>100.9) nchi2=nchi2+1;
                         chi2=chi2+TMath::Power( (mesh->GetBinContent(itb)-fitfit)/sigma,2 );

                        } //TB 

                        if ( chi2<chimin)
                        {
                          chimin= chi2;
                          stretchmin=stretch+stretch0;
                          shiftmin=shift;
                          sumrefmin=sumref;
                        } 

                  }//izero    

              } //istretch

                sumref=sumrefmin;//  !to transmit back the sum for the minimum..

                shift0=shiftmin;
                stretch0=stretchmin;
                dzero=dzero/2.;
                dstretch=dstretch/2.;


           }//iter

           return 0;

}  

Double_t chi2fit(TH1F* mesh, double Qrefproton,double Qreflith, double Qrefexp, double angle,int iTb, double &chimin,double &shiftmin, double &stretchmin, std::vector<double>& ref_curve, std::vector<double>& exp_curve)
{

	int    dzero=50; //initial step when minimizing
	double dstretch=0.5; 
	double stretch0=0.;  //shifts the calc to exp for ibegin
	int    shift0=200.-iTb;
	int    istretchmax=40;
	int    izeromax=20;
	int    itb0=200;  //bricolage, to be replaced by center of gravity but probably not important

	double stretch = 0;
	int    shift = 0;

	chimin=1.e6;

	double chi2[100][100];

		for(int iter=0;iter<2;++iter)
		{

			for(int istretch=0;istretch<istretchmax;++istretch)
			{

				for(int izero=0;izero<izeromax;++izero)
				{
					chi2[istretch][izero]=0.;

					std::vector<double> ref_curve_buffer;
					std::vector<double> exp_curve_buffer;


						for(int itb=100;itb<400;++itb)
						{

							shift          = dzero*(float(izero-izeromax/2.0)/float(izeromax));
							shift          = shift+shift0; //  !could be better limited this is the tb of the evnt
							stretch        = dstretch*(float(istretch-istretchmax/2.0)/float(istretchmax)) + stretch0;
							int tbref      = itb+shift+stretch*(itb-itb0); //!this is the coordinate of the analytical function before angle correction
							tbref          = 200.+ (tbref-200.)/cos(angle*0.01745); // !zero degree=beam=tbucket direction

							//NB:: For the 7Li fit, we need to multiply by cos(45) tbref and fitfit
							double fitfit  = fitref(tbref)/cos(angle*0.01745);// !function fitted at point bref corrected for projection on exp axis

							double sigma          = (mesh->GetBinContent(itb)+fitfit)*0.1+100.;//  !error to be tested
				 			sigma                 = 100.;//  !error to be tested
				 			chi2[istretch][izero] = chi2[istretch][izero]+ TMath::Power(((mesh->GetBinContent(itb)*Qrefproton/Qrefexp-fitfit)/sigma),2) ;// !now with renormalized
				 			
				 			ref_curve_buffer.push_back(fitfit);
				 			exp_curve_buffer.push_back(mesh->GetBinContent(itb)*Qrefproton/Qrefexp);
				 			//std::cout<<" chi2inner "<<chi2[istretch][izero]<<"\n";

						}//TB

						if( chi2[istretch][izero]<chimin)
						{
							chimin= chi2[istretch][izero];
							stretchmin=stretch;
							shiftmin=shift;
							exp_curve = exp_curve_buffer;
							ref_curve = ref_curve_buffer;
						}	

				}//izero

			}//istretch

			shift0=shiftmin;
			stretch0=stretchmin;
			dzero=dzero/10.;
			dstretch=dstretch/10.;


		}//iter

	return 0;
}

Double_t* GetPadWaveForm(Int_t padnum,std::vector<ATPad> *padarray)
{
	 auto it =  find_if( padarray->begin(),padarray->end(),[&padnum](ATPad& pad) {return pad.GetPadNum()==padnum;}   );

	 if(it != padarray->end()){
             auto padInd = std::distance<std::vector<ATPad>::const_iterator>(padarray->begin(),it);
             //std::cout<<" Pad found !"<<"\n";

             if(padarray->at(padInd).GetPadNum()>-1){
             	return padarray->at(padInd).GetADC();
         	 }else{
         	 	std::cout<<" Invalid pad number : "<<padarray->at(padInd).GetPadNum()<<"\n";
         	 	return 0;
         	 }
     }else{
      std::cerr<<" Warning! : Padnum not found in Pad Array!"<<"\n";	
      return 0;
     } 
}

Double_t distance2( double x,double y,double z, const double *p)
{

    // distance line point is D= | (xp-x0) cross  ux |
    // where ux is direction of line and x0 is a point in the line (like t = 0) and x1 is in t=1
    XYZVector xp(x,y,z);
    XYZVector x0(p[0], p[2], 0. );
    XYZVector x1(p[0] + p[1], p[2] + p[3], 1. );
    XYZVector u = (x1-x0).Unit();
    double d2 = ((xp-x0).Cross(u)).Mag2();
    return d2;
}

struct SumDistance2
      {
          TGraph2D * fGraph;

              SumDistance2(TGraph2D * g) : fGraph(g) {}
                  double operator() (const double * par) {
                  assert(fGraph    != 0);
                  double * x = fGraph->GetX();
                  double * y = fGraph->GetY();
                  double * z = fGraph->GetZ();
                  int npoints = fGraph->GetN();
                  double sum = 0;
                  for (int i  = 0; i < npoints; ++i) {
                    double d = distance2(x[i],y[i],z[i],par);
                    sum += d;
                  }
            #ifdef DEBUG
             if (first) std::cout << "point " << i << "\t"
                << x[i] << "\t"
                << y[i] << "\t"
                << z[i] << "\t"
                << std::sqrt(d) << std::endl;
            #endif

            //if (first)
              //std::cout << "Total Initial distance square = " << sum << std::endl;
              //first = false;
              return sum;


              }


};

Double_t GetAngle(ATTrack* track)
{

	gErrorIgnoreLevel=kFatal;
    Int_t nd = 10000;
	TGraph2D * gr = new TGraph2D();
	std::vector<ATHit> *HitArray = track->GetHitArray();

	 double p0[4] = {10,20,1,2}; //For the moment those are dummy parameters

            for(Int_t N=0;N<HitArray->size();N++){
              ATHit hit = HitArray->at(N);
              TVector3 pos = hit.GetPosition();
              gr->SetPoint(N,pos.X(),pos.Y(),pos.Z());
			}

			ROOT::Fit::Fitter fitter;
            SumDistance2 sdist(gr);
            #ifdef __CINT__
            ROOT::Math::Functor fcn(&sdist,4,"SumDistance2");
            #else
            ROOT::Math::Functor fcn(sdist,4);
            #endif
            // set the function and the initial parameter values
            double pStart[4] = {1,1,1,1};
            fitter.SetFCN(fcn,pStart);
            // set step sizes different than default ones (0.3 times parameter values)
            for (int i = 0; i <4; ++i) fitter.Config().ParSettings(i).SetStepSize(0.01);

            bool ok = fitter.FitFCN();
            if (!ok) {
              Error("line3Dfit","Line3D Fit failed");
              return 1;
            }

             const ROOT::Fit::FitResult & result = fitter.Result();
             const ROOT::Math::Minimizer * min = fitter.GetMinimizer();
             double sigma2 = 25.0; //Size of the pad
             double Chi2_min = min->MinValue();
             int NDF = min->NFree();
             int npoints = gr->GetN();
             const double * parFitBuff = result.GetParams();
             std::vector<Double_t> parFit;
             for(Int_t i=0;i<4;i++) parFit.push_back(parFitBuff[i]); //4 parameters per fit
             track->SetFitPar(parFit);
             track->SetMinimum(Chi2_min);
             track->SetNFree(NDF);

             /*std::cout<<parFit[0]<<" "<<parFit[1]<<"  "<<parFit[2]<<" "<<parFit[3]<<std::endl;
 		     std::cout<<" Chi2 (Minuit) : "<<Chi2_min<<" NDF : "<<NDF<<std::endl;
			 std::cout<<" Chi2 reduced  : "<<(Chi2_min/sigma2/(double) npoints)<<std::endl;*/



			 std::vector<Double_t> p_f = track->GetFitPar();
			 XYZVector Z_1(0.0,0.0,1.0); // Beam direction

			 if(p_f.size()>0)
             {
                    XYZVector L0(p_f[0], p_f[2], 0. );//p2
				   	XYZVector L1(p_f[1], p_f[3], 1. );//d2	

				   	XYZVector vec1 = Z_1;
				   	XYZVector vec2 = L1;

				    Double_t num = vec1.X()*vec2.X() + vec1.Y()*vec2.Y() + vec1.Z()*vec2.Z() ;
  					Double_t den = TMath::Sqrt(vec1.X()*vec1.X() + vec1.Y()*vec1.Y() + vec1.Z()*vec1.Z())*TMath::Sqrt(vec2.X()*vec2.X() + vec2.Y()*vec2.Y() + vec2.Z()*vec2.Z());
  					Double_t ang = TMath::ACos(num/den);

					return ang;


			 }else return -1000;


}

double GetMaximum(double *adc)
{

	double max = 0;

	for(int indTB=0;indTB<512;++indTB)
	{
	    //std::cout<<" TB "<<indTB<<" adc "<<adc[indTB]<<"\n";
	    if(adc[indTB]>max) max = adc[indTB];
	}             	   			 

	return max;

}


void analysis_v2()
{


	gSystem->Load("libATTPCReco.so");
	gSystem->Load("libAtTpcMap.so");

	FairRunAna* run = new FairRunAna(); //Forcing a dummy run

	TString workdir = getenv("VMCWORKDIR");
    TString FileNameHead = "run_0080";
    TString FilePath = workdir + "/macro/Unpack_HDF5/S1845/";
    TString FileNameTail = ".root";
    TString FileNameOut  = "_analysis";
    TString FileName     = FilePath + FileNameHead + FileNameTail;
    TString OutputFileName = FilePath + FileNameHead + FileNameOut + FileNameTail;

    std::cout<<" Opening File : "<<FileName.Data()<<std::endl;
    TFile* file = new TFile(FileName.Data(),"READ");

    TTree* tree = (TTree*) file -> Get("cbmsim");
    Int_t nEvents = tree -> GetEntries();
    std::cout<<" Number of events : "<<nEvents<<std::endl;

    TTreeReader Reader1("cbmsim", file);
    TTreeReaderValue<TClonesArray> raweventArray(Reader1, "ATRawEvent");
    TTreeReaderValue<TClonesArray> eventArray(Reader1, "ATEventH");
    TTreeReaderValue<TClonesArray> patterneventArray(Reader1, "ATPatternEvent");

    TString scriptfile = "LookupProto20181201v2.xml";
    TString protomapfile = "proto20181201.map";
    TString scriptdir = workdir + "/scripts/"+ scriptfile;
    TString protomapdir = workdir + "/scripts/"+ protomapfile;
    TString geo = "proto20181201_geo_hires.root";

    AtTpcMap *fAtMapPtr = new AtTpcProtoMap();
    fAtMapPtr->ParseXMLMap(scriptdir.Data());
    fAtMapPtr->SetGeoFile(geo);
    fAtMapPtr->SetProtoMap(protomapdir.Data());

    TH2Poly* fPadPlane = fAtMapPtr->GetATTPCPlane("ATTPC_Proto");

    // Histograms

    TH2F* range_vs_Q = new TH2F("range_vs_Q","range_vs_Q",100,0,200,1000,0,1000000);

    TH2F* Q1_vs_Q2 = new TH2F("Q1_vs_Q2","Q1_vs_Q2",200,0,50000,200,0,50000);

    TH2F* Q1_vs_Q2_int = new TH2F("Q1_vs_Q2_int","Q1_vs_Q2_int",500,0,500000,500,0,500000);

    TH1F* angleH = new TH1F("angleH","angleH",100,0,360);

    TH2F* range_vs_angle = new TH2F("range_vs_angle","range_vs_angle",100,0,200,100,0,360);

    std::ofstream outputFile("experiment_bragg_events.txt");

    TH1F* mesh = new TH1F("mesh","mesh",512,0,511);

    TH1F* chi2minH = new TH1F("chi2minH","chi2minH",1000,0,8000);

    TH1F* Qtot = new TH1F("Qtot","Qtot",1000,0,1000000);

    double Qprot_ref = 169276.80; //Average total charge of 180 keV protons
    double Qlith_ref = 233000.00;

    double chi2_tree = 0;
      
    double stretch_tree = 0;  
    double shift_tree = 0;

    double chi2Li_tree = 0;
    double stretchLi_tree = 0;
    double shiftLi_tree = 0;


    double protonTrigger_tree = 0;
    double alphaTrigger_tree = 0;
    double angle_tree = 0;
    double range_tree = 0;
    double Qexp_tree = 0;
    double Qref_tree = 0;
    double QrefLi_tree = 0;
    int    eventNum_tree = 0;
    double Qtot_tree = 0;

    std::vector<double> exp_curve_tree;
    std::vector<double> ref_curve_tree;
    std::vector<double> refLi_curve_tree;

    TFile *analysisFile = new TFile(OutputFileName,"RECREATE");
    TTree *analysisTree = new TTree("analysisTree","analysis");
    analysisTree->Branch("chi2_tree",&chi2_tree,"chi2_tree/D");
    analysisTree->Branch("chi2Li_tree",&chi2Li_tree,"chi2Li_tree/D");
    analysisTree->Branch("Qref_tree",&Qref_tree,"Qref_tree/D");
    analysisTree->Branch("QrefLi_tree",&QrefLi_tree,"QrefLi_tree/D");
    analysisTree->Branch("Qexp_tree",&Qexp_tree,"Qexp_tree/D");
    analysisTree->Branch("angle_tree",&angle_tree,"angle_tree/D");
    analysisTree->Branch("stretch_tree",&stretch_tree,"stretch_tree/D");
    analysisTree->Branch("stretchLi_tree",&stretchLi_tree,"stretchLi_tree/D");
    analysisTree->Branch("range_tree",&range_tree,"range_tree/D");
    analysisTree->Branch("shift_tree",&shift_tree,"shift_tree/D");
    analysisTree->Branch("shiftLi_tree",&shiftLi_tree,"shiftLi_tree/D");
    analysisTree->Branch("protonTrigger_tree",&protonTrigger_tree,"protonTrigger_tree/D");
    analysisTree->Branch("alphaTrigger_tree",&alphaTrigger_tree,"alphaTrigger_tree/D");
    analysisTree->Branch("eventNum_tree",&eventNum_tree,"eventNum_tree/I");
    analysisTree->Branch("Qtot_tree",&Qtot_tree,"Qtot_tree/D");
    //analysisTree->Branch("exp_curve_tree",&exp_curve_tree);
    //analysisTree->Branch("ref_curve_tree",&ref_curve_tree);
    //analysisTree->Branch("refLi_curve_tree",&ref_curve_tree);

    std::ofstream outputFileBragg("bragg_collection.txt");

    std::ofstream outputFileQtotProton("Qtot_proton.txt");
    std::ofstream outputFileQtotAlpha("Qtot_alpha.txt");



     for(Int_t i=0;i<nEvents;i++){
          //while (Reader1.Next()) {

              Reader1.Next();

              ATEvent* event = (ATEvent*) eventArray->At(0);
              ATRawEvent *rawEvent = (ATRawEvent*) raweventArray->At(0);

          chi2_tree = 0;
          chi2Li_tree = 0;
    		  Qref_tree = 0;
    		  Qexp_tree = 0;
    		  stretch_tree = -10;
          stretchLi_tree = -10;
    		  angle_tree = 0;
    		  range_tree = 0;
    		  shift_tree = -400;
          shiftLi_tree = -400;
    		  protonTrigger_tree = 0;
    		  alphaTrigger_tree = 0;
    		  Qtot_tree = 0;

			    exp_curve_tree.clear();
    		  ref_curve_tree.clear();
          refLi_curve_tree.clear();
              

              mesh->Reset();

              if(event!=NULL)
              {
            
	              Int_t nHits = event->GetNumHits();
	              std::vector<ATHit>* hitArray = event->GetHitArray();
	              event->GetHitArrayObj();
	              std::cout<<" 	**** Event Number : "<<i<<" Event Q : "<<event->GetEventCharge()<<std::endl;
	              eventNum_tree = int(i);
	              //std::cout<<hitArray->size()<<"\n";
	              ATPatternEvent* patternEvent = (ATPatternEvent*) patterneventArray->At(0);
	              std::vector<ATTrack>& tracks = patternEvent->GetTrackCand();
	              //std::cout<<" Found tracks "<<tracks.size()<<"\n";

	              std::vector<ATPad> *padArray = rawEvent->GetPads();
	              std::vector<ATPad> *auxPadArray = event->GetAuxPadArray();


	              std::cout<<" Number of pads : "<<padArray->size()<<" - Number of auxiliary pads : "<<auxPadArray->size()<<"\n";

	              for(auto auxpad : *auxPadArray)
	              {
	              	if(auxpad.GetAuxName().compare(std::string("downscaled_alpha"))==0)
	              	{
	              		//std::cout<<" Auxiliary pad name "<<auxpad.GetAuxName()<<"\n";
	              		Double_t *adc = auxpad.GetADC();
	              		float max = GetMaximum(adc);
    				      	alphaTrigger_tree = max;


	              	}else if(auxpad.GetAuxName().compare(std::string("protons"))==0){
	              	
	              		//std::cout<<" Auxiliary pad name "<<auxpad.GetAuxName()<<"\n";
	              		Double_t *adc = auxpad.GetADC();
	              		float max = GetMaximum(adc);
    					      protonTrigger_tree = max;


	              	}


	              }

	              Qtot_tree = event->GetEventCharge();

                if(alphaTrigger_tree>300)
                  outputFileQtotAlpha << i <<"  "<<Qtot_tree<<"\n";
                else if(protonTrigger_tree>300)
                  outputFileQtotProton << i <<"  "<<Qtot_tree<<"\n";

	              Qtot->Fill(Qtot_tree);


	              bool isValid = true;

	              for(auto  track : tracks)
	              {
	              	//std::cout<<" - Number of hits : "<<track.GetHitArray()->size()<<"	- Is noise? : "<<track.GetIsNoise()<<"\n";

	              	if( track.GetHitArray()->size()>10 && track.GetIsNoise()==0 && tracks.size()<=2)
	              	{
	              		track.SortHitArrayTime();
	              		//std::cout<<" - Number of hits : "<<track.GetHitArray()->size()<<"	- Is noise? : "<<track.GetIsNoise()<<"\n";
	              		//std::cout<<" 		**** Track ID : "<<track.GetTrackID()<<"\n";
	              		std::vector<ATHit>*  hits = track.GetHitArray();

	              		ATHit first_hit = hits->front();
	              		ATHit last_hit  = hits->back();
	              		TVector3 first_pos = first_hit.GetPosition();
	              		TVector3 last_pos  = last_hit.GetPosition();

	              		double Q_nearFirst = 0;
	              		double Q_nearLast = 0;

	              		double Qint_nearFirst = 0;
	              		double Qint_nearLast = 0;

	              		double Qtot = 0;

	              		int firstTBOT = 1000;
	              		int lastTBOT  = -10;

	              		double chi2min = 0;
	              		double stretchmin = 0;
	              		double shiftmin = 0;

	              		Double_t angOut = GetAngle(&track);
	              	    Double_t angDegOut = angOut*180.0/TMath::Pi();


	              		for(auto hit : *hits)
	              		{

	              			TVector3 pos = hit.GetPosition();
	              			//std::cout<<pos.X()<<"	"<<pos.Y()<<"	"<<pos.Z()<<"	"<<hit.GetCharge()<<"\n";
	              			if( TMath::Sqrt(pos.X()*pos.X() + pos.Y()*pos.Y())>110 )
	              			{
	              				isValid = false;
	              				break;
	              			}else{
	              				fPadPlane->Fill(pos.X(), pos.Y(), hit.GetCharge());

	              				//std::cout<<" Hit pad num "<<hit.GetHitPadNum()<<"\n";

	              				Double_t* adc = GetPadWaveForm(hit.GetHitPadNum(),padArray);


	              				for(int indTB=0;indTB<512;++indTB)
	              	   			 {
	              	    				if(adc[indTB]>40.0){	              	    		
	              	    					mesh->Fill(indTB,adc[indTB]);
	              	    					Qtot+=adc[indTB];
	              	    					if(indTB<firstTBOT) firstTBOT = indTB;
	              	    					if(indTB>lastTBOT)  lastTBOT  = indTB;

	              	    					 /*if(Qtot_tree>180000){	           
	              								   outputFileBragg<<"	"<<indTB<<"		"<<adc[indTB]<<"	"<<Qtot_tree<<"		"<<i<<"		"<<angDegOut<<"\n";
	              							   }*/	            
	              	    				}	
	              				}

	              				double distFromFirst = TMath::Sqrt( TMath::Power(pos.X()-first_pos.X(),2) + TMath::Power(pos.Y()-first_pos.Y(),2) + TMath::Power(pos.Z()-first_pos.Z(),2) ); 
	              	    		double distFromLast = TMath::Sqrt( TMath::Power(pos.X()-last_pos.X(),2) + TMath::Power(pos.Y()-last_pos.Y(),2) + TMath::Power(pos.Z()-last_pos.Z(),2) );

	              	    		if(distFromFirst < track.GetLinearRange()/2.0){
	              	    			Q_nearFirst+=hit.GetCharge();

	              	    			for(int indTB=0;indTB<512;++indTB)
	              	    			{
	              	    				//std::cout<<indTB<<"	"<<adc[indTB]<<"\n";
	              	    				if(adc[indTB]>40.0) Qint_nearFirst+=adc[indTB];
	              	    			}	
	              	    		}else if(distFromLast < track.GetLinearRange()/2.0 ){
	              	    			Q_nearLast+=hit.GetCharge();

	              	    			for(int indTB=0;indTB<512;++indTB){
	              	    				if(adc[indTB]>40.0) Qint_nearLast+=adc[indTB];
	              	    			}
	              	    		}//else if

	              			}

	              			//std::cout<<Qint_nearFirst<<"	"<<Qint_nearLast<<"\n";


	              		}//hits loop

	              		//std::cout<<" 		**** Range : "<<track.GetLinearRange()<<"\n";


	              	    if(isValid)
	              	    {

	              	    	Double_t ang = GetAngle(&track);
	              	    	angleH->Fill(ang*180.0/TMath::Pi());
	              	    	//std::cout<<" Angle "<<ang*180.0/TMath::Pi()<<"\n";
	              	    	Double_t angDeg = ang*180.0/TMath::Pi();
	              	    	range_vs_angle->Fill(track.GetLinearRange(),angDeg);

	              	    	if(angDeg<80.0 && angDeg>10.0){


        	              	    	range_vs_Q->Fill(track.GetLinearRange(),event->GetEventCharge()); //For the moment only events where the clustering works 1 track + noise
        	              	    	Q1_vs_Q2->Fill(Q_nearFirst,Q_nearLast);
        	              	    	Q1_vs_Q2_int->Fill(Qint_nearFirst,Qint_nearLast);

                                int ioptionpart = 1;
                                double sumrefp = 169276.80;
                                double sumexp = Qtot;

        	              	    	double dummy_result = chi2fitImp(ioptionpart,mesh,sumrefp,sumexp,angDeg,firstTBOT,chi2min,shiftmin,stretchmin,ref_curve_tree, exp_curve_tree);
                                

        	              	    	std::cout<<" Chi2Min proton "<<chi2min<<" - sumref "<<sumrefp<<"\n";

        	              	    	chi2minH->Fill(chi2min);

        	              	    	chi2_tree = chi2min;   						        
            						        stretch_tree = stretchmin; 						        
            						        shift_tree = shiftmin;

                                chi2min = 0;
                                stretchmin = 0;
                                shiftmin = 0;

                                ioptionpart = 2;
                                double sumrefLi = 233000.00;
                                dummy_result = chi2fitImp(ioptionpart,mesh,sumrefLi,sumexp,angDeg,firstTBOT,chi2min,shiftmin,stretchmin,refLi_curve_tree, exp_curve_tree);


                                std::cout<<" Chi2Min 7Li "<<chi2min<<" - sumref "<<sumrefLi<<"\n";

                                chi2Li_tree = chi2min;                      
                                stretchLi_tree = stretchmin;                    
                                shiftLi_tree = shiftmin;


                                Qref_tree = sumrefp;
                                QrefLi_tree = sumrefLi;
                                Qexp_tree = Qtot;
                                angle_tree = angDeg;
                                range_tree = track.GetLinearRange();

            					         	//analysisTree->Fill(); //Fill tree with each track. 


        	              	    		 for(int indTB=0;indTB<512;++indTB)
        			 				          	 {
        									           outputFile<<indTB<<"	"<<mesh->GetBinContent(indTB)<<"	"<<mesh->GetBinError(indTB)<<"  "<<Qtot_tree<<"	"<<i<<"		"<<angDeg<<"\n";	

        		        					     }

	              	    	
	              	    	}//angle valid

	              	    	
	              	    }//Is valid	

	              	    

	              	}// track size/ number of hits/ number of tracks



	              }//tracks loop



	          }

	    analysisTree->Fill();

     }//for event loop

     analysisTree->Write();
     analysisFile->Close();

     outputFileBragg.close();
     outputFileQtotAlpha.close();
     outputFileQtotProton.close();

     TCanvas *c1 = new TCanvas();
     c1->Divide(2,2);
     c1->cd(1);
     range_vs_Q->Draw("zcol"); 
     c1->cd(2);
     fPadPlane -> Draw("COL L");
     fPadPlane -> SetMinimum(1.0);
     c1->cd(3);
     Q1_vs_Q2->Draw("zcol");
     c1->cd(4);
     Q1_vs_Q2_int->Draw("zcol");

     TCanvas *c2 = new TCanvas();
     c2->Divide(2,2);
     c2->cd(1);
     angleH->Draw();
	   c2->cd(2);
	   range_vs_angle->Draw("zcol");
	   c2->cd(3);
	   chi2minH->Draw();

	   TCanvas *c3 = new TCanvas();
	   Qtot->Draw();


     gStyle->SetOptStat(0);
     gStyle->SetPalette(103);
     gPad ->Update();        

}