Double_t fitref(int x)
{
	double c4=x;
		
	double c6=1580.*TMath::Exp(TMath::Power(-((c4-197.)/16.),2)) + 1550.*TMath::Exp(TMath::Power(-((c4-224.)/11.),2)) 
		   + 650.*TMath::Exp(TMath::Power(-((c4-211.)/10.),2)) - 150.*TMath::Exp(TMath::Power(-((c4-221.)/6.),2)) 
		   + 70.*TMath::Exp(TMath::Power(-((c4-214.)/5.),2));
	
	double fitref=c6*2.0;

	return fitref;


}

Double_t chi2fit(TH1F* mesh, double Qrefproton, double Qrefexp, double angle,int iTb, double &chimin,double &shiftmin, double &stretchmin)
{

	int    dzero=50; //initial step when minimizing
	double dstretch=0.5; 
	double stretch0=0.;  //shifts the calc to exp for ibegin
	int    shift0=200.-iTb;
	int    istretchmax=20;
	int    izeromax=20;
	double chimin=1.e6;
	int    itb0=200;  //bricolage, to be replaced by center of gravity but probably not important

	double chi2[100][100];

		for(int iter=0;iter<2;++iter)
		{

			for(int istretch=0;istretch<istretchmax;++istretch)
			{

				for(int izero=0;izero<izeromax;++izero)
				{
					chi2[istretch][izero]=0.;

						for(int itb=100;itb<400;++itb)
						{

							int shift      = dzero*(float(izero-izeromax/2.0)/float(izeromax));
							shift          = shift+shift0; //  !could be better limited this is the tb of the evnt
							double stretch = dstretch*(float(istretch-istretchmax/2.0)/float(istretchmax)) + stretch0;
							int tbref      = itb+shift+stretch*(itb-itb0); //!this is the coordinate of the analytical function before angle correction
							tbref          = 200.+ (tbref-200.)/cos(angle*0.01745); // !zero degree=beam=tbucket direction
							double fitfit  = fitref(tbref)/cos(angle*0.01745);// !function fitted at point bref corrected for projection on exp axis

							double sigma          = (mesh->GetBinContent(itb)+fitfit)*0.1+100.;//  !error to be tested
				 			sigma                 = 100.;//  !error to be tested
				 			chi2[istretch][izero] = chi2[istretch][izero]+ TMath::Power(((mesh->GetBinContent(itb)*Qrefproton/Qrefexp-fitfit)/sigma),2) ;// !now with renormalized

						}//TB

						if( chi2[istretch][izero]<chimin)
						{
							chimin= chi2[istretch][izero];
							stretchmin=stretch;
							shiftmin=shift;
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


void analysis()
{

	FairRunAna* run = new FairRunAna(); //Forcing a dummy run

	TString workdir = getenv("VMCWORKDIR");
    TString FileNameHead = "run_0141";
    TString FilePath = workdir + "/macro/Unpack_HDF5/S1845/";
    TString FileNameTail = ".root";
    TString FileName     = FilePath + FileNameHead + FileNameTail;

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

    double Qprot_ref = 169276.80; //Average total charge of 180 keV protons


     for(Int_t i=0;i<10;i++){
          //while (Reader1.Next()) {

              Reader1.Next();

              ATEvent* event = (ATEvent*) eventArray->At(0);
              ATRawEvent *rawEvent = (ATRawEvent*) raweventArray->At(0);

              

              mesh->Reset();

              if(event!=NULL)
              {
            
	              Int_t nHits = event->GetNumHits();
	              std::vector<ATHit>* hitArray = event->GetHitArray();
	              event->GetHitArrayObj();
	              std::cout<<" 	**** Event Number : "<<i<<" Event Q : "<<event->GetEventCharge()<<std::endl;
	              //std::cout<<hitArray->size()<<"\n";
	              ATPatternEvent* patternEvent = (ATPatternEvent*) patterneventArray->At(0);
	              std::vector<ATTrack>& tracks = patternEvent->GetTrackCand();
	              std::cout<<" Found tracks "<<tracks.size()<<"\n";

	              std::vector<ATPad> *padArray = rawEvent->GetPads();

	              std::cout<<" Number of pads : "<<padArray->size()<<"\n";

	              bool isValid = true;

	              for(auto  track : tracks)
	              {
	              	//std::cout<<" - Number of hits : "<<track.GetHitArray()->size()<<"	- Is noise? : "<<track.GetIsNoise()<<"\n";

	              	if( track.GetHitArray()->size()>10 && track.GetIsNoise()==0 && tracks.size()==2)
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
	              	    	std::cout<<" Angle "<<ang*180.0/TMath::Pi()<<"\n";
	              	    	Double_t angDeg = ang*180.0/TMath::Pi();
	              	    	range_vs_angle->Fill(track.GetLinearRange(),angDeg);

	              	    	if(angDeg<55.0 && angDeg>35.0 &&  track.GetLinearRange()>80.0){

	              	    	range_vs_Q->Fill(track.GetLinearRange(),event->GetEventCharge()); //For the moment only events where the clustering works 1 track + noise
	              	    	Q1_vs_Q2->Fill(Q_nearFirst,Q_nearLast);
	              	    	Q1_vs_Q2_int->Fill(Qint_nearFirst,Qint_nearLast);

	              	    	double dummy_result = chi2fit(mesh,Qprot_ref,Qtot,angDeg,firstTBOT,chi2min,stretchmin,shiftmin);

	              	    		 for(int indTB=0;indTB<512;++indTB)
			 					 {
									outputFile<<indTB<<"	"<<mesh->GetBinContent(indTB)<<"	"<<mesh->GetBinError(indTB)<<"	"<<i<<"		"<<angDeg<<"\n";	


							     }

	              	    	
	              	    	}	

	              	    	
	              	    }	

	              	    

	              	}



	              }//tracks loop



	          }   

     }//for loop

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

     gStyle->SetOptStat(0);
     gStyle->SetPalette(103);
     gPad ->Update();        

}