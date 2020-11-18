//#include "spline.h"  //cubic interpolation
//#include "TInverseMap.hh"
#include <unistd.h>

TGraph *graphtable;

//change the Ekin_proj to the correct value,
//check the energy loss function
//check calibration/correction coef on the dE, and ToF S800.
//Is the referential of the experimental setup really what we use in simulation? If plots make no sens first check that


static Double_t proton_mass = 1.0078250322 * 931.494;
static Double_t proj_mass = 14.008596359 * 931.494;
static Double_t target_mass = 2.01410177812 * 931.494;
static Double_t recoil_mass = 14.00307400443 * 931.494;
static Double_t he2_mass = 2.0 * proton_mass;
static Double_t Ekin_proj = 115.0 * 14.008596359;//100.0

static Double_t corrGainE1up = 1.;
static Double_t corrGainE1down = 1.;
static double x0_corr_tof = 0.0;
static double afp_corr_tof = 0.0;
static double rf_offset = 19320.0;
static double afp_corr_dE = 0.0;
static double x0_corr_dE = 0.0;
static double bta_corr = 1.0;
static double coeff_hodo[32] = {100000000, 1164, 887, 100000000, 824, 1728, 826,
	100000000, 941, 962, 875, 915, 875, 835, 1079, 842, 855, 870, 891, 1786, 960,
	940, 825, 625, 990, 2195, 805, 870, 575, 1004, 1164, 748 };

std::vector <double> get_invmap_vars(TInverseMap *inv_map, double x0, double y0, double afp, double bfp)
	{
		int order;
		double sinb_sina;
		vector <double> outPuts;

		order = 5;

		outPuts.push_back(inv_map->Ata(order, x0, afp, y0, bfp));//ata
		outPuts.push_back(inv_map->Bta(order, x0, afp, y0, bfp) * bta_corr);//bta
		outPuts.push_back(inv_map->Yta(order, x0, afp, y0, bfp) * 1000.);//yta
		outPuts.push_back(inv_map->Dta(order, x0, afp, y0, bfp));//dta
		outPuts.push_back(atan(sqrt(pow(tan(outPuts.at(0)), 2) + pow(tan(outPuts.at(1)), 2))));//theta_lab
		outPuts.push_back(atan(tan(outPuts.at(1))/tan(outPuts.at(0))));//phi
		if (outPuts.at(0) < 0)
		outPuts.at(5) = 3.141592653 + outPuts.at(5);
		else if (outPuts.at(1) < 0)
		outPuts.at(5) = 2*3.141592653 + outPuts.at(5);

		return outPuts;
	}


Double_t FindAngleBetweenTracks(const TVector3 &vec1,const TVector3 &vec2)
    {
        Double_t ang = vec1.Angle(vec2);
      	return ang;
    }

double eloss_approx(double rangep){//rangep in mm, elossp in MeV; get this approximated function by fitting the GEANT4 E vs R obtained with the simulation
		double elossp;
		if(rangep<131.) elossp =  0.0833284+ 0.0118757*rangep -7.3698e-05*pow(rangep,2)+  2.42758e-07*pow(rangep,3);
		if(rangep>=131.) elossp =  0.36519+ 0.004761*rangep -4.28371e-06*pow(rangep,2)+ 2.35727e-09*pow(rangep,3);
		return elossp;
	}


void SetERtable(){//fit of the GEANT4 E vs R obtained from the simulation with the function model given by LISE++
		ifstream fER("p_in_d_LISE_1atm.dat");//from LISE++
		Double_t l1=0, l2=0, l3=0, l4=0, l5=0;
		Int_t model=1;
		vector <vector<Double_t>> Energy_Range;

		for (string line; getline(fER, line);) {
			stringstream parse_die(line);
			vector<Double_t> iRE;
			parse_die >> l1 >> l2 >> l3 >> l4 >> l5;
			iRE.push_back(l1);//E in MeV
			iRE.push_back(l2);//range in mm, model 1
			iRE.push_back(l3);//range in mm, model 2
			iRE.push_back(l4);//range in mm, model 3
			iRE.push_back(l5);//range in mm, model 4
			Energy_Range.push_back(iRE);
		}
		fER.close();
		Int_t v_size = Energy_Range.size();
		Double_t X[v_size];
		Double_t Y[v_size];
		for(Int_t i=0; i<v_size; i++){
			X[i]=Energy_Range.at(i).at(0);
			Y[i]=Energy_Range.at(i).at(model)*0.738;//0.738 for LISE++ Eloss to match with GEANT4
			//cout<<X[i]<<" "<<Y[i]<<endl;
		}
		graphtable = new TGraph(v_size,Y,X);
	}


void analysis_reprocess()
	{

		SetERtable();

		FairRunAna* run = new FairRunAna(); //Forcing a dummy run
		ATd2HeAnalysis *d2heana = new ATd2HeAnalysis ();

		// std::string digiFileName = "run_2021_0030.root";//merged.root
		//TString digiFileName = TString::Format("/mnt/analysis/e18008/rootMerg/run_%04d_%04d.root", runNumberS800, runNumberATTPC);
		TString digiFileName = "salida_ran.root";

		TFile* file = new TFile(digiFileName,"READ");

		TTree* tree = (TTree*) file -> Get("cbmsim");
		Int_t nEvents = tree -> GetEntries();

		/*
		S800Calc *s800cal = new S800Calc();
		TBranch *bS800cal = tree->GetBranch("s800cal");
	  bS800cal->SetAddress(&s800cal);
		*/
		// TChain* inputdata = new TChain("cbmsim");
  	// inputdata->Add(digiFileName);
		// inputdata->LoadTree(0);
  	//inputdata->Print();
  	//std::cout<<inputdata->GetListOfFiles()<<std::endl;
		// TTreeReader inputfilereader(inputdata);

		TTreeReader reader("cbmsim", file);
		// TTreeReaderValue<S800Calc> s800Calc(reader, "s800cal");
		TTreeReaderValue<TClonesArray> ransacArray(reader, "ATRansac");
		TTreeReaderValue<TClonesArray> eventArray(reader, "ATEventH");

		TFile* outfile;
		// TString  outFileNameHead = "attpcana_merg_newRansac.root";
		//TString outFileNameHead = TString::Format("/mnt/analysis/e18008/rootAna/runAnalyzed_%04d_%04d.root", runNumberS800, runNumberATTPC);
		TString outFileNameHead = "salidaAnalyzed.root";

		outfile   = TFile::Open(outFileNameHead.Data(),"recreate");

		TH1F* scatteringAngle = new TH1F("scatteringAngle","scatteringAngle",1000,0,200);
		TH1F* energy = new TH1F("enegy","energy",100,0,40);
		TH2F* ang_vs_energy = new TH2F("ang_vs_energy","ang_vs_energy,",100,0,200,100,0,40);
		//TH2D *tracks_z_r = new TH2D ("tracks_z_r", "ZvsR", 500, -100, 1000, 500, 0, 300);
		//TH2D *tracks_x_y = new TH2D ("tracks_x_y", "XvsY", 500, -300, 300, 500, -300, 300);
		TH1D *theta_r_he2_reco = new TH1D ("theta_r_he2_reco", "theta 2He", 1800, 0, 180);
		TH1D *kin_r_he2_reco = new TH1D ("kin_r_he2_reco", "Energy 2He", 100, 0, 5);
		TH1D *phi_r_he2_reco = new TH1D ("phi_r_he2_reco", "phi 2He", 3600, -180, 180);
		TH2D *theta_kin_he2_reco = new TH2D ("theta_kin_he2_reco", "Kin vs Theta 2He", 1800, 0, 180, 100, 0, 5);
		TH1D *thetacm_he2_reco = new TH1D ("thetacm_he2_reco", "thetacm_he2", 200, 0, 20);
		TH1D *Ex_reco = new TH1D ("Ex_reco", "Ex_reco", 350, -5, 30);
		TH2D *thetacm_Ex_he2_reco = new TH2D ("thetacm_Ex_he2_reco", "thetacm_Ex_he2", 200, 0, 20, 350, -5, 30);
		TH1D *ex_he2_reco = new TH1D ("ex_he2_reco", "ex_he2", 100, 0, 10);
		TH1D *epsilon_pp_reco = new TH1D ("epsilon_pp_reco", "#epsilon_{pp}", 100, 0, 10);
		TH2D *thetacm_epsilon_pp_reco = new TH2D ("thetacm_epsilon_pp_reco", "#theta_{cm} #epsilon_{pp} #^{2}He", 200, 0, 20, 100, 0, 10);
		//----- S800

		// TH2D *tof_dE = new TH2D ("tof_dE", "tof_dE", 250, 1000, 1500, 250, 0, 500);//PID
		TH2D *XfpObj_tof = new TH2D ("XfpObj_tof", "XfpObj_tof", 3000, -250, 500, 2000, 0, 500);//PID1
		TH2D *dE_tof = new TH2D ("dE_tof", "dE_tof", 3000, -250, 500, 2000, 0, 500);//PID2
		TH2D *dta_ata = new TH2D ("dta_ata", "dta_ata", 250, -0.25, 0.25, 100, -10, 10);//acceptance
		TH2D *x_y_crdc1 = new TH2D ("x_y_crdc1", "x_dy_crdc1", 300, -300, 300, 150, -150, 150);//positions crdc1
		TH2D *x_y_crdc2 = new TH2D ("x_y_crdc2", "x_dy_crdc2", 300, -300, 300, 150, -150, 150);//positions crdc2
		//-----
		Int_t ivt = 0,ivt_same=0;
		Double_t range_p1 = 0.,range_p2 =0.;
		Double_t eLoss_p1_reco = 0.0, eLoss_p2_reco = 0.0;
		Double_t epsilon_pp = -999;
		Double_t theta1=0., theta2=0., phi1=0., phi2=0., angle12=0.;
		Double_t mom1_norm_reco=0., mom2_norm_reco=0.;
		Double_t E_tot_he2=0., he2_mass_ex=0.;
		Double_t kin_He2=0., theta_He2=0.,kin_He2_same=0., theta_He2_same=0., phi_He2=0.;
		Double_t theta_cm=0., Ex4=0., Ex_reco_same=0.;
		Double_t lastX1=0.,lastX2=0.,lastY1=0.,lastY2=0.,lastZ1=0.,lastZ2=0., vertexX=0., vertexY=0., vertexZ=0.;
		Double_t ata=0., dta=0.;
		ULong64_t S800_timeStamp=0;
		Double_t S800_timeRf=0.,S800_x0=0.,S800_x1=0.,S800_y0=0.,S800_y1=0.,S800_E1up=0.,S800_E1down=0.,S800_tof=0.,
		S800_tofCorr=0.,S800_dE=0.,S800_dECorr=0.,S800_hodoSum=0.,S800_afp=0.,S800_bfp=0.,S800_ata=0.,S800_bta=0.,
		S800_yta=0.,S800_dta=0.,S800_thetaLab=0.,S800_phi=0.,S800_timeE1up=0.,S800_timeE1down=0.,S800_timeE1=0.,S800_timeXf=0.,S800_timeObj=0.;
		Double_t XfObj_tof=0.;
		Double_t MaxR1, MaxR2, MaxZ1, MaxZ2;

		TVector3 mom_proton1_reco, mom_proton2_reco,mom_He2_reco;

		TTree *anatree = new TTree("anatree","new TTree");

		anatree->Branch("ivt",&ivt);
		anatree->Branch("range_p1",&range_p1);
		anatree->Branch("range_p2",&range_p2);
		anatree->Branch("theta1",&theta1);
		anatree->Branch("theta2",&theta2);
		anatree->Branch("phi1",&phi1);
		anatree->Branch("phi2",&phi2);
		anatree->Branch("lastX1",&lastX1);
		anatree->Branch("lastY1",&lastY1);
		anatree->Branch("lastZ1",&lastZ1);
		anatree->Branch("lastX2",&lastX2);
		anatree->Branch("lastY2",&lastY2);
		anatree->Branch("lastZ2",&lastZ2);
		anatree->Branch("vertexX",&vertexX);
		anatree->Branch("vertexY",&vertexY);
		anatree->Branch("vertexZ",&vertexZ);
		anatree->Branch("angle12",&angle12);
		anatree->Branch("eLoss_p1_reco",&eLoss_p1_reco);
		anatree->Branch("eLoss_p2_reco",&eLoss_p2_reco);
		anatree->Branch("kin_He2",&kin_He2);
		anatree->Branch("theta_He2",&theta_He2);
		anatree->Branch("E_tot_he2",&E_tot_he2);
		anatree->Branch("he2_mass_ex",&he2_mass_ex);
		anatree->Branch("theta_cm",&theta_cm);
		anatree->Branch("Ex4",&Ex4);
		anatree->Branch("epsilon_pp",&epsilon_pp);
		anatree->Branch("mom1_norm_reco",&mom1_norm_reco);
		anatree->Branch("mom2_norm_reco",&mom2_norm_reco);
		//----- S800
		anatree->Branch("S800_timeStamp",&S800_timeStamp,"S800_timeStamp/l");
		anatree->Branch("S800_timeRf",&S800_timeRf);
		anatree->Branch("S800_timeE1up",&S800_timeE1up);
		anatree->Branch("S800_timeE1down",&S800_timeE1down);
		anatree->Branch("S800_timeE1",&S800_timeE1);
		anatree->Branch("S800_timeXf",&S800_timeXf);
		anatree->Branch("S800_timeObj",&S800_timeObj);
		anatree->Branch("S800_tof",&S800_tof);
		anatree->Branch("XfObj_tof",&XfObj_tof);

		anatree->Branch("S800_x0",&S800_x0);
		anatree->Branch("S800_x1",&S800_x1);
		anatree->Branch("S800_y0",&S800_y0);
		anatree->Branch("S800_y1",&S800_y1);
		anatree->Branch("S800_E1up",&S800_E1up);
		anatree->Branch("S800_E1down",&S800_E1down);
		anatree->Branch("S800_tof",&S800_tof);
		anatree->Branch("S800_tofCorr",&S800_tofCorr);
		anatree->Branch("S800_dE",&S800_dE);
		anatree->Branch("S800_dECorr",&S800_dECorr);
		anatree->Branch("S800_hodoSum",&S800_hodoSum);
		anatree->Branch("S800_afp",&S800_afp);
		anatree->Branch("S800_bfp",&S800_bfp);
		anatree->Branch("S800_ata",&S800_ata);
		anatree->Branch("S800_bta",&S800_bta);
		anatree->Branch("S800_yta",&S800_yta);
		anatree->Branch("S800_dta",&S800_dta);
		anatree->Branch("S800_thetaLab",&S800_thetaLab);
		anatree->Branch("S800_phi",&S800_phi);


		std::vector<ATTrack> trackCand;


		//----------------------- S800 -------------------------------------------------

		std::string mapFile="inv_map.inv";
		TInverseMap *inv_map = new TInverseMap(mapFile.c_str());


		/// --------------------- Event loop -------------------------------------------

		for(Int_t i=0;i<nEvents;i++){
			//s800cal->Clear();
			trackCand.clear();
			//bS800cal->GetEntry(i);
			reader.Next();

			S800_timeStamp=0;
			S800_timeRf=0.;S800_x0=0.;S800_x1=0.;S800_y0=0.;S800_y1=0.;S800_E1up=0.;S800_E1down=0.;S800_tof=0.;
			S800_tofCorr=0.;S800_dE=0.;S800_dECorr=0.;S800_hodoSum=0.;S800_afp=0.;S800_bfp=0.;S800_ata=0.;
			S800_bta=0.;S800_yta=0.;S800_dta=0.;S800_thetaLab=0.;S800_phi=0.;
			S800_timeE1up=0.;S800_timeE1down=0.;S800_timeE1=0.;S800_timeXf=0.;S800_timeObj=0.;
			S800_tof=0.;XfObj_tof=0.;


			// ATRANSACN::ATRansac* fATRansac  = dynamic_cast<ATRANSACN::ATRansac*> (ransacArray->At(0));
		   ATRansacMod* fATRansac  = dynamic_cast<ATRansacMod*> (ransacArray->At(0));
		  // ATMlesacMod* fATRansac  = dynamic_cast<ATMlesacMod*> (ransacArray->At(0));
		  //ATLmedsMod* fATRansac  = dynamic_cast<ATLmedsMod*> (ransacArray->At(0));
			 if(fATRansac==nullptr){
			  std::cout<<" Null pointer fATRansac "<<"\n";
				continue;
				}

			trackCand = fATRansac->GetTrackCand();

			 // std::cout<<s800cal->GetTS()<<"   "<<s800cal->GetSCINT(0)->GetDE()<<"  "<<trackCand.size()<<"  "<<s800cal->GetIsInCut()<<std::endl;

			 //TClonesArray* cArray = (TClonesArray*) fRootManager->GetObject("ATEventH");
	     ATEvent* cevent = (ATEvent*) eventArray->At(0);
	     Bool_t gated = cevent->IsExtGate();


			//if(trackCand.size()>1 && s800cal->GetIsInCut()==kTRUE){
			if(trackCand.size()>1 && gated==kTRUE) {

				/*
				//----------------------- S800 -------------------------------------------------
				S800_timeStamp = s800cal->GetTS();

				S800_x0 = s800cal->GetCRDC(0)->GetX();
				S800_x1 = s800cal->GetCRDC(1)->GetX();
				// S800_x0 = s800cal->GetCRDC(0)->GetXfit();
				// S800_x1 = s800cal->GetCRDC(1)->GetXfit();
				S800_y0 = s800cal->GetCRDC(0)->GetY();
				S800_y1 = s800cal->GetCRDC(1)->GetY();

				S800_timeRf = s800cal->GetMultiHitTOF()->GetFirstRfHit();
			  S800_timeE1up = s800cal->GetMultiHitTOF()->GetFirstE1UpHit();
			  S800_timeE1down = s800cal->GetMultiHitTOF()->GetFirstE1DownHit();
			  S800_timeE1 = sqrt( (corrGainE1up*S800_timeE1up) * (corrGainE1down*S800_timeE1down) );
			  S800_timeXf = s800cal->GetMultiHitTOF()->GetFirstXfHit();
			  S800_timeObj = s800cal->GetMultiHitTOF()->GetFirstObjHit();

			  S800_E1up = s800cal->GetSCINT(0)->GetDEup();
			  S800_E1down = s800cal->GetSCINT(0)->GetDEdown();

			  S800_tof = S800_timeObj - S800_timeE1;
			  XfObj_tof = S800_timeXf - S800_timeObj;

				S800_tof = S800_timeRf;//might change
				S800_afp = atan( (S800_x1-S800_x0)/1073. );
				S800_bfp = atan( (S800_y1-S800_y0)/1073. );
				S800_tofCorr = S800_tof + x0_corr_tof*S800_x0 + afp_corr_tof*S800_afp;// - rf_offset;
				// S800_dE = s800cal->GetSCINT(0)->GetDE();//check if is this scint (0)
				S800_dE = sqrt( (corrGainE1up*S800_E1up) * (corrGainE1down* S800_E1down ) );
				S800_dECorr = S800_dE + afp_corr_dE*S800_afp + x0_corr_dE*fabs(S800_x0);
				for (Int_t j=0; j<32; j++) if (s800cal->GetHODOSCOPE(j)->GetEnergy()>=10 && s800cal->GetHODOSCOPE(j)->GetEnergy()<=4000) S800_hodoSum += s800cal->GetHODOSCOPE(j)->GetEnergy()*3000./coeff_hodo[j];
				std::vector <double> S800_invMapOut = get_invmap_vars(inv_map,S800_x0,S800_y0,S800_afp,S800_bfp);
				S800_ata = S800_invMapOut.at(0);
				S800_bta = S800_invMapOut.at(1);
				S800_yta = S800_invMapOut.at(2);
				S800_dta = S800_invMapOut.at(3);
				S800_thetaLab= S800_invMapOut.at(4);
				S800_phi= S800_invMapOut.at(5);

				// tof_dE->Fill(S800_tof,S800_dE);
				dE_tof->Fill(S800_tofCorr,S800_dECorr);
		    XfpObj_tof->Fill(S800_tofCorr,XfObj_tof);

				dta_ata->Fill(S800_dta,S800_ata*180./TMath::Pi());//ata in deg
				x_y_crdc1->Fill(S800_x0,S800_y0);
				x_y_crdc2->Fill(S800_x1,S800_y1);

			//std::cout<<i<<" "<<" "<<S800_timeStamp<<" "<<S800_timeRf<<" "<<S800_y0<<std::endl;
			*/

				std::vector< std::pair <int,int> >  two_p_pair;
				two_p_pair.clear();

				//multi track analysis
				for(Int_t w=0;w<trackCand.size()-1;w++){
						TVector3 vertex0 = trackCand.at(w).GetTrackVertex();
						for(Int_t z=w+1; z<trackCand.size();z++){
									TVector3 vertex1 = trackCand.at(z).GetTrackVertex();
									if(vertex0==vertex1) two_p_pair.push_back(std::make_pair(w,z));
							}
					}

				/*
				//only 2 protons per event
				two_p_pair.clear();
				two_p_pair.push_back(std::make_pair(0,1));
				*/

				for(Int_t w=0;w<two_p_pair.size();w++){


					theta1=0.; theta2=0.; phi1=0.; phi2=0.; range_p1=0.; range_p2=0.; eLoss_p1_reco=0.; eLoss_p2_reco=0.; mom1_norm_reco=0.; mom2_norm_reco=0.; //reset variables
					E_tot_he2=0.; he2_mass_ex=0.; kin_He2=0.; theta_He2=0.; phi_He2=0.;theta_cm=0.; Ex4=0.;
			  	MaxR1=0.; MaxR2=0.; MaxZ1=0.; MaxZ2=0.;

					TVector3 vertexMean = trackCand.at(two_p_pair[w].first).GetTrackVertex();
      		TVector3 lastPoint1 = trackCand.at(two_p_pair[w].first).GetLastPoint();
      		TVector3 lastPoint2 = trackCand.at(two_p_pair[w].second).GetLastPoint();
      		MaxR1=sqrt(pow(lastPoint1.X(),2)+pow(lastPoint1.Y(),2));
      		MaxR2=sqrt(pow(lastPoint2.X(),2)+pow(lastPoint2.Y(),2));
      		MaxZ1=lastPoint1.Z();
      		MaxZ2=lastPoint2.Z();

					//std::cout<<s800cal->GetTS()<<" "<<MaxR1<<std::endl;

					if(MaxR1>=242. || MaxR2>=242. || MaxR1<=25. || MaxR2<=25. || MaxZ1>973. || MaxZ2>973.) continue; //if do no stop in chamber or in the beam hole



					//------------------------------------------------------------------------------

        	lastX1 = lastPoint1.X();
        	lastY1 = lastPoint1.Y();
        	lastZ1 = lastPoint1.Z();
        	lastX2 = lastPoint2.X();
        	lastY2 = lastPoint2.Y();
        	lastZ2 = lastPoint2.Z();
        	vertexX = vertexMean.X();
        	vertexY = vertexMean.Y();
        	vertexZ = vertexMean.Z();

        	theta1 = trackCand.at(two_p_pair[w].first).GetThetaPhi(vertexMean, lastPoint1).first;
        	theta2 = trackCand.at(two_p_pair[w].second).GetThetaPhi(vertexMean, lastPoint2).first;
        	phi1 = trackCand.at(two_p_pair[w].first).GetThetaPhi(vertexMean, lastPoint1).second;
        	phi2 = trackCand.at(two_p_pair[w].second).GetThetaPhi(vertexMean, lastPoint2).second;

        	std::vector<Double_t> fitPar1 = trackCand.at(two_p_pair[w].first).GetFitPar();
        	std::vector<Double_t> fitPar2 = trackCand.at(two_p_pair[w].second).GetFitPar();

        	TVector3 vp1(TMath::Sign(1,lastX1)*fabs(fitPar1[1]),TMath::Sign(1,lastY1)*fabs(fitPar1[3]),-TMath::Sign(1,(lastZ1-vertexZ))*fabs(fitPar1[5]));
        	TVector3 vp2(TMath::Sign(1,lastX2)*fabs(fitPar2[1]),TMath::Sign(1,lastY2)*fabs(fitPar2[3]),-TMath::Sign(1,(lastZ2-vertexZ))*fabs(fitPar2[5]));
        	angle12=FindAngleBetweenTracks(vp1,vp2);

					//std::cout<<i<<" "<<" protons 1 2 theta : "<<theta1<<" "<<theta2<<"\n";
					//std::cout<<i<<" protons 1 2 phi : "<<phi1<<" "<<phi2<<"\n";

        	range_p1 = trackCand.at(two_p_pair[w].first).GetLinearRange(vertexMean,lastPoint1);
        	range_p2 = trackCand.at(two_p_pair[w].second).GetLinearRange(vertexMean,lastPoint2);

        	//==============================================================================
        	// methods to get the proton eloss

        	//eLoss_p1_reco = eloss_approx(range_p1);
        	//eLoss_p2_reco = eloss_approx(range_p2);

        	eLoss_p1_reco = graphtable->Eval(range_p1);
        	eLoss_p2_reco = graphtable->Eval(range_p2);

        	//std::cout<<i<<" vertex : "<<vertexZ<<"\n";
        	//std::cout<<i<<" range p1 p2 : "<<range_p1<<" "<<range_p2<<"\n";
        	//std::cout<<i<<" eloss reco : "<<eLoss_p1_reco<<" "<<eLoss_p2_reco<<" "<<"\n";

        	//==============================================================================

        	epsilon_pp = 0.5*(eLoss_p1_reco + eLoss_p2_reco - 2 * sqrt(eLoss_p1_reco * eLoss_p2_reco) * TMath::Cos (angle12));
        	epsilon_pp_reco->Fill(epsilon_pp);

          // reconstruction of 2He
        	mom1_norm_reco = TMath::Sqrt(eLoss_p1_reco * eLoss_p1_reco + 2.0 * eLoss_p1_reco * proton_mass);
        	mom_proton1_reco.SetX (mom1_norm_reco * TMath::Sin(theta1) * TMath::Cos(phi1));
        	mom_proton1_reco.SetY (mom1_norm_reco * TMath::Sin(theta1) * TMath::Sin(phi1));
        	mom_proton1_reco.SetZ (mom1_norm_reco * TMath::Cos(theta1));

        	mom2_norm_reco = TMath::Sqrt (eLoss_p2_reco * eLoss_p2_reco + 2.0 * eLoss_p2_reco * proton_mass);
        	mom_proton2_reco.SetX (mom2_norm_reco * TMath::Sin(theta2) * TMath::Cos(phi2));
        	mom_proton2_reco.SetY (mom2_norm_reco * TMath::Sin(theta2) * TMath::Sin(phi2));
        	mom_proton2_reco.SetZ (mom2_norm_reco * TMath::Cos(theta2));
        	//std::cout<<i<<" mom1 : "<<mom_proton1_reco.Mag()<<"\n";
        	//std::cout<<i<<" mom2 : "<<mom_proton2_reco.Mag()<<"\n";

        	mom_He2_reco = mom_proton1_reco + mom_proton2_reco;
        	E_tot_he2 = (proton_mass + eLoss_p1_reco) + (proton_mass + eLoss_p2_reco);
        	he2_mass_ex = TMath::Sqrt (E_tot_he2 * E_tot_he2 - mom_He2_reco.Mag2 ());
        	ex_he2_reco->Fill (he2_mass_ex - he2_mass);

        	kin_He2 = TMath::Sqrt (mom_He2_reco.Mag2 () + he2_mass_ex * he2_mass_ex) - he2_mass_ex;
        	theta_He2 = mom_He2_reco.Theta ()* TMath::RadToDeg();

        	phi_He2 = mom_He2_reco.Phi ()* TMath::RadToDeg();
        	theta_r_he2_reco->Fill (theta_He2);
        	phi_r_he2_reco->Fill (phi_He2);
        	kin_r_he2_reco->Fill (kin_He2);
        	theta_kin_he2_reco->Fill (theta_He2, kin_He2);

        	d2heana->kine_2b (proj_mass, target_mass, he2_mass_ex, recoil_mass, Ekin_proj, theta_He2 * TMath::DegToRad (), kin_He2);

        	theta_cm = d2heana->GetThetaCM ();
        	Ex4 = d2heana->GetMissingMass ();

        	thetacm_he2_reco->Fill (theta_cm);
        	Ex_reco->Fill (Ex4);
        	thetacm_Ex_he2_reco->Fill (theta_cm, Ex4);
        	thetacm_epsilon_pp_reco->Fill(theta_cm,epsilon_pp);

        	ivt=i;
        	anatree->Fill();
				}//two proton pairs
		} //gated and  tracks.size()>1
	}// Event loop


	/// --------------------- End event loop ---------------------------------------
	anatree->Write();

	//	tracks_z_r->Write ();
	//	tracks_x_y->Write ();
	theta_r_he2_reco->Write ();
	phi_r_he2_reco->Write ();
	kin_r_he2_reco->Write ();
	theta_kin_he2_reco->Write ();
	thetacm_he2_reco->Write ();
	Ex_reco->Write ();
	ex_he2_reco->Write ();
	thetacm_Ex_he2_reco->Write ();
	thetacm_epsilon_pp_reco->Write();
	epsilon_pp_reco->Write();

	//tof_dE->Write();
	dta_ata->Write();
	x_y_crdc1->Write();
	x_y_crdc2->Write();


	outfile->Close();

} //end main
