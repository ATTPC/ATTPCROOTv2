//#include "spline.h"  //cubic interpolation
//#include "TInverseMap.hh"
#include <unistd.h>

TGraph *graphtable;

//change MTDC gate S800_timeMTDCXf.at(k)>215 && etc..
//change the Ekin_proj to the correct value,
//check the energy loss function
//check calibration/correction coef on the dE, and ToF S800.
//Is the referential of the experimental setup really what we use in simulation? If plots make no sens first check that


//static Double_t proton_mass = 1.0078250322 * 931.494;
static Double_t alpha_mass = 4.002603254 * 931.494;
static Double_t proj_mass = 69.9364313 * 931.494;
static Double_t target_mass = 4.002603254 * 931.494;
static Double_t recoil_mass = 4.002603254 * 931.494;
static Double_t ejectile_mass = 69.9364313 * 931.494;
//static Double_t he2_mass = 2.0 * proton_mass;
static Double_t Ekin_proj = 82.0 * 69.9364313;//100.0
//static Double_t Ekin_proj = 110.0 * 14.008596359;//100.0

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


void SetERtable(){//fit of the GEANT4 E vs R obtained from the simulation with the function model given by LISE++
		//ifstream fER("p_in_d_LISE_1atm.dat");//from LISE++, with this one comment the convertion um to mm
	//	ifstream fER("p_in_d_350torr.txt");//from LISE++,
		//ifstream fER("p_in_d_530torr.txt");//from LISE++,
		ifstream fER("he_in_he.txt");
		//ifstream fER("p_in_d_730torr.txt");//from LISE++,
		//ifstream fER("p_in_D2_LISE_530torr.dat");//from LISE++
		Double_t l1=0, l2=0, l3=0, l4=0, l5=0, l6=0, l7=0, l8=0, l9=0, l10=0;
		Int_t model=1;
		vector <vector<Double_t>> Energy_Range;

		for (string line; getline(fER, line);) {
			stringstream parse_die(line);
			vector<Double_t> iRE;
			parse_die >> l1 >> l2 >> l3 >> l4 >> l5>>l6>>l7>>l8>>l9>> l10;
			iRE.push_back(l1*4.002603254);//E in MeV
			iRE.push_back(l2/1000.);//range in mm, model 1
			iRE.push_back(l3/1000.);//range in mm, model 2
			iRE.push_back(l4/1000.);//range in mm, model 3
			iRE.push_back(l5/1000.);//range in mm, model 4

			Energy_Range.push_back(iRE);
		}
		fER.close();
		Int_t v_size = Energy_Range.size();
		Double_t X[v_size];
		Double_t Y[v_size];
		for(Int_t i=0; i<v_size; i++){
			X[i]=Energy_Range.at(i).at(0);
			Y[i]=Energy_Range.at(i).at(model);//*0.738 for LISE++ Eloss to match with GEANT4
			//<<X[i]<<" "<<Y[i]<<endl;
		}
		graphtable = new TGraph(v_size,Y,X);
	}


void analysis_jp(int runNumberS800, int runNumberATTPC)
	{
                fstream ofile;
                ofile.open("test_ev.txt",ios::app);
                 ofile<<"run number="<<runNumberATTPC<<endl;
                 ofile<<"evnt"<<"\t"<<"theta1"<<"\t"<<"eLoss_p1_reco"<<"\t"<<"theta_cm"<<"\t"<<"vertexZ"<<"\t"<<"range_p1"<<"\t"<<"Ex4"<<"\t"<<"S800_XfObj_tof"<<"\t"<<"S800_ObjCorr"<<std::endl;
		SetERtable();

		FairRunAna* run = new FairRunAna(); //Forcing a dummy run
		ATd2HeAnalysis *d2heana = new ATd2HeAnalysis ();

		// std::string digiFileName = "run_2021_0030.root";//merged.root
		TString digiFileName = TString::Format("/mnt/analysis/e18027/rootMerg/run_%04d_%04d.root", runNumberS800, runNumberATTPC);

		TFile* file = new TFile(digiFileName,"READ");

		TTree* tree = (TTree*) file -> Get("cbmsim");
		Int_t nEvents = tree -> GetEntries();

		S800Calc *s800cal = new S800Calc();
		TBranch *bS800cal = tree->GetBranch("s800cal");
	  bS800cal->SetAddress(&s800cal);

		TTreeReader reader("cbmsim", file);
		TTreeReaderValue<TClonesArray> ransacArray(reader, "ATRansac");

		TFile* outfile;
		// TString  outFileNameHead = "attpcana_merg_newRansac.root";
		TString outFileNameHead = TString::Format("/mnt/analysis/e18027/rootAna/runAnalyzed_%04d_%04d_jp.root", runNumberS800, runNumberATTPC);
		//TString outFileNameHead = TString::Format("runAnalyzed_%04d_%04d.root", runNumberS800, runNumberATTPC);

		outfile   = TFile::Open(outFileNameHead.Data(),"recreate"); 


		TH1F* scatteringAngle = new TH1F("scatteringAngle","scatteringAngle",1000,0,200);
		TH1F* energy = new TH1F("enegy","energy",100,0,40);
		TH2F* ang_vs_energy = new TH2F("ang_vs_energy","ang_vs_energy,",100,0,200,100,0,40);
		//TH2D *tracks_z_r = new TH2D ("tracks_z_r", "ZvsR", 500, -100, 1000, 500, 0, 300);
		//TH2D *tracks_x_y = new TH2D ("tracks_x_y", "XvsY", 500, -300, 300, 500, -300, 300);
		TH1F* thetacm = new TH1F("thetacm","thetacm",100,0,20);
		TH1D *Ex_reco = new TH1D ("Ex", "Ex", 400, -10, 30);
		TH2D *thetacm_Ex = new TH2D ("thetacm_Ex", "thetacm_Ex", 200, 0, 20, 400, -10, 30);
		TH2D *theta_range = new TH2D ("theta_range", "theta_range", 180, 0, 180, 250, 0, 600);
		TH2D *theta_kin = new TH2D ("theta_kin", "theta_kin", 180, 0, 180, 250, 0, 40);

		//----- S800
		// TH2D *tof_dE = new TH2D ("tof_dE", "tof_dE", 250, 1000, 1500, 250, 0, 500);//PID
		TH2D *XfpObj_tof = new TH2D ("XfpObj_tof", "XfpObj_tof", 500,-70,-20,600,250,280);//PID1
		TH2D *ICSum_Obj = new TH2D ("ICSum_Obj", "ICSum_Obj",500,-70,-20,1000,50,750);//PID2
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
		Double_t S800_XfObj_tof=0.,S800_ObjCorr=0., S800_Obj=0.;
		Double_t S800_ICSum=0.;
		Double_t MaxR1=0.0, MaxR2, MaxZ1, MaxZ2;
    		Int_t CondMTDCObj = 0,CondMTDCXfObj = 0;
		Double_t beam_theta=0., beam_phi=0.;


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
		anatree->Branch("beam_theta",&beam_theta);
		anatree->Branch("beam_phi",&beam_phi);
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
                 anatree->Branch("MaxR1",&MaxR1);
                 
		//----- S800
		anatree->Branch("S800_timeStamp",&S800_timeStamp,"S800_timeStamp/l");
		//anatree->Branch("S800_timeRf",&S800_timeRf);
		//anatree->Branch("S800_timeE1up",&S800_timeE1up);
		//anatree->Branch("S800_timeE1down",&S800_timeE1down);
		//anatree->Branch("S800_timeE1",&S800_timeE1);
		anatree->Branch("S800_timeXf",&S800_timeXf);
		anatree->Branch("S800_timeObj",&S800_timeObj);
		anatree->Branch("S800_tof",&S800_tof);
		anatree->Branch("S800_XfObj_tof",&S800_XfObj_tof);
		anatree->Branch("S800_ObjCorr",&S800_ObjCorr);
		anatree->Branch("S800_Obj",&S800_Obj);
		anatree->Branch("CondMTDCObj",&CondMTDCObj);
		anatree->Branch("CondMTDCXfObj",&CondMTDCXfObj);
		anatree->Branch("S800_ICSum",&S800_ICSum);

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
			s800cal->Clear();
			trackCand.clear();
			bS800cal->GetEntry(i);
			reader.Next();

			S800_timeStamp=0;
			S800_timeRf=0.;S800_x0=0.;S800_x1=0.;S800_y0=0.;S800_y1=0.;S800_E1up=0.;S800_E1down=0.;S800_tof=0.;
			S800_tofCorr=0.;S800_dE=0.;S800_dECorr=0.;S800_hodoSum=0.;S800_afp=0.;S800_bfp=0.;S800_ata=0.;
			S800_bta=0.;S800_yta=0.;S800_dta=0.;S800_thetaLab=0.;S800_phi=0.;
			S800_timeE1up=0.;S800_timeE1down=0.;S800_timeE1=0.;S800_timeXf=0.;S800_timeObj=0.;
			S800_tof=0.;


			// ATRANSACN::ATRansac* fATRansac  = dynamic_cast<ATRANSACN::ATRansac*> (ransacArray->At(0));
		  // ATRansacMod* fATRansac  = dynamic_cast<ATRansacMod*> (ransacArray->At(0));
		  // ATMlesacMod* fATRansac  = dynamic_cast<ATMlesacMod*> (ransacArray->At(0));
		  ATLmedsMod* fATRansac  = dynamic_cast<ATLmedsMod*> (ransacArray->At(0));
			 if(fATRansac==nullptr){
			  std::cout<<" Null pointer fATRansac "<<"\n";
				continue;
				}

			trackCand = fATRansac->GetTrackCand();

			//if(trackCand.size()>0) std::cout<<i<<" "<<s800cal->GetTS()<<"   "<<s800cal->GetSCINT(0)->GetDE()<<"  "<<trackCand.size()<<"  "<<s800cal->GetIsInCut()<<" "<<s800cal->GetCRDC(0)->GetX()<<std::endl;

			 //TClonesArray* cArray = (TClonesArray*) fRootManager->GetObject("ATEventH");
			 /*
			 ATEvent* cevent = (ATEvent*) eventArray->At(0);
			 if(cevent==nullptr){
			  std::cout<<" Null pointer cevent "<<"\n";
			  continue;
			  }
			 Bool_t gated = cevent->IsExtGate();
			 */

			if(trackCand.size()>0 && s800cal->GetIsInCut()==kTRUE){
			//if(trackCand.size()==0){
			//if(trackCand.size()>0 && gated==kTRUE) {
			//if(trackCand.size()>0){
                    // std::cout<<"event ="<<i<<"\t";

				//----------------------- S800 -------------------------------------------------

    CondMTDCObj = 0;
    CondMTDCXfObj = 0;
    vector<Float_t> S800_timeMTDCObj = s800cal->GetMultiHitTOF()->GetMTDCObj();
    vector<Float_t> S800_timeMTDCXf = s800cal->GetMultiHitTOF()->GetMTDCXf();
    Float_t S800_timeObjSelect=-999;
    Float_t S800_timeXfSelect=-999;
    S800_XfObj_tof = -999.;
    S800_ObjCorr = -999;
    S800_Obj = -999;
    Double_t ObjCorr1C1 = 100.; //70//100.
    Double_t ObjCorr1C2 = 0.009; //0.0085//0.021

    S800_ICSum = s800cal->GetIC()->GetSum();
    S800_x0 = s800cal->GetCRDC(0)->GetX();
    S800_x1 = s800cal->GetCRDC(1)->GetX();
    S800_y0 = s800cal->GetCRDC(0)->GetY();
    S800_y1 = s800cal->GetCRDC(1)->GetY();
    S800_timeStamp = s800cal->GetTS();

			//  S800_timeRf = s800cal->GetMultiHitTOF()->GetFirstRfHit();
			//  S800_timeE1up = s800cal->GetMultiHitTOF()->GetFirstE1UpHit();
			//  S800_timeE1down = s800cal->GetMultiHitTOF()->GetFirstE1DownHit();
			//  S800_timeE1 = sqrt( (corrGainE1up*S800_timeE1up) * (corrGainE1down*S800_timeE1down) );
			//  S800_timeXf = s800cal->GetMultiHitTOF()->GetFirstXfHit();
			//  S800_timeObj = s800cal->GetMultiHitTOF()->GetFirstObjHit();

			  S800_E1up = s800cal->GetSCINT(0)->GetDEup();
			  S800_E1down = s800cal->GetSCINT(0)->GetDEdown();

			  //S800_tof = S800_timeObj - S800_timeE1;
			  //XfObj_tof = S800_timeXf - S800_timeObj;

				S800_tof = S800_timeRf;//might change
				S800_afp = atan( (S800_x1-S800_x0)/1073. );
				S800_bfp = atan( (S800_y1-S800_y0)/1073. );


    for(int k=0; k<S800_timeMTDCXf.size(); k++){
    	//if(S800_timeMTDCXf.at(k)>140 && S800_timeMTDCXf.at(k)<205) S800_timeXfSelect=S800_timeMTDCXf.at(k);
    	if(S800_timeMTDCXf.at(k)>170 && S800_timeMTDCXf.at(k)<230) S800_timeXfSelect=S800_timeMTDCXf.at(k);
    }
    for(int k=0; k<S800_timeMTDCObj.size(); k++){
    	//if(S800_timeMTDCObj.at(k)>-120 && S800_timeMTDCObj.at(k)<50) S800_timeObjSelect=S800_timeMTDCObj.at(k);
    	if(S800_timeMTDCObj.at(k)>-75 && S800_timeMTDCObj.at(k)<0) S800_timeObjSelect=S800_timeMTDCObj.at(k);
    }

    if(S800_timeObjSelect!=-999){
	CondMTDCObj=1;
	S800_Obj = S800_timeObjSelect;
	S800_ObjCorr = S800_timeObjSelect + ObjCorr1C1*S800_afp + ObjCorr1C2*S800_x0;

    }
    if(S800_timeXfSelect!=-999 && S800_timeObjSelect!=-999) {
    	S800_XfObj_tof=S800_timeXfSelect-S800_timeObjSelect;
	CondMTDCXfObj=1;
    }
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


    //std::cout<<" "<<S800_timeObjSelect<<" "<<XfObj_tof<<" "<<S800_ICSum<<std::endl;


				// tof_dE->Fill(S800_tof,S800_dE);
				//ICSum_Obj->Fill(S800_ObjCorr,S800_ICSum);
		    //XfpObj_tof->Fill(S800_XfObj_tof,S800_ObjCorr);

				dta_ata->Fill(S800_dta,S800_ata* TMath::RadToDeg());//ata in deg
				x_y_crdc1->Fill(S800_x0,S800_y0);
				x_y_crdc2->Fill(S800_x1,S800_y1);

			//std::cout<<i<<" "<<" "<<S800_timeStamp<<" "<<S800_timeRf<<" "<<S800_y0<<std::endl;


					for(Int_t w=0;w<trackCand.size();w++){

                                                
						theta1=0.;  phi1=0.;  range_p1=0.;  eLoss_p1_reco=0.;  mom1_norm_reco=0.;  //reset variables
						theta_cm=0.; Ex4=0.;
				  	MaxR1=0.;  MaxZ1=0.;
						beam_theta=0., beam_phi=0.;
                                              
						TVector3 vertexMean = trackCand.at(w).GetTrackVertex();
	      		TVector3 lastPoint1 = trackCand.at(w).GetLastPoint();
	      		MaxR1=sqrt(pow(lastPoint1.X(),2)+pow(lastPoint1.Y(),2));
	      		MaxZ1=lastPoint1.Z();
						beam_theta = vertexMean.Theta()* TMath::RadToDeg();
						beam_phi = vertexMean.Phi()* TMath::RadToDeg();





						//std::cout<<s800cal->GetTS()<<" "<<MaxR1<<std::endl;

						//if(MaxR1>=242. || MaxR2>=242. || MaxR1<=25. || MaxR2<=25. || MaxZ1>973. || MaxZ2>973.) continue; //if do no stop in chamber or in the beam hole
						//if(MaxR1>=242. || MaxR2>=242.  || MaxZ1>973. || MaxZ2>973.) continue; //if do no stop in chamber or in the beam hole



						//------------------------------------------------------------------------------

	        	lastX1 = lastPoint1.X();
	        	lastY1 = lastPoint1.Y();
	        	lastZ1 = lastPoint1.Z();
	        	vertexX = vertexMean.X();
	        	vertexY = vertexMean.Y();
	        	vertexZ = vertexMean.Z();

	        	theta1 = trackCand.at(w).GetThetaPhi(vertexMean, lastPoint1).first;
	        	phi1 = trackCand.at(w).GetThetaPhi(vertexMean, lastPoint1).second;

						//std::vector<Double_t> fitPar1 = trackCand.at(w).GetFitPar();

	        	range_p1 = trackCand.at(w).GetLinearRange(vertexMean,lastPoint1);
	        	eLoss_p1_reco = graphtable->Eval(range_p1);

						theta_range->Fill(theta1* TMath::RadToDeg(),range_p1);
						theta_kin->Fill(theta1* TMath::RadToDeg(),eLoss_p1_reco);

	        		/*
	          // reconstruction of 2He
	        	mom1_norm_reco = TMath::Sqrt(eLoss_p1_reco * eLoss_p1_reco + 2.0 * eLoss_p1_reco * proton_mass);
	        	mom_proton1_reco.SetX (mom1_norm_reco * TMath::Sin(theta1) * TMath::Cos(phi1));
	        	mom_proton1_reco.SetY (mom1_norm_reco * TMath::Sin(theta1) * TMath::Sin(phi1));
	        	mom_proton1_reco.SetZ (mom1_norm_reco * TMath::Cos(theta1));
						*/

						//if((theta1*180/3.1415>20 && theta1*180/3.1415<80) || (theta1*180/3.1415>100 && theta1*180/3.1415<160) && (range_p1<230))theta_r_he2_reco->Fill(theta1*180/3.1415);

	        	d2heana->kine_2b (proj_mass, target_mass, recoil_mass, ejectile_mass, Ekin_proj, theta1, eLoss_p1_reco);
	        	theta_cm = d2heana->GetThetaCM ();
	        	Ex4 = d2heana->GetMissingMass ();
                        double E_alpha = eLoss_p1_reco;
//std::cout<<"remove +0.2 later"<<endl;
                        double th_alpha_lab= theta1*TMath::RadToDeg();

                        //std::cout<<"Theta="<<th_alpha_lab<<"\t"<<range_p1/10.0<<"\t"<<eLoss_p1_reco<<"\t"<<MaxR1<<std::endl;
	        	thetacm->Fill (theta_cm);
	        	Ex_reco->Fill (Ex4);
	        	thetacm_Ex->Fill (theta_cm, Ex4);

	        	ivt=i;
                        ofile<<i<<"\t"<<theta1<<"\t"<<eLoss_p1_reco<<"\t"<<theta_cm<<"\t"<<vertexZ<<"\t"<<range_p1<<"\t"<<Ex4<<"\t"<<S800_XfObj_tof<<"\t"<<S800_ObjCorr<<std::endl;
	        	anatree->Fill();

				}//all tracks
		} //gated and  tracks.size()>1
	}// Event loop
        ofile.close();

	/// --------------------- End event loop ---------------------------------------

	anatree->Write();
	//	tracks_z_r->Write ();
	//	tracks_x_y->Write ();

	thetacm_Ex->Write ();
	theta_range->Write();
	theta_kin->Write();
	thetacm->Write();
	Ex_reco->Write();
	//tof_dE->Write();
	dta_ata->Write();
	x_y_crdc1->Write();
	x_y_crdc2->Write();
 theta_kin->Draw();

	outfile->Close();

} //end main
