#include "spline.h"  //cubic interpolation
//#include "TInverseMap.hh"
#include <unistd.h>

TGraph *graphtable;
	//------------------------------------------------------------------------------
/*	Double_t FindAngleBetweenTracks(const ROOT::Math::XYZVector& vec1,const ROOT::Math::XYZVector& vec2)
	{
		// NB:: The vector of hits is always sorted in time so the direction of porpagation of the particle is always positive.
		// This means that the angle will be always between 0 and 90 degree.
		// The vertex and the mean time of the track are needed to determine the direction of the track and then add 90 degrees.
		Double_t num = vec1.X()*vec2.X() + vec1.Y()*vec2.Y() + vec1.Z()*vec2.Z() ;
		Double_t den = TMath::Sqrt(vec1.X()*vec1.X() + vec1.Y()*vec1.Y() + vec1.Z()*vec1.Z())*TMath::Sqrt(vec2.X()*vec2.X() + vec2.Y()*vec2.Y() + vec2.Z()*vec2.Z());
		Double_t ang = TMath::ACos(num/den);
		return ang;
	}*/
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


	void runana_d2He()
	{

		SetERtable();

		FairRunAna* run = new FairRunAna(); //Forcing a dummy run
		ATd2HeAnalysis *d2heana = new ATd2HeAnalysis ();

		std::string digiFileName = "../outputFiles/attpcdigi_d2He_30000.root";
		//std::string digiFileName = "../output_debug.root";

		TFile* file = new TFile(digiFileName.c_str(),"READ");

		TTree* tree = (TTree*) file -> Get("cbmsim");
		Int_t nEvents = tree -> GetEntries();

		TTreeReader reader("cbmsim", file);
		TTreeReaderValue<TClonesArray> ransacArray(reader, "ATRansac");

		TFile* outfile;
		TString  outFileNameHead = "attpcana_30000.root";//attpcsim_d2He_test20k
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

		Double_t proton_mass = 1.0078250322 * 931.494;
		Double_t proj_mass = 14.008596359 * 931.494;
		Double_t target_mass = 2.01410177812 * 931.494;
		Double_t recoil_mass = 14.00307400443 * 931.494;
		Double_t he2_mass = 2.0 * proton_mass;
		Double_t Ekin_proj = 115.0 * 14.008596359;//100.0

		//std::vector<ATTrack> trackCand;
		//std::vector<ATRANSACN::ATRansac::PairedLines> lines;

		/// --------------------- Event loop -------------------------------------------

		for(Int_t i=0;i<nEvents;i++){

			reader.Next();
			//trackCand.clear();
			//lines.clear();
			ATRANSACN::ATRansac* fATRansac  = dynamic_cast<ATRANSACN::ATRansac*> (ransacArray->At(0));
			if(fATRansac==NULL) std::cout<<" Null pointer fATRansac "<<"\n";
			else {
			
			//@ransacArray->At(0)->Dump();
			
			std::vector<ATRANSACN::ATRansac::PairedLines> lines = fATRansac->GetPairedLinesArray();
			std::vector<ATTrack> trackCand = fATRansac->GetTrackCand();

			//----------------------- TPC tracks -------------------------------------------

			theta1=0.; theta2=0.; phi1=0.; phi2=0.; range_p1=0.; range_p2=0.; eLoss_p1_reco=0.; eLoss_p2_reco=0.; mom1_norm_reco=0.; mom2_norm_reco=0.; //reset variables
			E_tot_he2=0.; he2_mass_ex=0.; kin_He2=0.; theta_He2=0.; phi_He2=0.;theta_cm=0.; Ex4=0.;
		    	MaxR1=0.; MaxR2=0.; MaxZ1=0.; MaxZ2=0.;
			
			if(i%2!=0 && lines.size()>0){ //else std::cout<<" No pair lines "<<i<<"\n";
			
			TVector3 vertexMean = lines.at(0).meanVertex;	
			TVector3 lastPoint1 = trackCand.at(0).GetLastPoint();
			TVector3 lastPoint2 = trackCand.at(1).GetLastPoint();
			MaxR1=sqrt(pow(lastPoint1.X(),2)+pow(lastPoint1.Y(),2));
			MaxR2=sqrt(pow(lastPoint2.X(),2)+pow(lastPoint2.Y(),2));
			MaxZ1=lastPoint1.Z();
			MaxZ2=lastPoint2.Z();

		//	std::cout<<MaxR1<<" "<<MaxR2<<" "<<MaxZ1<<" "<<MaxZ2<<" "<<vertexMean.X()<<" "<<vertexMean.Y()<<" "<<vertexMean.Z()<<std::endl;
			
			if(MaxR1>=242. || MaxR2>=242. || MaxR1<=25. || MaxR2<=25. || MaxZ1>973. || MaxZ2>973.) continue; //if do no stop in chamber or in the beam hole
			lastX1 = lastPoint1.X();
			lastY1 = lastPoint1.Y();
			lastZ1 = lastPoint1.Z();
			lastX2 = lastPoint2.X();
			lastY2 = lastPoint2.Y();
			lastZ2 = lastPoint2.Z();
			vertexX = vertexMean.X();
			vertexY = vertexMean.Y();
			vertexZ = vertexMean.Z();

			theta1 = trackCand.at(0).GetThetaPhi(vertexMean, lastPoint1).first;
			theta2 = trackCand.at(1).GetThetaPhi(vertexMean, lastPoint2).first;
			phi1 = trackCand.at(0).GetThetaPhi(vertexMean, lastPoint1).second;
			phi2 = trackCand.at(1).GetThetaPhi(vertexMean, lastPoint2).second;

			std::vector<Double_t> fitPar1 = trackCand.at(0).GetFitPar();
			std::vector<Double_t> fitPar2 = trackCand.at(1).GetFitPar();

			TVector3 vp1(TMath::Sign(1,lastX1)*fabs(fitPar1[1]),TMath::Sign(1,lastY1)*fabs(fitPar1[3]),-TMath::Sign(1,(lastZ1-vertexZ))*fabs(fitPar1[5]));
			TVector3 vp2(TMath::Sign(1,lastX2)*fabs(fitPar2[1]),TMath::Sign(1,lastY2)*fabs(fitPar2[3]),-TMath::Sign(1,(lastZ2-vertexZ))*fabs(fitPar2[5]));
			angle12=FindAngleBetweenTracks(vp1,vp2);

			//std::cout<<i<<" "<<" protons 1 2 theta : "<<theta1*TMath::RadToDeg()<<" "<<theta2*TMath::RadToDeg()<<"\n";
			//std::cout<<i<<" protons 1 2 phi : "<<phi1*TMath::RadToDeg()<<" "<<phi2*TMath::RadToDeg()<<"\n";

			range_p1 = trackCand.at(0).GetLinearRange(vertexMean,lastPoint1);
			range_p2 = trackCand.at(1).GetLinearRange(vertexMean,lastPoint2);

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
		} //if lines.size()>0
	}//RANSAC null pointer
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

	outfile->Close();

} //end main
