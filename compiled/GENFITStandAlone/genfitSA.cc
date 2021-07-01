#include "genfitSA.h"
#include "AtFitter.h"
#include "AtGenfit.h"

#include <chrono>
#include <thread>
#include <iostream>

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

int main()
{
  // -----   Timer   --------------------------------------------------------
  TStopwatch timer;
  timer.Start();

  std::vector<TString> files;
  
  
   //Analysis parameters
   Float_t magneticField = 3.0;//T
   Float_t dMass         = 2.0135532;
   Float_t pMass         = 1.00727646;
   Float_t Be10Mass      = 10.013533818;
   Float_t Be11Mass      = 11.021657749;
   Int_t atomicNumber    = 1;
   Int_t pPDGCode        = 2212;
   Int_t dPDGCode        = 1000010020;
   Int_t Be10PDGCode     = 1000040100;
   Int_t Be11PDGCode     = 1000040110;
   Int_t BeAtomicNumber  = 4;

   Int_t   particlePDG   = 2212;
   Float_t particleMass  = 0;
   Int_t   recoilPDG     = 0;
   Float_t recoilMass    = 0;
   
   switch(particlePDG)
     {
       case 2212:
	 std::cout<<cGREEN<<" Analyzing 10Be(d,p)11Be (PDG: 2212) "<<cNORMAL<<"\n";
	 particleMass = pMass;
	 recoilMass   = Be11Mass;
	 recoilPDG    = 1000040110;
	 files.push_back("output_digi_ctest.root");
	 break;
     case 1000010020:
       std::cout<<cGREEN<<" Analyzing 10Be(d,d)10Be (PDG: 1000010020) "<<cNORMAL<<"\n";
         particleMass = dMass;
	 recoilMass   = Be10Mass;
	 recoilPDG    = 1000040100;
	 files.push_back("output_digi_el.root");
       break;
     }
   
   const Double_t M_Ener = particleMass * 931.49401 / 1000.0;

   //Q value calculation
   //Q-value calculation
   Double_t m_p    = 1.007825*931.49401;
   Double_t m_d    = 2.0135532*931.49401;
   Double_t m_Be10 = 10.013533818*931.49401;
   Double_t m_Be11 = 11.021657749*931.49401;
   Double_t m_beam = m_Be10;

   Double_t Ebeam_buff = 100.0;//(EnergyRecoil + EnergySca + ex_energy[iFile]);
		
   // Histograms
   TH1F *angle = new TH1F("angle", "angle", 720, 0, 179);
   TH1F *phi = new TH1F("phi", "phi", 1440, -359, 359);
   TH1F *phi_pattern = new TH1F("phi_pattern", "phi_pattern", 1440, -359, 359);
   TH2F *phi_phi_pattern = new TH2F("phi_phi_pattern", "phi_phi_pattern", 720, -359, 359, 720, -359, 359);
   TH1F *momentum = new TH1F("momentum", "momentum", 1000, 0, 2.0); // GeV
   TH2F *angle_vs_momentum = new TH2F("angle_vs_momentum", "angle_vs_momentum", 720, 0, 179, 1000, 0, 2.0);
   TH2F *pos_vs_momentum = new TH2F("pos_vs_momentum", "pos_vs_momentum", 200, 0, 200, 1000, 0, 2.0);
   TH2F *length_vs_momentum = new TH2F("length_vs_momentum", "length_vs_momentum", 200, 0, 200, 1000, 0, 2.0);
   TH2F *hits_vs_momentum = new TH2F("hits_vs_momentum", "hits_vs_momentum", 200, 0, 200, 1000, 0, 2.0);
   TH2F *angle_vs_energy = new TH2F("angle_vs_energy", "angle_vs_energy", 720, 0, 179, 1000, 0, 100.0);
   TH2F *angle_vs_energy_pattern =
      new TH2F("angle_vs_energy_pattern", "angle_vs_energy_pattern", 720, 0, 179, 1000, 0, 100.0);
   TH1F* HQval = new TH1F("HQval","HQval",1000,-10,10);

   TH1F* eRes = new TH1F("eRes", "eRes", 100, -2.0, 2.0);

   //Paths
   TString dir = getenv("VMCWORKDIR");
   TString geoManFile  = dir + "/geometry/ATTPC_D1bar_v2_geomanager.root";
   std::cout << " Geometry file : " << geoManFile.Data() << "\n";
   TString filePath ="/macro/Simulation/ATTPC/10Be_dp/";
   TString fileName = "output_digi.root";
   //std::vector<TString> files{"output_digi_el_20deg.root"};
   //std::vector<TString> files{"output_digi_el.root","output_digi_inel.root","output_digi.root"};
   //std::vector<TString> files{"output_digi_ctest.root"};
   
   TString fileNameWithPath = dir + filePath + fileName;

   FairRunAna *run = new FairRunAna();
   run->SetGeomFile(geoManFile.Data());
   TFile *file = new TFile(fileNameWithPath.Data(), "READ");

   // GENFIT geometry
   new TGeoManager("Geometry", "ATTPC geometry");
   TGeoManager::Import(geoManFile.Data());
   genfit::MaterialEffects::getInstance()->init(new genfit::TGeoMaterialInterface());
   genfit::FieldManager::getInstance()->init(new genfit::ConstField(0., 0.,magneticField*10.0)); //

   // event display
   genfit::EventDisplay *display = genfit::EventDisplay::getInstance();

   AtFITTER::AtFitter *fFitter = new AtFITTER::AtGenfit(magneticField, 0.00001, 1000.0);
   dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetPDGCode(particlePDG);
   dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetMass(particleMass);
   dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetAtomicNumber(atomicNumber);
   dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetNumFitPoints(1.0);
   dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetVerbosityLevel(1);


   
   for (auto iFile = 0; iFile < files.size(); ++iFile) {

      fileNameWithPath = dir + filePath + files.at(iFile).Data();
      std::cout << " Opening File : " << fileNameWithPath.Data() << std::endl;

      file = new TFile(fileNameWithPath.Data(), "READ");

      TTree *tree = (TTree *)file->Get("cbmsim");
      Int_t nEvents = 200; // tree->GetEntries();
      std::cout << " Number of events : " << nEvents << std::endl;

      TTreeReader Reader1("cbmsim", file);
      TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtPatternEvent");
      // TTreeReaderValue<std::vector<genfit::Track>> fitterVector(Reader1, "ATTPC");

      for (Int_t i = 0; i < nEvents; i++) {

	
	std::chrono::seconds tickingBomb( 10 ) ;
	std::chrono::time_point<std::chrono::system_clock> end;
	end = std::chrono::system_clock::now() + tickingBomb;
	
	//if(i%2==0)
	  //continue;
	
	std::cout << cGREEN << " Event Number : " << i << cNORMAL << "\n";

         Reader1.Next();

         AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray->At(0);

         if (patternEvent) {

            std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
            std::cout << " Number of pattern tracks " << patternTrackCand.size() << "\n";

            for (auto track : patternTrackCand) {

	       std::cout << " Track " << track.GetTrackID() << " with " << track.GetHitClusterArray()->size()
                         << " clusters "
                         << "\n";

	       if (track.GetIsNoise() || track.GetHitClusterArray()->size() < 5)
                  continue;

	       
               TVector3 iniPos =
                  track.GetHitClusterArray()->back().GetPosition(); // TODO Check first cluster is the first in time
	       //NB: back/front for forward/backward direction, respectively
	       std::cout << " Initial position : " << iniPos.X() << " - " << iniPos.Y() << " - " << iniPos.Z() << "\n";

               //Double_t dist = TMath::Sqrt(iniPos.X() * iniPos.X() + iniPos.Y() * iniPos.Y());

               //if (dist > 20.0)
	       //continue;

               fFitter->Init();
	       genfit::Track *fitTrack;
	       
	       try{
                 fitTrack = fFitter->FitTracks(&track);
	       } catch (std::exception &e) {
                  std::cout << " Exception fitting track !" << e.what() << "\n";
                  continue;
	       }
	       
	       if(fitTrack==nullptr)
	         continue;
		 
	       TVector3 pos_res;
               TVector3 mom_res;
               TMatrixDSym cov_res;
	       Double_t pVal = 0;
	       Double_t bChi2 = 0, fChi2 = 0, bNdf = 0, fNdf = 0;

	       
               try {

                  if (fitTrack && fitTrack->hasKalmanFitStatus()) {

                     auto KalmanFitStatus = fitTrack->getKalmanFitStatus();

                     if (KalmanFitStatus->isFitConverged(false)) {
		       //KalmanFitStatus->Print();
                        genfit::MeasuredStateOnPlane fitState = fitTrack->getFittedState();
                        //fitState.Print();
                        fitState.getPosMomCov(pos_res, mom_res, cov_res);
			pVal = KalmanFitStatus -> getPVal();
			//fKalmanFitter -> getChiSquNdf(gfTrack, trackRep, bChi2, fChi2, bNdf, fNdf);

			//Building histograms
                        display->addEvent(fitTrack);
			
			Double_t thetaA = 0.0;
			if(track.GetGeoTheta()>90.0*TMath::DegToRad()){
			  thetaA = 180.0 * TMath::DegToRad() - mom_res.Theta();
			}else
			  thetaA = mom_res.Theta();
		 
                        angle->Fill(thetaA * TMath::RadToDeg());
                        // std::cout<<" Angle "<<mom_res.Theta()<<"\n";
                        auto pos_radial = TMath::Sqrt(TMath::Power(pos_res.X(), 2) + TMath::Power(pos_res.Y(), 2));
                        momentum->Fill(mom_res.Mag());
                        angle_vs_momentum->Fill(thetaA * TMath::RadToDeg(), mom_res.Mag());
                        pos_vs_momentum->Fill(pos_res.Mag(), mom_res.Mag());
                        auto len = fitTrack->getTrackLen();
                        length_vs_momentum->Fill(len, mom_res.Mag());
                        auto numHits = fitTrack->getNumPoints();
                        hits_vs_momentum->Fill(numHits, mom_res.Mag());
                        Double_t E = TMath::Sqrt(TMath::Power(mom_res.Mag(), 2) + TMath::Power(M_Ener, 2)) - M_Ener;
                        angle_vs_energy->Fill(180.0-thetaA * TMath::RadToDeg(), E * 1000.0);
                        phi->Fill(mom_res.Phi() * TMath::RadToDeg());

			Double_t Qval = E*1000.0*(1 + m_d/m_Be10) - Ebeam_buff*(1 - m_beam/m_Be10) - (2.0/m_Be10)*TMath::Sqrt(Ebeam_buff*E*1000*m_d)*TMath::Cos(mom_res.Theta());

                        //Excitation energy
			Double_t ex_energy_exp = kine_2b(m_Be10,m_d,m_d,m_Be10,Ebeam_buff,mom_res.Theta(),E*1000);
			//std::cout<<" Pos res Z "<<pos_res.Z()<<"\n";
			if(pos_res.Z()<30 )HQval->Fill(ex_energy_exp);

			//Getting information from points
			/*for(auto iPoint = 0;iPoint<fitTrack->getNumPoints();++iPoint)
			  {
			    genfit::TrackPoint* trackPointMnF = fitTrack->getPointWithMeasurementAndFitterInfo(iPoint);
			    if(trackPointMnF->getKalmanFitterInfo())
			      {
				genfit::KalmanFitterInfo* fitterInfo = trackPointMnF->getKalmanFitterInfo();
				//genfit::MeasuredStateOnPlane pointFitState = fitterInfo->getFittedState();
				//genfit::MeasurementOnPlane* pointMeasurement = fitterInfo->getMeasurementOnPlane();
				genfit::MeasuredStateOnPlane* pointPrediction = fitterInfo->getPrediction(1);
				TVector3 pos_res_p;
				TVector3 mom_res_p;
				TMatrixDSym cov_res_p;
				//if(fitterInfo->hasMeasurements()){
				pointPrediction->getPosMomCov(pos_res_p, mom_res_p, cov_res_p);
				//std::cout<<" Point "<<iPoint<<" - Momemtum : "<<mom_res_p.Mag()<<" - Theta : "<<mom_res_p.Theta()*TMath::RadToDeg()<<" - Phi : "<<mom_res_p.Phi()*TMath::RadToDeg()<<"\n"; 
				//}
			      }	
			      }*/  
                     }
                  }
               } catch (std::exception &e) {
                  std::cout << " " << e.what() << "\n";
                  continue;
		  }

	       
               Double_t theta =  180.0 * TMath::DegToRad() - track.GetGeoTheta();
	       Double_t radius = track.GetGeoRadius() / 1000.0;    // mm to m
               Double_t phi = track.GetGeoPhi();
               Double_t brho = magneticField * radius / TMath::Sin(theta); // Tm
               std::tuple<Double_t, Double_t> mom_ener = GetMomFromBrho(particleMass, atomicNumber, brho);
               angle_vs_energy_pattern->Fill(theta * TMath::RadToDeg(), std::get<1>(mom_ener) * 1000.0);
               phi_pattern->Fill(phi * TMath::RadToDeg());
               phi_phi_pattern->Fill(phi * TMath::RadToDeg(), mom_res.Phi() * TMath::RadToDeg());
	       
            } // track loop

         } // if pattern event

      } // Event

   } // File

   // Adding kinematic lines
   Double_t *ThetaCMS = new Double_t[20000];
   Double_t *ThetaLabRec = new Double_t[20000];
   Double_t *EnerLabRec = new Double_t[20000];
   Double_t *ThetaLabSca = new Double_t[20000];
   Double_t *EnerLabSca = new Double_t[20000];
   Double_t *MomLabRec = new Double_t[20000];

   TString fileKine = "../Be10pp_el.txt";
   std::ifstream *kineStr = new std::ifstream(fileKine.Data());
   Int_t numKin = 0;

   if (!kineStr->fail()) {
      while (!kineStr->eof()) {
         *kineStr >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_AngRec_EnerRec = new TGraph(numKin, ThetaLabRec, EnerLabRec);

   fileKine = "../Be10pp_in_2+1.txt";
   std::ifstream *kineStr2 = new std::ifstream(fileKine.Data());
   numKin = 0;

   if (!kineStr2->fail()) {
      while (!kineStr2->eof()) {
         *kineStr2 >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr2->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;  
     
   TGraph *Kine_AngRec_EnerRec_in = new TGraph(numKin, ThetaLabRec, EnerLabRec);

   fileKine = "../Be10dp_gs.txt";
   std::ifstream *kineStr3 = new std::ifstream(fileKine.Data());
   numKin = 0;

   if (!kineStr3->fail()) {
      while (!kineStr3->eof()) {
         *kineStr3 >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr3->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;  
     
   TGraph *Kine_AngRec_EnerRec_dp = new TGraph(numKin, ThetaLabRec, EnerLabRec);

   fileKine = "../Be10dp_first.txt";
   std::ifstream *kineStr4 = new std::ifstream(fileKine.Data());
   numKin = 0;

   if (!kineStr4->fail()) {
      while (!kineStr4->eof()) {
         *kineStr4 >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr4->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;  
     
   TGraph *Kine_AngRec_EnerRec_dp_first = new TGraph(numKin, ThetaLabRec, EnerLabRec);

   
   TCanvas *c1 = new TCanvas();
   c1->Divide(2, 2);
   c1->Draw();
   c1->cd(1);
   angle->Draw();
   c1->cd(2);
   momentum->Draw();
   c1->cd(3);
   angle_vs_momentum->Draw();
   c1->cd(4);
   pos_vs_momentum->Draw();

   TCanvas *cKine = new TCanvas();
   cKine->Divide(1, 2);
   cKine->cd(1);
   cKine->Draw();
   angle_vs_energy->SetMarkerStyle(20);
   angle_vs_energy->SetMarkerSize(0.5);
   angle_vs_energy->Draw();
   Kine_AngRec_EnerRec->SetLineWidth(1);
   Kine_AngRec_EnerRec->SetLineColor(kRed);
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_AngRec_EnerRec_in->SetLineWidth(1);
   Kine_AngRec_EnerRec_in->SetLineColor(kBlue);
   Kine_AngRec_EnerRec_in->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_dp->SetLineWidth(1);
   Kine_AngRec_EnerRec_dp->SetLineColor(kGreen);
   Kine_AngRec_EnerRec_dp->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_dp_first->SetLineWidth(1);
   Kine_AngRec_EnerRec_dp_first->SetLineColor(kViolet);
   Kine_AngRec_EnerRec_dp_first->Draw("ZCOL SAME");
   cKine->cd(2);
   angle_vs_energy_pattern->Draw();
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_AngRec_EnerRec_in->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_dp->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_dp_first->Draw("ZCOL SAME");

   TCanvas *c2 = new TCanvas();
   c2->Divide(2, 2);
   c2->Draw();
   c2->cd(1);
   length_vs_momentum->Draw();
   c2->cd(2);
   hits_vs_momentum->Draw();

   c2->cd(3);
   phi->Draw();

   c2->cd(4);
   phi_pattern->Draw();

   TCanvas *c3 = new TCanvas();
   c3->Divide(2,2);
   c3->Draw();
   c3->cd(1);
   phi_phi_pattern->Draw();
   c3->cd(2);
   HQval->Draw();

   // open event display
   display->open();

  return 0;
}

std::tuple<Double_t, Double_t> GetMomFromBrho(Double_t M, Double_t Z, Double_t brho)
{

   const Double_t M_Ener = M * 931.49401 / 1000.0;
   Double_t p = brho * Z * (2.99792458 / 10.0); // In GeV
   Double_t E = TMath::Sqrt(TMath::Power(p, 2) + TMath::Power(M_Ener, 2)) - M_Ener;
   std::cout << " Brho : " << brho << " - p : " << p << " - E : " << E << "\n";
   return std::make_tuple(p, E);
}

double kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t  thetalab, Double_t  K_eject)
{

        //in this definition: m1(projectile); m2(target); m3(ejectile); and m4(recoil);
        double Et1 = K_proj + m1;
        double Et2 = m2;
        double Et3 = K_eject + m3;
        double Et4  = Et1 + Et2 - Et3;
        double m4_ex, Ex, theta_cm;
        double s,t,u; //---Mandelstam variables



        s = pow(m1,2) + pow(m2,2) +2*m2*Et1;
        u = pow(m2,2) + pow(m3,2) - 2*m2*Et3;

        m4_ex = sqrt(  (cos(thetalab) * omega(s,pow(m1,2),pow(m2,2)) * omega(u,pow(m2,2),pow(m3,2)) - (s - pow(m1,2) - pow(m2,2))*(pow(m2,2) + pow(m3,2) - u) )/(2*pow(m2,2)) + s + u - pow(m2,2)  );
        Ex = m4_ex - m4;

        t =   pow(m2,2) + pow(m4_ex,2) - 2*m2*Et4;


        //for inverse kinematics Note: this angle corresponds to the recoil
        theta_cm = TMath::Pi() - acos( ( pow(s,2) +s*(2*t - pow(m1,2) - pow(m2,2) - pow(m3,2) - pow(m4_ex,2)) + (pow(m1,2) - pow(m2,2))*(pow(m3,2) - pow(m4_ex,2)) )/( omega(s,pow(m1,2),pow(m2,2))*omega(s,pow(m3,2),pow(m4_ex,2))) ) ;




	 //THcm = theta_cm*TMath::RadToDeg();
         return  Ex;


}
