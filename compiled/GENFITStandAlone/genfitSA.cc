#include "genfitSA.h"
#include "AtFitter.h"
#include "AtGenfit.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

int main()
{
  // -----   Timer   --------------------------------------------------------
  TStopwatch timer;
  timer.Start();
  
  
   //Analysis parameters
   Float_t magneticField = 3.0;//T
   Float_t dMass = 2.0135532;
   Float_t pMass = 1.00727646;
   Int_t atomicNumber = 1;
   Int_t pPDGCode = 2212;
   Int_t dPDGCode = 1000010020;
   
   //Paths
   TString dir = getenv("VMCWORKDIR");
   TString geoManFile  = dir + "/geometry/ATTPC_D1bar_v2_geomanager.root";
   TString filePath ="/macro/Simulation/ATTPC/10Be_dp/";
   TString fileName = "output_digi_el.root";
   TString fileNameWithPath = dir + filePath + fileName;
   std::cout << " Opening File : " << fileNameWithPath.Data() << std::endl;

   FairRunAna *run = new FairRunAna();
   run->SetGeomFile(geoManFile.Data());
   TFile *file = new TFile(fileNameWithPath.Data(), "READ");

   TTree *tree = (TTree *)file->Get("cbmsim");
   Int_t nEvents = tree->GetEntries();
   std::cout << " Number of events : " << nEvents << std::endl;

   TTreeReader Reader1("cbmsim", file);
   TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtPatternEvent");
   //TTreeReaderValue<std::vector<genfit::Track>> fitterVector(Reader1, "ATTPC");

   // GENFIT geometry
   new TGeoManager("Geometry", "ATTPC geometry");
   TGeoManager::Import(geoManFile.Data());
   genfit::MaterialEffects::getInstance()->init(new genfit::TGeoMaterialInterface());
   genfit::FieldManager::getInstance()->init(new genfit::ConstField(0., 0.,magneticField*10.0)); //


   // event display
   genfit::EventDisplay *display = genfit::EventDisplay::getInstance();

   
	  AtFITTER::AtFitter* fFitter = new AtFITTER::AtGenfit(magneticField,0.00001,1000.0);
	  dynamic_cast<AtFITTER::AtGenfit*>(fFitter)->SetPDGCode(dPDGCode);
	  dynamic_cast<AtFITTER::AtGenfit*>(fFitter)->SetMass(dMass);
	  dynamic_cast<AtFITTER::AtGenfit*>(fFitter)->SetAtomicNumber(atomicNumber);
	  dynamic_cast<AtFITTER::AtGenfit*>(fFitter)->SetNumFitPoints(1.0);


   for (Int_t i = 0; i < nEvents; i++) {

      std::cout << " Event Number : " << i << "\n";

      Reader1.Next();
      
      AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray->At(0);

      if (patternEvent) {

	std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
        std::cout << " Number of pattern tracks " << patternTrackCand.size() << "\n";

	for (auto track : patternTrackCand) {

	  if(track.GetIsNoise())
	  continue;
	  
          fFitter->Init();
           
	  genfit::Track *fitTrack=fFitter->FitTracks(&track);
	   TVector3 pos_res;
         TVector3 mom_res;
         TMatrixDSym cov_res;
	  
	   try {

	     if (fitTrack && fitTrack->hasKalmanFitStatus()) {

	       auto KalmanFitStatus = fitTrack->getKalmanFitStatus();


	        if (KalmanFitStatus->isFitted()) {
                  KalmanFitStatus->Print();
                  genfit::MeasuredStateOnPlane fitState = fitTrack->getFittedState();
                  fitState.Print();
                  fitState.getPosMomCov(pos_res, mom_res, cov_res);
                  display->addEvent(fitTrack);
		} 
	       
	     }	  
	  } catch (std::exception &e) {
            std::cout << " " << e.what() << "\n";
            break;
         }
	  
	}//track loop
	 
      }//if pattern event
      
   }
      
  // open event display
   display->open();
   
  return 0;
}
  
