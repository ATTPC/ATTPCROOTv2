#include "AtGenfit.h"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"
#include "FairRunAna.h"

#include "TGeoManager.h"

#include "TDatabasePDG.h"

ClassImp(AtFITTER::AtGenfit)

   AtFITTER::AtGenfit::AtGenfit(Float_t magfield, Float_t minbrho, Float_t maxbrho, std::string eLossFile, Int_t minit,
                                Int_t maxit)
{

   fTPCDetID = 0;
   fCurrentDirection = -1;
   fMinIterations = minit;
   fMaxIterations = maxit;
   fMinBrho = minbrho;
   fMaxBrho = maxbrho;
   fMagneticField = 10.0 * magfield; // T to kGauss
   fNumFitPoints = 0.90;             // Percentage of processed points
   fMass = 1.00727647;               //<! Particle mass in amu
   fAtomicNumber = 1;
   fPDGCode = 2212;
   fVerbosity = 0;
   fEnergyLossFile = eLossFile;
   fSimulationConv = kFALSE;

   fKalmanFitter = std::make_shared<genfit::KalmanFitterRefTrack>();
   fKalmanFitter->setMinIterations(fMinIterations);
   fKalmanFitter->setMaxIterations(fMaxIterations);
   //fKalmanFitter->setDebugLvl();

   fGenfitTrackArray = new TClonesArray("genfit::Track");
   fHitClusterArray = new TClonesArray("AtHitCluster");

   fMeasurementProducer =
      new genfit::MeasurementProducer<AtHitCluster, genfit::AtSpacepointMeasurement>(fHitClusterArray);
   fMeasurementFactory = new genfit::MeasurementFactory<genfit::AbsMeasurement>();
   fMeasurementFactory->addProducer(fTPCDetID, fMeasurementProducer);

   genfit::FieldManager::getInstance()->init(new genfit::ConstField(0., 0., fMagneticField)); // TODO kGauss
   genfit::MaterialEffects *materialEffects = genfit::MaterialEffects::getInstance();
   materialEffects->setEnergyLossFile(fEnergyLossFile);
   materialEffects->init(new genfit::TGeoMaterialInterface());

   // fPDGCandidateArray = new std::vector<Int_t>; // TODO
   // fPDGCandidateArray->push_back(2212);

   std::cout << " AtFITTER::AtGenfit::AtGenfit(): Checking materials that GENFIT will use "
             << "\n";

   TGeoManager *gGeoMan = (TGeoManager *)gROOT->FindObject("FAIRGeom");
   TObjArray *volume_list = gGeoMan->GetListOfVolumes();
   if (!volume_list) {
      std::cout << cRED << " Warning! Null list of geometry volumes." << cNORMAL << "\n";
   }

   int numVol = volume_list->GetEntries();

   for (int ivol = 0; ivol < numVol; ivol++) {
      TGeoVolume *volume = dynamic_cast<TGeoVolume *>(volume_list->At(ivol));
      if (!volume) {

         std::cout << "Got a null geometry volume!! Skipping current list element"
                   << "\n";
         continue;
      }

      std::cout << cGREEN << " Volume name : " << volume->GetName() << cNORMAL << "\n";

      TGeoMaterial *mat = volume->GetMedium()->GetMaterial();

      Int_t mat_indx = mat->GetIndex();

      if (mat->IsMixture()) {
         TGeoMixture *mixt = dynamic_cast<TGeoMixture *>(mat);
         int Nelements = mixt->GetNelements();
         std::cout << cYELLOW << " - Material : " << mat->GetName() << cNORMAL << "\n";
      } else {
         std::cout << cYELLOW << " - Material : " << mat->GetName() << cNORMAL << "\n";
      }
   }

   // PDG definitions
   const Double_t kAu2Gev = 0.9314943228;
   const Double_t khSlash = 1.0545726663e-27;
   const Double_t kErg2Gev = 1 / 1.6021773349e-3;
   const Double_t khShGev = khSlash * kErg2Gev;
   const Double_t kYear2Sec = 3600 * 24 * 365.25;

   TDatabasePDG *db = TDatabasePDG::Instance();
   db->AddParticle("Deuteron", "Deuteron", 1.875612928, kTRUE, 0, 3, "Ion", 1000010020);
   db->AddParticle("Triton", "Triton", 2.80943211, kFALSE, khShGev / (12.33 * kYear2Sec), 3, "Ion", 1000010030);
   db->AddParticle("Alpha", "Alpha", 3.7284, kTRUE, khShGev / (12.33 * kYear2Sec), 6, "Ion", 1000020040);
   db->AddParticle("HE3", "HE3", 2.80941352, kFALSE, 0, 6, "Ion", 1000020030);
}

AtFITTER::AtGenfit::~AtGenfit()
{
   // delete fKalmanFitter;
   delete fGenfitTrackArray;
   delete fHitClusterArray;
   delete fMeasurementProducer;
   delete fMeasurementFactory;
   delete fPDGCandidateArray;
}

void AtFITTER::AtGenfit::Init()
{
   std::cout << cGREEN << " AtFITTER::AtGenfit::Init() " << cNORMAL << "\n";

   fHitClusterArray->Delete();
   fGenfitTrackArray->Delete();
}


TClonesArray *AtFITTER::AtGenfit::GetGenfitTrackArray()
{
   return fGenfitTrackArray;
}

genfit::Track *AtFITTER::AtGenfit::FitTracks(AtTrack *track)
{

   // std::vector<genfit::Track> genfitTrackArray;

   // std::vector<AtTrack> &patternTrackCand = patternEvent.GetTrackCand();

   // std::cout << cGREEN << " AtFITTER::AtGenfit::FitTracks - Number of candidate tracks : " << patternTrackCand.size()
   //<< cNORMAL << "\n";

   // for (auto track : patternTrackCand) {
   fHitClusterArray->Delete();
   genfit::TrackCand trackCand;

   auto hitClusterArray = track->GetHitClusterArray();

   TVector3 pos_res;
   TVector3 mom_res;
   TMatrixDSym cov_res;

   std::cout << cYELLOW << " Track " << track->GetTrackID() << " with " << hitClusterArray->size() << " clusters "
             << cNORMAL << "\n";

   if (hitClusterArray->size() < 5 ||
       track->GetIsNoise()) //&& patternTrackCand.size()<5) { // TODO Check minimum number of clusters
      return nullptr;

   if (fVerbosity > 0) {
      std::cout << " Initial angles from PRA "
                << "\n";
      std::cout << " Theta : " << TMath::RadToDeg() * track->GetGeoTheta()
                << " - Phi : " << TMath::RadToDeg() * track->GetGeoPhi() << "\n";
   }

   // Saving original PRA angles
   Double_t thetaPRA = track->GetGeoTheta();
   Double_t phiPRA = track->GetGeoPhi();

   // New angle convention
   Double_t theta = 0.0;
   Double_t phi = 0.0;

   // Variable for convention (simulation comes reversed)
   Double_t thetaConv;
   if (fSimulationConv) {
      thetaConv = 180.0 * TMath::DegToRad() - track->GetGeoTheta();
   } else {
      thetaConv = track->GetGeoTheta();
   }

   if (thetaConv < 90.0 * TMath::DegToRad()) { // Forward (Backward) for experiment (simulation)

      if (fSimulationConv) {
         theta = 180.0 * TMath::DegToRad() - track->GetGeoTheta();
         phi = track->GetGeoPhi();
      } else {
         theta = track->GetGeoTheta();
         phi = 180.0 * TMath::DegToRad() - track->GetGeoPhi();
      }
      std::reverse(hitClusterArray->begin(), hitClusterArray->end());

   } else if (thetaConv > 90.0 * TMath::DegToRad()) { // Backward (Forward) for experiment (simulation)

      if (fSimulationConv) {
         theta = track->GetGeoTheta();
         phi = 180.0 * TMath::DegToRad() - track->GetGeoPhi();
      } else {
         theta = 180.0 * TMath::DegToRad() - track->GetGeoTheta();
         phi = track->GetGeoPhi();
      }
   } else {
      std::cout << cRED << " AtGenfit::FitTracks - Warning! Undefined theta angle. Skipping event..." << cNORMAL
                << "\n";
      return nullptr;
   }

   Double_t radius = track->GetGeoRadius() / 1000.0; // mm to m

   Double_t brho = (fMagneticField / 10.0) * radius / TMath::Sin(theta); // Tm

   Double_t p_mass = fMass;
   Int_t p_Z = fAtomicNumber;
   Int_t PDGCode = fPDGCode;

   if (fVerbosity > 0) {
      std::cout << " Initial parameters "
                << "\n";
      std::cout << " PDG : " << PDGCode << " - Mass : " << p_mass << " - Atomic number : " << p_Z << "\n";
      std::cout << " B field : " << fMagneticField / 10.0 << " - Min. Bhro : " << fMinBrho
                << " - Max. Brho : " << fMaxBrho << "\n";
      std::cout << " Theta : " << theta * TMath::RadToDeg() << " - Phi : " << phi * TMath::RadToDeg()
                << " - Brho : " << brho << "\n";
      
   }

   // hitClusterArray->resize(hitClusterArray->size() * 0.50);

   // Adding clusterized  hits
   // for (auto cluster : *hitClusterArray) {
   for (auto iCluster = 0; iCluster < hitClusterArray->size(); ++iCluster) {
      auto cluster = hitClusterArray->at(iCluster);
      TVector3 pos = cluster.GetPosition();
      auto clusterClone(cluster);

      if(iCluster==0)
      {
	std::cout<<" First cluster : "<<pos.X()<<" - "<<pos.Y()<<" - "<<pos.Z()<<"\n"; 

      }else if(iCluster==(hitClusterArray->size()-1)){

	std::cout<<" Last cluster : "<<pos.X()<<" - "<<pos.Y()<<" - "<<pos.Z()<<"\n";
      }

      
      if (thetaConv < 90.0 * TMath::DegToRad()) {//Experiment forward
	if(fSimulationConv)
	  clusterClone.SetPosition(pos.X(), pos.Y(), 1000.0 - pos.Z());
	else  
	   clusterClone.SetPosition(-pos.X(), pos.Y(), 1000.0 - pos.Z());
      }

      Int_t idx = fHitClusterArray->GetEntriesFast();
      new ((*fHitClusterArray)[idx]) AtHitCluster(clusterClone);
      trackCand.addHit(fTPCDetID, idx);
      // std::cout<<" Adding  cluster "<<idx<<"\n";
      // std::cout<<pos.X()<<"     "<<pos.Y()<<"   "<<pos.Z()<<"\n";
      // if(iCluster==0)
      // iniPos	= pos;
   }

   // Initial cluster
   // Last(first) one for tracks with angle >(<) 90
   AtHitCluster iniCluster;
   Double_t zIniCal = 0;
   Double_t xIniCal = 0;
   TVector3 iniPos;

   if (thetaConv < 90.0 * TMath::DegToRad()) {
      iniCluster = hitClusterArray->front();
      // iniCluster = hitClusterArray->back();
      iniPos = iniCluster.GetPosition();
      zIniCal = 1000.0 - iniPos.Z();
      
      if(fSimulationConv)
	xIniCal = iniPos.X();
      else
       xIniCal = -iniPos.X();

   } else if (thetaConv > 90.0 * TMath::DegToRad()) {
      iniCluster = hitClusterArray->front();
      // iniCluster = hitClusterArray->back();
      iniPos = iniCluster.GetPosition();
      zIniCal = iniPos.Z();
      xIniCal = iniPos.X();
   } else {
      std::cout << cRED << " AtGenfit::FitTracks - Warning! Undefined theta angle. Skipping event..." << cNORMAL
                << "\n";
      return nullptr;
   }

   Double_t dist = TMath::Sqrt(iniPos.X() * iniPos.X() + iniPos.Y() * iniPos.Y());

   //std::cout<<cRED<<" Distance to Z "<<dist<<cNORMAL<<"\n";
   //if (dist > 70.0)
   //  return nullptr;

   // if(fVerbosity>1)
   std::cout << " Initial position : " << xIniCal << " - " << iniPos.Y() << " - " << zIniCal << "\n";

   TVector3 posSeed(xIniCal / 10.0, iniPos.Y() / 10.0, zIniCal / 10.0);
   posSeed.SetMag(posSeed.Mag());

   TMatrixDSym covSeed(6); // TODO Check where COV matrix is defined, likely in AtPattern clusterize (hard coded
                           // in AtSpacePoint measurement)
   TMatrixD covMatrix = iniCluster.GetCovMatrix();
   for (Int_t iComp = 0; iComp < 3; iComp++)
      covSeed(iComp, iComp) = covMatrix(iComp, iComp) / 100.; // unit conversion mm2 -> cm2

   for (Int_t iComp = 3; iComp < 6; iComp++)
      covSeed(iComp, iComp) = covSeed(iComp - 3, iComp - 3);

   std::tuple<Double_t, Double_t> mom_ener =
      GetMomFromBrho(p_mass, p_Z, brho); // TODO Change to structured bindings when C++17

   // Momentum calculation
   Double_t px = 0, py = 0, pz = 0;
   TVector3 mom_dir(TMath::Sin(theta) * TMath::Cos(phi), TMath::Sin(theta) * TMath::Sin(phi), TMath::Cos(theta));
   px = std::get<0>(mom_ener) * mom_dir.X();
   py = std::get<0>(mom_ener) * mom_dir.Y();
   pz = std::get<0>(mom_ener) * mom_dir.Z();

   if (fVerbosity > 0)
      std::cout << " Momentum from PRA- px : " << px << " - py : " << py << " - pz : " << pz << "\n";

   Double_t momSeedMag = std::get<0>(mom_ener);
   // TVector3 momSeed(0., 0., momSeedMag); //
   TVector3 momSeed(px, py, pz);
   momSeed.SetTheta(theta); // TODO: Check angle conventions
   momSeed.SetPhi(phi);     // TODO
   trackCand.setCovSeed(covSeed);
   trackCand.setPosMomSeed(posSeed, momSeed, p_Z);
   trackCand.setPdgCode(fPDGCode);
   // trackCand.Print();

   if (brho > fMaxBrho && brho < fMinBrho)
      return nullptr;

   genfit::Track *gfTrack =
      new ((*fGenfitTrackArray)[fGenfitTrackArray->GetEntriesFast()]) genfit::Track(trackCand, *fMeasurementFactory);
   gfTrack->addTrackRep(new genfit::RKTrackRep(fPDGCode));

   genfit::RKTrackRep *trackRep = (genfit::RKTrackRep *)gfTrack->getTrackRep(0);
   // trackRep->setPropDir(-1);

   try {
      fKalmanFitter->processTrackWithRep(gfTrack, trackRep, false);
   } catch (genfit::Exception &e) {
      std::cout << " AtGenfit -  Exception caught from Kalman Fitter : " << e.what() << "\n";
      return nullptr;
   }

         // gfTrack->prune("FCW");

         genfit::FitStatus *fitStatus;
         try {
            if (fVerbosity > 0) {
               fitStatus = gfTrack->getFitStatus(trackRep);
               std::cout << cYELLOW << " Is fitted? " << fitStatus->isFitted() << "\n";
               std::cout << " Is Converged ? " << fitStatus->isFitConverged() << "\n";
               std::cout << " Is Converged Partially? " << fitStatus->isFitConvergedPartially() << "\n";
               std::cout << " Is pruned ? " << fitStatus->isTrackPruned() << cNORMAL << "\n";
               fitStatus->Print();
            }
         } catch (genfit::Exception &e) {
            return nullptr;
         }

         genfit::MeasuredStateOnPlane fitState;
         // genfit::TrackPoint* firstPoint;
         // genfit::TrackPoint* lastPoint;
         // genfit::KalmanFitterInfo* pointKFitterInfo;
         try {
            fitState = gfTrack->getFittedState();
            if (fVerbosity > 0)
               fitState.Print();
            // Fit result
            fitState.getPosMomCov(pos_res, mom_res, cov_res);
            if (fVerbosity > 0)
               std::cout << cYELLOW << " Total Momentum : " << mom_res.Mag() << " - Position : " << pos_res.X() << "  "
                         << pos_res.Y() << "  " << pos_res.Z() << cNORMAL << "\n";
            // firstPoint = gfTrack->getPointWithMeasurement(0);
            // lastPoint  = gfTrack->getPointWithMeasurement(gfTrack->getNumPoints()-1);
            // firstPoint->Print();
            // lastPoint->Print();
            // pointKFitterInfo = firstPoint->getKalmanFitterInfo();

         } catch (genfit::Exception &e) {
            return nullptr;
         }

         // TODO: Extrapolate back to Z axis (done in e20009 analysis for the moment)
         /*TVector3 posVertex(0,0,pos_res.Z());
              TVector3 normalVertex(0, 0, 1);


         try {
           for(auto iStep=0;iStep<20;++iStep){
           trackRep -> extrapolateBy(fitState, -0.1*iStep);
           //trackRep -> extrapolateToPlane(fitState,genfit::SharedPlanePtr(new
         genfit::DetPlane(posVertex,normalVertex))); mom_res = fitState.getMom(); pos_res = fitState.getPos(); double
         distance = TMath::Sqrt(pos_res.X()*pos_res.X() + pos_res.Y()*pos_res.Y()); if (fVerbosity > 0) std::cout <<
         cYELLOW << " Extrapolation: Total Momentum : " << mom_res.Mag() << " - Position : " << pos_res.X() << "  "
                              << pos_res.Y() << "  " << pos_res.Z() <<" - POCA : "<<POCA<< cNORMAL << "\n";
           }

              } catch (genfit::Exception &e) {
           mom_res.SetXYZ(0, 0, 0);
           pos_res.SetXYZ(0, 0, 0);
           }*/

         // gfTrack ->Print();

         //} // iTrack

         std::cout << " End of GENFIT "
                   << "\n";
         std::cout << "               "
                   << "\n";

         return gfTrack;
}
