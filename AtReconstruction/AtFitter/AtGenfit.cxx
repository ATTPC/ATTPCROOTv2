#include "AtGenfit.h"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"
#include "FairRunAna.h"

#include "TGeoManager.h"

#include "TDatabasePDG.h"

ClassImp(AtFITTER::AtGenfit)

   AtFITTER::AtGenfit::AtGenfit(Float_t magfield, Float_t minbrho, Float_t maxbrho, Int_t minit, Int_t maxit)
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

   fKalmanFitter = std::make_shared<genfit::KalmanFitterRefTrack>();
   fKalmanFitter->setMinIterations(fMinIterations);
   fKalmanFitter->setMaxIterations(fMaxIterations);
   // fKalmanFitter->setDebugLvl();

   fGenfitTrackArray = new TClonesArray("genfit::Track");
   fHitClusterArray = new TClonesArray("AtHitCluster");

   fMeasurementProducer =
      new genfit::MeasurementProducer<AtHitCluster, genfit::AtSpacepointMeasurement>(fHitClusterArray);
   fMeasurementFactory = new genfit::MeasurementFactory<genfit::AbsMeasurement>();
   fMeasurementFactory->addProducer(fTPCDetID, fMeasurementProducer);

   genfit::FieldManager::getInstance()->init(new genfit::ConstField(0., 0., fMagneticField)); // TODO kGauss
   genfit::MaterialEffects *materialEffects = genfit::MaterialEffects::getInstance();
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

         std::cout << "Got a null geometry volume!! Skiping current list element"
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
   db->AddParticle("Deuteron", "Deuteron", 2 * kAu2Gev + 8.071e-3, kTRUE, 0, 3, "Ion", 1000010020);
   db->AddParticle("Triton", "Triton", 3 * kAu2Gev + 14.931e-3, kFALSE, khShGev / (12.33 * kYear2Sec), 3, "Ion",
                   1000010030);
   db->AddParticle("Alpha", "Alpha", 4 * kAu2Gev + 2.424e-3, kTRUE, khShGev / (12.33 * kYear2Sec), 6, "Ion",
                   1000020040);
   db->AddParticle("HE3", "HE3", 3 * kAu2Gev + 14.931e-3, kFALSE, 0, 6, "Ion", 1000020030);
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

   if (hitClusterArray->size() < 15 ||
       track->GetIsNoise()) //&& patternTrackCand.size()<5) { // TODO Check minimum number of clusters
      return nullptr;

   std::reverse(hitClusterArray->begin(), hitClusterArray->end()); // TODO: Reverted to adapt it to simulation

   // Adding clusterized  hits
   // for (auto cluster : *hitClusterArray) {
   for (auto iCluster = 0; iCluster < hitClusterArray->size() * fNumFitPoints; ++iCluster) {
      auto cluster = hitClusterArray->at(iCluster);
      TVector3 pos = cluster.GetPosition();
      Int_t idx = fHitClusterArray->GetEntriesFast();
      new ((*fHitClusterArray)[idx]) AtHitCluster(cluster);
      trackCand.addHit(fTPCDetID, idx);
      // std::cout<<" Adding  cluster "<<idx<<"\n";
      // std::cout<<pos.X()<<"     "<<pos.Y()<<"   "<<1000.0-pos.Z()<<"\n";
   }

         TVector3 iniPos = hitClusterArray->front().GetPosition(); // TODO Check first cluster is the first in time
         std::cout << " Initial position : " << iniPos.X() << " - " << iniPos.Y() << " - " << iniPos.Z() << "\n";

         TVector3 posSeed(iniPos.X() / 10.0, iniPos.Y() / 10.0, (1000.0 - iniPos.Z()) / 10.0);
         posSeed.SetMag(posSeed.Mag());

         TMatrixDSym covSeed(6); // TODO Check where COV matrix is defined, likely in AtPattern clusterize (hard coded
                                 // in AtSpacePoint measurement)
         TMatrixD covMatrix = hitClusterArray->front().GetCovMatrix();
         for (Int_t iComp = 0; iComp < 3; iComp++)
            covSeed(iComp, iComp) = covMatrix(iComp, iComp) / 100.; // unit conversion mm2 -> cm2

         for (Int_t iComp = 3; iComp < 6; iComp++)
            covSeed(iComp, iComp) = covSeed(iComp - 3, iComp - 3);

         Double_t theta =
            180.0 * TMath::DegToRad() - track->GetGeoTheta(); // 180.0*TMath::DegToRad()-track.GetGeoTheta();
         Double_t radius = track->GetGeoRadius() / 1000.0;    // mm to m
         Double_t phi = track->GetGeoPhi();
         Double_t brho = (fMagneticField / 10.0) * radius / TMath::Sin(theta); // Tm

         Double_t p_mass = fMass;
         Int_t p_Z = fAtomicNumber;
         Int_t PDGCode = fPDGCode;

         std::cout << " Initial parameters "
                   << "\n";
         std::cout << " PDG : " << PDGCode << " - Mass : " << p_mass << " - Atomic number : " << p_Z << "\n";
         std::cout << " B field : " << fMagneticField / 10.0 << " - Min. Bhro : " << fMinBrho
                   << " - Max. Brho : " << fMaxBrho << "\n";
         std::cout << " Theta : " << theta * TMath::RadToDeg() << " - Phi : " << phi * TMath::RadToDeg()
                   << " - Brho : " << brho << "\n";

         std::tuple<Double_t, Double_t> mom_ener =
            GetMomFromBrho(p_mass, p_Z, brho); // TODO Change to structured bindings when C++17
         Double_t momSeedMag = std::get<0>(mom_ener);
         TVector3 momSeed(0., 0., momSeedMag); //
         momSeed.SetTheta(theta);              // TODO: Check angle conventions
         momSeed.SetPhi(phi);                  // TODO
         trackCand.setCovSeed(covSeed);
         trackCand.setPosMomSeed(posSeed, momSeed, p_Z);
         trackCand.setPdgCode(1000020040);
         // trackCand.Print();

         if (brho > fMaxBrho && brho < fMinBrho)
            return nullptr;

         genfit::Track *gfTrack = new ((*fGenfitTrackArray)[fGenfitTrackArray->GetEntriesFast()])
            genfit::Track(trackCand, *fMeasurementFactory);
         gfTrack->addTrackRep(new genfit::RKTrackRep(1000020040)); // TODO: Forcing proton track representation

         genfit::RKTrackRep *trackRep = (genfit::RKTrackRep *)gfTrack->getTrackRep(0);
         // trackRep->setPropDir(1);

         try {
            fKalmanFitter->processTrackWithRep(gfTrack, trackRep, false);
            // fKalmanFitter->processTrackPartially(gfTrack, trackRep,0,hitClusterArray->size()*0.30);
         } catch (genfit::Exception &e) {

            std::cout << " AtGenfit -  Exception caught from Kalman Fitter : " << e.what() << "\n";
         }

         // gfTrack->prune("FCW");

         genfit::FitStatus *fitStatus;
         try {
            fitStatus = gfTrack->getFitStatus(trackRep);
            std::cout << cYELLOW << " Is fitted? " << fitStatus->isFitted() << "\n";
            std::cout << " Is Converged ? " << fitStatus->isFitConverged() << "\n";
            std::cout << " Is Converged Partially? " << fitStatus->isFitConvergedPartially() << "\n";
            std::cout << " Is pruned ? " << fitStatus->isTrackPruned() << cNORMAL << "\n";
            fitStatus->Print();
         } catch (genfit::Exception &e) {
            return nullptr;
         }

         /*genfit::MeasuredStateOnPlane fitState;
    genfit::TrackPoint* firstPoint;
    genfit::TrackPoint* lastPoint;
    genfit::KalmanFitterInfo* pointKFitterInfo;*/
         /*try {
      fitState = gfTrack->getFittedState();
      fitState.Print();
      // Fit result
            fitState.getPosMomCov(pos_res, mom_res, cov_res);
            std::cout << cYELLOW << " Total Momentum : " << mom_res.Mag() << " - Position : " << pos_res.X() << "  "
               << pos_res.Y() << "  " << pos_res.Z() << cNORMAL << "\n";
       firstPoint = gfTrack->getPointWithMeasurement(0);
       lastPoint  = gfTrack->getPointWithMeasurement(gfTrack->getNumPoints()-1);
       //firstPoint->Print();
       //lastPoint->Print();
       //pointKFitterInfo = firstPoint->getKalmanFitterInfo();

         } catch (genfit::Exception &e) {
         }*/

         // gfTrack ->Print();

         //} // iTrack

         std::cout << " End of GENFIT "
                   << "\n";
         std::cout << "               "
                   << "\n";

         return gfTrack;
}
