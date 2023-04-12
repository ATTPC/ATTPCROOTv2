#include "AtGenfit.h"

#include "AtHitCluster.h"
#include "AtSpacePointMeasurement.h"
#include "AtTrack.h"

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <RKTrackRep.h>
#include <TClonesArray.h>
#include <TDatabasePDG.h>
#include <TGeoManager.h>
#include <TGeoMaterial.h>
#include <TGeoMaterialInterface.h>
#include <TGeoMedium.h>
#include <TGeoVolume.h>
#include <TMath.h>
#include <TMatrixDSymfwd.h>
#include <TMatrixDfwd.h>
#include <TMatrixT.h>
#include <TMatrixTSym.h>
#include <TObjArray.h>
#include <TObject.h>
#include <TROOT.h>
#include <TVector3.h>
#include <TrackCand.h>

#include <AbsKalmanFitter.h>
#include <AbsMeasurement.h>
#include <AbsTrackRep.h>
#include <ConstField.h>
#include <Exception.h>
#include <FieldManager.h>
#include <FitStatus.h>
#include <KalmanFitterRefTrack.h>
#include <MaterialEffects.h>
#include <MeasuredStateOnPlane.h>
#include <MeasurementFactory.h>
#include <MeasurementProducer.h>

#include <algorithm>
#include <iostream>
#include <tuple>
#include <utility>
constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

using XYZPoint = ROOT::Math::XYZPoint;

AtFITTER::AtGenfit::AtGenfit(Float_t magfield, Float_t minbrho, Float_t maxbrho, std::string eLossFile,
                             Float_t gasMediumDensity, Int_t pdg, Int_t minit, Int_t maxit)
   : fEnergyLossFile(std::move(eLossFile)),
     fMeasurementProducer(
        new genfit::MeasurementProducer<AtHitCluster, genfit::AtSpacepointMeasurement>(fHitClusterArray)),
     fMeasurementFactory(new genfit::MeasurementFactory<genfit::AbsMeasurement>()), fMinIterations(minit),
     fMaxIterations(maxit), fMinBrho(minbrho), fMaxBrho(maxbrho), fMagneticField(10.0 * magfield), fPDGCode(pdg),
     fGenfitTrackArray(new TClonesArray("genfit::Track")), fHitClusterArray(new TClonesArray("AtHitCluster"))
{
   fKalmanFitter = std::make_shared<genfit::KalmanFitterRefTrack>();
   fKalmanFitter->setMinIterations(fMinIterations);
   fKalmanFitter->setMaxIterations(fMaxIterations);
   // fKalmanFitter->setDebugLvl();
   fMeasurementFactory->addProducer(fTPCDetID, fMeasurementProducer);

   genfit::FieldManager::getInstance()->init(new genfit::ConstField(0., 0., fMagneticField)); // NOLINT TODO kGauss
   genfit::MaterialEffects *materialEffects = genfit::MaterialEffects::getInstance();
   materialEffects->setEnergyLossBrems(false);
   materialEffects->setNoiseBrems(false);
   materialEffects->useEnergyLossParam();
   materialEffects->init(new genfit::TGeoMaterialInterface()); // NOLINT
   // Parameteres set after initialization
   materialEffects->setGasMediumDensity(gasMediumDensity);
   materialEffects->setEnergyLossFile(fEnergyLossFile, fPDGCode);

   // fPDGCandidateArray = new std::vector<Int_t>; // TODO
   // fPDGCandidateArray->push_back(2212);

   std::cout << " AtFITTER::AtGenfit::AtGenfit(): Checking materials that GENFIT will use "
             << "\n";

   auto *gGeoMan = dynamic_cast<TGeoManager *>(gROOT->FindObject("FAIRGeom"));
   TObjArray *volume_list = gGeoMan->GetListOfVolumes();
   if (!volume_list) {
      std::cout << cRED << " Warning! Null list of geometry volumes." << cNORMAL << "\n";
   }

   int numVol = volume_list->GetEntries();

   for (int ivol = 0; ivol < numVol; ivol++) {
      auto *volume = dynamic_cast<TGeoVolume *>(volume_list->At(ivol));
      if (!volume) {

         std::cout << "Got a null geometry volume!! Skipping current list element"
                   << "\n";
         continue;
      }

      std::cout << cGREEN << " Volume name : " << volume->GetName() << cNORMAL << "\n";

      TGeoMaterial *mat = volume->GetMedium()->GetMaterial();

      // Int_t mat_indx = mat->GetIndex();
      std::cout << cYELLOW << " - Material : " << mat->GetName() << cNORMAL << "\n";
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
   db->AddParticle("Alpha", "Alpha", 3.7284, kTRUE, 0, 6, "Ion", 1000020040);
   db->AddParticle("He3", "He3", 2.80941352, kTRUE, 0, 6, "Ion", 1000020030);
   db->AddParticle("He6", "He6", 5.60655972, kFALSE, khShGev / 0.806, 6, "Ion", 1000020060);
   db->AddParticle("Be10", "Be10", 9.3275477, kFALSE, khShGev / (1.51E6 * kYear2Sec), 12, "Ion", 1000040100);
   db->AddParticle("Be11", "Be11", 10.2666092, kFALSE, khShGev / (13.76), 12, "Ion", 1000040110);
   db->AddParticle("C12", "C12", 11.18, kTRUE, 0, 18, "Ion", 1000060120);
   db->AddParticle("O16", "O16", 14.8991686, kTRUE, 0, 24, "Ion", 1000080160);
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
   // std::cout << cGREEN << " PDG : "<<fPDGCode<<"\n";
   // std::cout << cGREEN << " Ion : "<<fIonName<<"\n";

   fHitClusterArray->Delete();
   fGenfitTrackArray->Delete();
}

TClonesArray *AtFITTER::AtGenfit::GetGenfitTrackArray()
{
   return fGenfitTrackArray;
}

/**
 * Angles from track are used to construct the direction of the initial momentum of the track.
 * Radius from track is used to construct the magnitude of the initial momentum of the track.
 */
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

   if (hitClusterArray->size() < 3) //&& patternTrackCand.size()<5) { // TODO Check minimum number of clusters
      return nullptr;

   if (fVerbosity > 0) {
      std::cout << " Initial angles from PRA "
                << "\n";
      std::cout << " Theta : " << TMath::RadToDeg() * track->GetGeoTheta()
                << " - Phi : " << TMath::RadToDeg() * track->GetGeoPhi() << "\n";
   }

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

   if (IsForwardTrack(thetaConv)) { // Forward (Backward) for experiment (simulation)

      if (fSimulationConv) {
         theta = 180.0 * TMath::DegToRad() - track->GetGeoTheta();
         phi = track->GetGeoPhi();
      } else {
         theta = track->GetGeoTheta();
         phi = track->GetGeoPhi(); // 180.0 * TMath::DegToRad() - track->GetGeoPhi();
      }

      // Needs to be reversed back for multifit
      std::reverse(hitClusterArray->begin(), hitClusterArray->end());

   } else if (thetaConv > 90.0 * TMath::DegToRad()) { // Backward (Forward) for experiment (simulation)

      if (fSimulationConv) {
         theta = track->GetGeoTheta();
         phi = 180.0 * TMath::DegToRad() - track->GetGeoPhi(); // 180.0 * TMath::DegToRad() - track->GetGeoPhi();
      } else {
         theta = 180.0 * TMath::DegToRad() - track->GetGeoTheta();
         phi = -track->GetGeoPhi();
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
      std::cout << cYELLOW << "    ---- AtGenfit : Initial parameters "
                << "\n";
      std::cout << "    PDG : " << PDGCode << " - Mass : " << p_mass << " - Atomic number : " << p_Z << "\n";
      std::cout << "    B field : " << fMagneticField / 10.0 << " - Min. Bhro : " << fMinBrho
                << "    - Max. Brho : " << fMaxBrho << "\n";
      std::cout << "    Theta : " << theta * TMath::RadToDeg() << " - Phi : " << phi * TMath::RadToDeg()
                << "    - Brho (geo) : " << brho << cNORMAL << "\n";
   }

   // hitClusterArray->resize(hitClusterArray->size() * 0.50);

   // Adding clusterized  hits
   // for (auto cluster : *hitClusterArray) {
   for (auto iCluster = 0; iCluster < hitClusterArray->size(); ++iCluster) {
      auto cluster = hitClusterArray->at(iCluster);
      auto pos = cluster.GetPosition();
      auto clusterClone(cluster);

      if (iCluster == 0) {
         std::cout << cYELLOW << "    First cluster : " << pos.X() << " - " << pos.Y() << " - " << pos.Z() << cNORMAL
                   << "\n";

      } else if (iCluster == (hitClusterArray->size() - 1)) {

         std::cout << cYELLOW << "    Last cluster : " << pos.X() << " - " << pos.Y() << " - " << pos.Z() << cNORMAL
                   << "\n";
      }

      if (IsForwardTrack(thetaConv)) { // Experiment forward
         if (fSimulationConv)
            clusterClone.SetPosition({pos.X(), pos.Y(), 1000.0 - pos.Z()});
         else
            clusterClone.SetPosition({-pos.X(), pos.Y(), 1000.0 - pos.Z()});
      } else if (thetaConv > 90.0 * TMath::DegToRad()) {
         if (fSimulationConv)
            clusterClone.SetPosition({pos.X(), pos.Y(), pos.Z()});
         else
            clusterClone.SetPosition({-pos.X(), pos.Y(), pos.Z()});
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

   XYZPoint iniPos;

   // Initial track position
   if (IsForwardTrack(thetaConv)) {
      iniCluster = hitClusterArray->front();
      // iniCluster = hitClusterArray->back();
      iniPos = iniCluster.GetPosition();
      zIniCal = 1000.0 - iniPos.Z();

      if (fSimulationConv)
         xIniCal = iniPos.X();
      else
         xIniCal = -iniPos.X();

      // Leave hit cluster array in its original state
      std::reverse(hitClusterArray->begin(), hitClusterArray->end());

   } else if (thetaConv > 90.0 * TMath::DegToRad()) {
      iniCluster = hitClusterArray->front();
      // iniCluster = hitClusterArray->back();
      iniPos = iniCluster.GetPosition();
      zIniCal = iniPos.Z();

      if (fSimulationConv)
         xIniCal = iniPos.X();
      else
         xIniCal = -iniPos.X();

   } else {
      std::cout << cRED << " AtGenfit::FitTracks - Warning! Undefined theta angle. Skipping event..." << cNORMAL
                << "\n";
   }

   // Double_t dist = TMath::Sqrt(iniPos.X() * iniPos.X() + iniPos.Y() * iniPos.Y());

   // std::cout<<cRED<<" Distance to Z "<<dist<<cNORMAL<<"\n";
   // if (dist > 70.0)
   //   return nullptr;

   // if(fVerbosity>1)
   std::cout << "    Initial position : " << xIniCal << " - " << iniPos.Y() << " - " << zIniCal << "\n";

   TVector3 posSeed(xIniCal / 10.0, iniPos.Y() / 10.0, zIniCal / 10.0);
   posSeed.SetMag(posSeed.Mag());

   // Starting wih fit...

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
      std::cout << cYELLOW << "    Momentum from PRA- px : " << px << " - py : " << py << " - pz : " << pz << cNORMAL
                << "\n";

   // Double_t momSeedMag = std::get<0>(mom_ener);
   //  TVector3 momSeed(0., 0., momSeedMag); //
   TVector3 momSeed(px, py, pz);
   momSeed.SetTheta(theta); // TODO: Check angle conventions
   momSeed.SetPhi(phi);     // TODO
   trackCand.setCovSeed(covSeed);
   trackCand.setPosMomSeed(posSeed, momSeed, p_Z);
   trackCand.setPdgCode(fPDGCode);
   // trackCand.Print();

   if (brho > fMaxBrho && brho < fMinBrho)
      return nullptr;

   auto *gfTrack = new ((*fGenfitTrackArray)[fGenfitTrackArray->GetEntriesFast()]) // NOLINT
      genfit::Track(trackCand, *fMeasurementFactory);
   gfTrack->addTrackRep(new genfit::RKTrackRep(fPDGCode)); // NOLINT

   auto *trackRep = dynamic_cast<genfit::RKTrackRep *>(gfTrack->getTrackRep(0));
   // trackRep->setPropDir(-1);

   try {
      fKalmanFitter->processTrackWithRep(gfTrack, trackRep, false);
   } catch (genfit::Exception &e) {
      std::cout << " AtGenfit -  Exception caught from Kalman Fitter : " << e.what() << "\n";
      return nullptr;
   }

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
         std::cout << cYELLOW << "    Total Momentum : " << mom_res.Mag() << " - Position : " << pos_res.X() << "  "
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

ClassImp(AtFITTER::AtGenfit);
