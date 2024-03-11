#include "AtGenfit.h"

#include "AtFittedTrack.h"
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

using XYZPoint = ROOT::Math::XYZPoint;

AtFITTER::AtGenfit::AtGenfit(Float_t magfield, Float_t minbrho, Float_t maxbrho, std::string eLossFile,
                             Float_t gasMediumDensity, Int_t pdg, Int_t minit, Int_t maxit, Bool_t noMatEffects)
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
   materialEffects->setNoEffects(noMatEffects);
   materialEffects->useEnergyLossParam();
   materialEffects->init(new genfit::TGeoMaterialInterface()); // NOLINT
   // Parameteres set after initialization
   materialEffects->setGasMediumDensity(gasMediumDensity);
   materialEffects->setEnergyLossFile(fEnergyLossFile, fPDGCode);

   fTrackTransformer = std::make_unique<AtTools::AtTrackTransformer>();
   fKinematics = std::make_shared<AtTools::AtKinematics>();

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
   LOG(debug) << cGREEN << " AtFITTER::AtGenfit::Init() " << cNORMAL << "\n";
   LOG(debug) << cGREEN << " AtFitterTask::Init - Fit parameters. "
              << "\n";
   LOG(debug) << " Magnetic Field       : " << fMagneticField << " T\n";
   LOG(debug) << " PDG Code             : " << fPDGCode << "\n";
   LOG(debug) << " Mass                 : " << fMass << " amu\n";
   LOG(debug) << " Atomic Number        : " << fAtomicNumber << "\n";
   LOG(debug) << " Number of fit points : " << fNumFitPoints << "\n";
   LOG(debug) << " Maximum iterations   : " << fMaxIterations << "\n";
   LOG(debug) << " Minimum iterations   : " << fMinIterations << "\n";
   LOG(debug) << " Maximum brho         : " << fMaxBrho << "\n";
   LOG(debug) << " Minimum brho         : " << fMinBrho << "\n";
   LOG(debug) << " Energy loss file     : " << fEnergyLossFile << "\n";
   LOG(debug) << " --------------------------------------------- " << cNORMAL << "\n";

   fHitClusterArray->Delete();
   fGenfitTrackArray->Delete();
}

TClonesArray *AtFITTER::AtGenfit::GetGenfitTrackArray()
{
   return fGenfitTrackArray;
}

std::vector<std::unique_ptr<AtFittedTrack>> AtFITTER::AtGenfit::ProcessTracks(std::vector<AtTrack> &tracks)
{

   std::vector<std::unique_ptr<AtFittedTrack>> fittedTracks;

   // Global variables for AtFittedTrack
   Int_t trackID = -1;
   Float_t xiniPRA = -100;
   Float_t yiniPRA = -100;
   Float_t ziniPRA = -1000;
   Float_t EPRA = 0;
   Float_t APRA = 0;
   Float_t PhiPRA = 0;
   std::string PDG = "2212";
   Float_t xiniFit = -100;
   Float_t yiniFit = -100;
   Float_t ziniFit = -1000;
   Float_t xiniFitXtr = -100;
   Float_t yiniFitXtr = -100;
   Float_t ziniFitXtr = -1000;
   Float_t pVal = 0;
   Float_t trackLength = -1000.0;
   Float_t POCAXtr = -1000.0;
   Int_t particleQ = -10;
   Float_t EFit = 0;
   Float_t EFitXtr = 0;
   Float_t AFit = 0;
   Float_t PhiFit = 0;
   Float_t distXtr = -1000.0;
   //////////////////////////////

   // TODO
   Double_t distThres = 150.0;

   // Merge tracks and calculate directions
   std::vector<AtTrack> mergedTrackPool;
   std::vector<AtTrack *> candTrackPool;
   Int_t eventMultiplicity = 0;
   Int_t praMultiplicity = 0;

   auto sp = std::unique_ptr<AtTrack[]>(new AtTrack[tracks.size()]);

   for (auto iTrack = 0; iTrack < tracks.size(); ++iTrack) {
      AtTrack track = tracks.at(iTrack);

      std::cout << cYELLOW << " Track " << track.GetTrackID() << " with " << track.GetHitClusterArray()->size()
                << " clusters and " << track.GetHitArray().size() << " hits. " << cNORMAL << "\n";

      trackID = track.GetTrackID();

      if (track.GetHitClusterArray()->size() < 3) {
         LOG(debug) << cRED << " Track has less than 3 clusters! " << cNORMAL << "\n";
         continue;
      }

      if (track.GetTrackID() == -1) {
         LOG(debug) << cRED << " Track is noise! " << cNORMAL << "\n";
         continue;
      }

      ++praMultiplicity;

      // Track merging
      Double_t theta = track.GetGeoTheta();
      std::pair<Double_t, Double_t> center = track.GetGeoCenter();
      Double_t thetaConv;

      if (fSimulationConv)
         thetaConv = 180.0 - theta * TMath::RadToDeg();
      else
         thetaConv = theta * TMath::RadToDeg();

      auto hitClusterArray = track.GetHitClusterArray();
      AtHitCluster iniCluster;
      AtHitCluster secCluster;
      AtHitCluster endCluster;
      Double_t zIniCal = 0;
      Double_t zEndCal = 0;
      ROOT::Math::XYZPoint iniPos;
      ROOT::Math::XYZPoint secPos;
      ROOT::Math::XYZPoint endPos;

      if (thetaConv < 90.0) { // Forward tracks
         iniCluster =
            hitClusterArray->back(); // NB: Use back because We do not reverse the cluster vector like in AtGenfit!
         secCluster = hitClusterArray->at(hitClusterArray->size() - 2);
         iniPos = iniCluster.GetPosition();
         secPos = secCluster.GetPosition();
         endCluster = hitClusterArray->front();
         endPos = endCluster.GetPosition();
         zIniCal = 1000.0 - iniPos.Z();
         zEndCal = 1000.0 - endPos.Z();
      } else if (thetaConv > 90.0) { // Backward tracks
         iniCluster = hitClusterArray->front();
         secCluster = hitClusterArray->at(1);
         iniPos = iniCluster.GetPosition();
         secPos = secCluster.GetPosition();
         endCluster = hitClusterArray->back();
         endPos = endCluster.GetPosition();
         zIniCal = iniPos.Z();
         zEndCal = endPos.Z();
      }

      xiniPRA = iniPos.X();
      yiniPRA = iniPos.Y();
      ziniPRA = zIniCal;

      // NB: Mind the x sign. Currently set for backward tracks
      // std::cout<<"   Old phi from PRA : "<<track.GetGeoPhi()*TMath::RadToDeg()<<"\n";
      Double_t phiClus = 0;

      if (thetaConv > 90) {
         phiClus = TMath::ATan2(secPos.Y() - iniPos.Y(), -secPos.X() + iniPos.X());
         if (fSimulationConv)
            phiClus = TMath::ATan2(secPos.Y() - iniPos.Y(), secPos.X() - iniPos.X());
         track.SetGeoPhi(-phiClus);
      } else if (thetaConv < 90) {
         phiClus = TMath::ATan2(secPos.Y() - iniPos.Y(), -secPos.X() + iniPos.X());
         if (fSimulationConv)
            phiClus = TMath::ATan2(secPos.Y() - iniPos.Y(), secPos.X() - iniPos.X());
         track.SetGeoPhi(phiClus);
      }

      // This is just to select distances
      LOG(debug) << cBLUE << "   Track ID " << track.GetTrackID() << "\n";
      LOG(debug) << "   Initial position : " << xiniPRA << " - " << yiniPRA << " - " << ziniPRA << " "
                 << iniCluster.GetTimeStamp() << "\n";
      LOG(debug) << "   Second position : " << secPos.X() << " - " << secPos.Y() << " - " << secPos.Z() << " "
                 << secCluster.GetTimeStamp() << "\n";
      LOG(debug) << "   End position : " << endPos.X() << " - " << endPos.Y() << " - " << zEndCal << " "
                 << endCluster.GetTimeStamp() << "\n";
      LOG(debug) << "   Theta (PRA) " << track.GetGeoTheta() * TMath::RadToDeg()
                 << "   Theta (convention) : " << thetaConv << " - Phi Clus : " << phiClus * TMath::RadToDeg() << "\n";
      LOG(debug) << "   Track center - X :  " << center.first << " - Y : " << center.second << "\n";
      LOG(debug) << "   Track phi recalc : " << track.GetGeoPhi() * TMath::RadToDeg() << cNORMAL << "\n";

      // Skip tracks that are far from Z (to be checked against number of iterations for extrapolation)
      Double_t dist = TMath::Sqrt(iniPos.X() * iniPos.X() + iniPos.Y() * iniPos.Y());
      LOG(debug) << KRED << "    Distance to Z " << dist << cNORMAL << "\n";
      LOG(debug) << KGRN << "    ---- Adding track candidate " << cNORMAL << "\n";
      track.SetVertexToZDist(dist);
      if (fEnableMerging) {
         sp[iTrack] = track;
         candTrackPool.push_back(std::move(&sp[iTrack]));

      } else {
         continue;
      }

   } // Tracks

   FindSingleTracks(candTrackPool);

   // Find candidate tracks closer to vertex and discard duplicated

   std::vector<AtTrack *> candToMergePool;

   if (fEnableMerging && !fSimulationConv) { // TODO: Not adapted to simulation yet

      for (auto itA = candTrackPool.begin(); itA != candTrackPool.end(); ++itA) {

         candToMergePool.clear();
         AtTrack *trA = *(itA);
         std::cout << " Processing track : " << trA->GetTrackID() << "\n";

         auto itB = std::copy_if(itA + 1, candTrackPool.end(), std::back_inserter(candToMergePool),
                                 [&trA, this](AtTrack *track) { return CompareTracks(trA, track); });

         if (candToMergePool.size() > 0) { // Merge if matches are found
            candToMergePool.push_back(trA);
            Bool_t merged = MergeTracks(&candToMergePool, &mergedTrackPool, fEnableSingleVertexTrack, fClusterRadius,
                                        fClusterSize); // NB: Reclustering is also performed here

         } else {
            if (!trA->GetIsMerged() && trA->GetVertexToZDist() < distThres)
               mergedTrackPool.push_back(*trA);
         }

         std::cout << " Merged track pool size " << mergedTrackPool.size() << " - candidate to merge pool size "
                   << candToMergePool.size() << "\n";
      }

   } else {
      for (auto track : candTrackPool)
         mergedTrackPool.push_back(*track);
   }

   std::cout << "\n";
   std::cout << KGRN << "    Candidate/Merged Tracks Pool Size : " << mergedTrackPool.size() << "\n";
   std::cout << "    Been merged? " << fEnableMerging << "\n";
   std::cout << "    Tracks prepared. Proceeding with fits. " << cNORMAL << "\n";
   std::cout << "\n";

   eventMultiplicity = mergedTrackPool.size();

   // Fitting track candidates
   for (auto track : mergedTrackPool) {

      if (fEnableReclustering) {
         track.ResetHitClusterArray();
         fTrackTransformer->ClusterizeSmooth3D(track, fClusterRadius,
                                               fClusterSize); // NB: Just for analysis benchmarking
      }

      Double_t theta = track.GetGeoTheta();
      Double_t radius = track.GetGeoRadius() / 1000.0; // mm to m
      Double_t phi = track.GetGeoPhi();
      Double_t brho = (fMagneticField / 10.0) * radius / TMath::Sin(theta); // Tm
      Double_t points = track.GetHitArray().size();

      std::cout << "      Merged track - Theta : " << theta * TMath::RadToDeg() << " Phi : " << phi * TMath::RadToDeg()
                << "\n";

      auto hitClusterArray = track.GetHitClusterArray();
      AtHitCluster iniCluster;
      Double_t zIniCal = 0;
      ROOT::Math::XYZPoint iniPos;

      // for(auto hitCluster : *hitClusterArray)
      // std::cout<<" Cluster hit "<<hitCluster.GetHitID()<<" - "<<hitCluster.GetPosition().X()<<" -
      // "<<hitCluster.GetPosition().Y()<<" - "<<1000.0-hitCluster.GetPosition().Z()<<"\n";

      // Variable for convention (simulation comes reversed)
      Double_t thetaConv;
      if (fSimulationConv) {
         thetaConv = 180.0 - theta * TMath::RadToDeg();
      } else {
         thetaConv = theta * TMath::RadToDeg();
      }

      if (thetaConv < 90.0) {
         iniCluster =
            hitClusterArray->back(); // NB: Use back because We do not reverse the cluster vector like in AtGenfit!
         iniPos = iniCluster.GetPosition();
         zIniCal = 1000.0 - iniPos.Z();
      } else if (thetaConv > 90.0) {
         iniCluster = hitClusterArray->front();
         iniPos = iniCluster.GetPosition();
         zIniCal = iniPos.Z();
      }

      xiniPRA = iniPos.X();
      yiniPRA = iniPos.Y();
      ziniPRA = zIniCal;

      // This is just to select distances
      // std::cout << cGREEN << "      Merged track - Initial position : " << xiniPRA << " - " << yiniPRA << " - "
      //         << ziniPRA << cNORMAL << "\n";

      // Skip border angles
      //    if (theta * TMath::RadToDeg() < 5 || theta * TMath::RadToDeg() > 175)
      // continue;
      // Skip tracks that are far from Z (to be checked against number of iterations for extrapolation)
      Double_t dist = TMath::Sqrt(iniPos.X() * iniPos.X() + iniPos.Y() * iniPos.Y());

      std::cout << KRED << "       Merged track - Distance to Z (Candidate Track Pool) " << dist << cNORMAL << "\n";

      Init();

      std::vector<Int_t> pdgCandFit;
      if (thetaConv > 90) {

         switch (fExpNum) {
         case e20020: pdgCandFit.push_back(1000010020); break;
         case e20009: pdgCandFit.push_back(2212); break;
         case a1975: pdgCandFit.push_back(1000010020); break;
         default: pdgCandFit.push_back(2212);
         }

      } else if (thetaConv < 90 && thetaConv > 10) {

         switch (fExpNum) {
         case e20020: pdgCandFit.push_back(1000020040); break;
         case e20009: pdgCandFit.push_back(1000010020); break;
         case a1954: pdgCandFit.push_back(2212); break;
         case a1954b: pdgCandFit.push_back(2212); break;
         // case a1954b: pdgCandFit.push_back(1000010020); break;
         // case a1975: pdgCandFit.push_back(2212); break;
         case a1975: pdgCandFit.push_back(1000010020); break;
         default: pdgCandFit.push_back(2212);
         }

      } else if (thetaConv < 10) {

         switch (fExpNum) {
         case e20009:
            pdgCandFit.push_back(1000040100);
            break;
            // pdgCandFit.push_back(1000040110);
         default: pdgCandFit.push_back(2212);
         }
      }

      try {

         genfit::Track *fitTrack = FitTracks(&track);

         Int_t atomicNumber = fAtomicNumber;
         Double_t mass = fMass;
         Double_t M_Ener = mass * 931.49401 / 1000.0;

         // PDG needs to be defined
         Int_t pdg = pdgCandFit.at(0);

         /*auto fIl =
            std::find_if(ionList->begin(), ionList->end(), [&pdg](AtTools::IonFitInfo ion) { return ion._PDG == pdg; });
         if (fIl != ionList->end()) {

            int index = std::distance(ionList->begin(), fIl);
            std::cout << cBLUE << "  -  Ion info for : " << pdg << " found in " << index << cNORMAL << "\n";
            atomicNumber = ionList->at(index)._atomicNumber;
            mass = ionList->at(index)._mass;
            M_Ener = mass * 931.49401 / 1000.0;
         }*/

         // Kinematics from PRA

         std::tuple<Double_t, Double_t> mom_ener = fKinematics->GetMomFromBrho(mass, atomicNumber, brho);
         EPRA = std::get<1>(mom_ener) * 1000.0;
         APRA = theta * TMath::RadToDeg();
         PhiPRA = phi * TMath::RadToDeg();

         // Extract info from Fit track

         PDG = std::to_string(pdg);

         TVector3 pos_res;
         TVector3 mom_res;
         TMatrixDSym cov_res;

         Double_t bChi2 = 0, fChi2 = 0, bNdf = 0, fNdf = 0;
         Double_t distance = -100;
         Double_t POCA = 1E6;
         TVector3 mom_ext;
         TVector3 pos_ext;
         TVector3 mom_ext_buff;
         TVector3 pos_ext_buff;
         Int_t nSteps = 0;

         // First orbit
         Double_t POCAOrbZ = 1E6;
         Double_t firstOrbZ = 0.0;
         Double_t phiOrbZ = 0.0;
         Double_t lengthOrbZ = 0.0;
         Double_t eLossOrbZ = 0.0;

         // Fit convergence
         Int_t fitConverged = 0;

         // Reset variables assigned in fitting
         pVal = -1;
         trackLength = 0;
         xiniFitXtr = -1000.0;
         yiniFitXtr = -1000.0;
         ziniFitXtr = -1E4;
         xiniFit = -1000.0;
         yiniFit = -1000.0;
         ziniFit = -1E4;
         POCAXtr = -1000.0;
         EFit = -10.0;
         EFitXtr = -10.0;
         AFit = 0.0;
         PhiFit = 0.0;
         particleQ = -10.0;

         // PID
         Double_t len = 0;
         Double_t eloss = 0;
         Double_t dedx = 0;

         // Energy loss from ADC
         auto hitClusterArray = track.GetHitClusterArray();
         std::size_t cnt = 0;

         if (thetaConv < 90) {

            auto it = hitClusterArray->rbegin();
            while (it != hitClusterArray->rend()) {

               if (((Float_t)cnt / (Float_t)hitClusterArray->size()) > 0.8)
                  break;
               auto dir = (*it).GetPosition() - (*std::next(it, 1)).GetPosition();
               eloss += (*it).GetCharge();
               len = std::sqrt(dir.Mag2());
               dedx += (*it).GetCharge();
               // std::cout<<(*it).GetCharge()<<"\n";
               it++;
               ++cnt;
            }
         } else if (thetaConv > 90) {

            eloss += hitClusterArray->at(0).GetCharge();

            cnt = 1;
            for (auto iHitClus = 1; iHitClus < hitClusterArray->size(); ++iHitClus) {

               if (((Float_t)cnt / (Float_t)hitClusterArray->size()) > 0.8)
                  break;
               auto dir = hitClusterArray->at(iHitClus).GetPosition() - hitClusterArray->at(iHitClus - 1).GetPosition();
               len = std::sqrt(dir.Mag2());
               eloss += hitClusterArray->at(iHitClus).GetCharge();
               dedx += hitClusterArray->at(iHitClus).GetCharge();
               // std::cout<<len<<" - "<<eloss<<" - "<<hitClusterArray->at(iHitClus).GetCharge()<<"\n";
               ++cnt;
            }
         }

         eloss /= cnt;
         // dedx /= len;

         if (fitTrack == nullptr)
            continue;

         try {

            if (fitTrack && fitTrack->hasKalmanFitStatus()) {

               auto KalmanFitStatus = fitTrack->getKalmanFitStatus();
               auto trackRep = fitTrack->getTrackRep(0); // Only one representation is sved for the moment.
               fitConverged = KalmanFitStatus->isFitConverged(false);

               if (KalmanFitStatus->isFitConverged(false)) {
                  // KalmanFitStatus->Print();
                  genfit::MeasuredStateOnPlane fitState = fitTrack->getFittedState();
                  particleQ = fitState.getCharge();

                  fChi2 = KalmanFitStatus->getForwardChi2();
                  bChi2 = KalmanFitStatus->getBackwardChi2();
                  fNdf = KalmanFitStatus->getForwardNdf();
                  bNdf = KalmanFitStatus->getBackwardNdf();
                  // fitState.Print();
                  fitState.getPosMomCov(pos_res, mom_res, cov_res);
                  trackLength = KalmanFitStatus->getTrackLen();
                  pVal = KalmanFitStatus->getPVal();

                  // fKalmanFitter -> getChiSquNdf(gfTrack, trackRep, bChi2, fChi2, bNdf, fNdf);
                  Float_t stepXtr = -0.01;
                  Int_t minCnt = 0;
                  Int_t minCntExt = 0;

                  TVector3 pos_ini_buff = pos_res;
                  Double_t length = 0.0;
                  mom_ext = fitState.getMom();
                  pos_ext = fitState.getPos();

                  // Backward extrapolation
                  try {

                     for (auto iStep = 0; iStep < 200; ++iStep) {

                        trackRep->extrapolateBy(fitState, stepXtr * iStep);
                        mom_ext_buff = fitState.getMom();
                        pos_ext_buff = fitState.getPos();

                        length += (pos_ext_buff - pos_ini_buff).Mag();
                        pos_ini_buff = pos_ext_buff;

                        double distPOCA =
                           TMath::Sqrt(pos_ext_buff.X() * pos_ext_buff.X() + pos_ext_buff.Y() * pos_ext_buff.Y());
                        // if (fVerbosityLevel > 2){
                        // std::cout << cYELLOW << " Extrapolation: Total Momentum : " << mom_ext_buff.Mag()
                        //        << " - Position : " << pos_ext_buff.X() << "  " << pos_ext_buff.Y() << "  "
                        //      << pos_ext_buff.Z() << " - distance of approach : " << distance <<" - length :
                        //  "<<length<< cNORMAL << "\n";
                        //}

                        // if(pos_ext_buff.Z()<0)
                        // break;

                        if (distPOCA < POCA) {
                           POCA = distPOCA;
                           POCAXtr = distPOCA;
                           mom_ext = mom_ext_buff;
                           pos_ext = pos_ext_buff;
                           distXtr = iStep * stepXtr;
                           ++minCnt;
                           minCntExt = 0;
                           nSteps = iStep;
                        } else {

                           break;
                        }

                        ++minCntExt;
                     } // Extrapolation loop

                  } catch (genfit::Exception &e) {
                     mom_ext.SetXYZ(0, 0, 0);
                     pos_ext.SetXYZ(0, 0, 0);
                  } // try

                  // mom_res = mom_ext;
                  // pos_res = pos_ext;
                  xiniFitXtr = pos_ext.X();
                  yiniFitXtr = pos_ext.Y();
                  ziniFitXtr = pos_ext.Z();

                  // std::cout << cYELLOW << " Extrapolation: Total Momentum : " << mom_ext.Mag()
                  // << " - Position : " << pos_ext.X() << "  " << pos_ext.Y() << "  " << pos_ext.Z()
                  // << " - POCA : " << POCA << " - Steps : " << nSteps << cNORMAL << "\n";

                  Double_t thetaA = 0.0;
                  if (thetaConv > 90.0) {
                     thetaA = 180.0 * TMath::DegToRad() - mom_res.Theta();

                  } else {
                     thetaA = mom_res.Theta();
                  }

                  Double_t E = TMath::Sqrt(TMath::Power(mom_res.Mag(), 2) + TMath::Power(M_Ener, 2)) - M_Ener;
                  EFit = E * 1000.0;
                  EFitXtr = 1000.0 * (TMath::Sqrt(TMath::Power(mom_ext.Mag(), 2) + TMath::Power(M_Ener, 2)) - M_Ener);
                  std::cout << " Energy : " << E * 1000.0 << " - Energy Xtr : " << EFitXtr << "\n";
                  AFit = thetaA * TMath::RadToDeg();
                  PhiFit = mom_res.Phi();
                  xiniFit = pos_res.X();
                  yiniFit = pos_res.Y();
                  ziniFit = pos_res.Z();

               } // Kalman fit

            } // Kalman status

         } catch (std::exception &e) {
            std::cout << " " << e.what() << "\n";
            continue;
         }

         ROOT::Math::XYZVector iniFitVec(xiniFit, yiniFit, ziniFit);
         ROOT::Math::XYZVector iniFitXtrVec(xiniPRA, yiniPRA, ziniPRA);
         ROOT::Math::XYZVector iniPRAVec(xiniPRA, yiniPRA, ziniPRA);

         // NB: Here need the data block
         std::unique_ptr<AtFittedTrack> fittedTrack = std::make_unique<AtFittedTrack>();
         fittedTrack->SetTrackID(trackID);
         fittedTrack->SetEnergyAngles(EFit, EFitXtr, AFit, PhiFit, EPRA, APRA, PhiPRA);
         fittedTrack->SetVertexPosition(iniFitVec, iniPRAVec, iniFitXtrVec);
         fittedTrack->SetStats(pVal, fChi2, bChi2, fNdf, bNdf, fitConverged);
         fittedTrack->SetTrackProperties(particleQ, brho, eloss, dedx, std::to_string(pdg), points);
         // fittedTrack->SetIonChamber(Float_t icenergy, Int_t ictime);
         // fittedTrack->SetExcitationEnergy(Float_t exenergy, Float_t exenergyxtr);
         fittedTrack->SetDistances(distXtr, trackLength, POCA);
         fittedTracks.push_back(std::move(fittedTrack));

      } catch (std::exception &e) {
         std::cout << " Exception fitting track !" << e.what() << "\n";
         continue;
      }

   } // Merged track loop

   // std::cout<<" Fitted tracks "<<fittedTracks.size()<<"\n";
   return std::move(fittedTracks);
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
      LOG(debug) << cYELLOW << "    ---- AtGenfit : Initial parameters "
                 << "\n";
      LOG(debug) << "    PDG : " << PDGCode << " - Mass : " << p_mass << " - Atomic number : " << p_Z << "\n";
      LOG(debug) << "    B field : " << fMagneticField / 10.0 << " - Min. Bhro : " << fMinBrho
                 << "    - Max. Brho : " << fMaxBrho << "\n";
      LOG(debug) << "    Theta : " << theta * TMath::RadToDeg() << " - Phi : " << phi * TMath::RadToDeg()
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
         LOG(debug) << cYELLOW << " Is fitted? " << fitStatus->isFitted() << "\n";
         LOG(debug) << " Is Converged ? " << fitStatus->isFitConverged() << "\n";
         LOG(debug) << " Is Converged Partially? " << fitStatus->isFitConvergedPartially() << "\n";
         LOG(debug) << " Is pruned ? " << fitStatus->isTrackPruned() << cNORMAL << "\n";
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

   std::cout << " End of GENFIT "
             << "\n";
   std::cout << "               "
             << "\n";

   return gfTrack;
}

std::vector<AtTrack *> AtFITTER::AtGenfit::FindSingleTracks(std::vector<AtTrack *> &tracks)
{

   // TODO
   Double_t angleSpread = 5.0 * TMath::DegToRad();
   Double_t distThresh = 100.0;
   Double_t distMax = 100.0;
   std::vector<AtTrack *> singleTracks;
   std::vector<AtTrack *> fragmentedTracks;

   for (auto itA = tracks.begin(); itA != tracks.end(); ++itA) {
      AtTrack *trA = *(itA);
      std::cout << " Checking if track " << trA->GetTrackID() << " is single."
                << "\n";
      Bool_t isSingle = false;
      // Length - center - overlap
      for (auto itB = itA + 1; itB != tracks.end(); ++itB) {
         AtTrack *trB = *(itB);
         std::cout << " Track A : " << trA->GetTrackID() << " - Track B : " << trB->GetTrackID() << "\n";
         std::cout << " Track A Theta : " << trA->GetGeoTheta() * TMath::RadToDeg()
                   << " - Track B : " << trB->GetGeoTheta() * TMath::RadToDeg() << "\n";
         std::cout << " Track A Phi : " << trA->GetGeoPhi() * TMath::RadToDeg()
                   << " - Track B : " << trB->GetGeoPhi() * TMath::RadToDeg() << "\n";

         Double_t centerDistance = CenterDistance(trA, trB);
      }
   }

   return singleTracks;
}

Double_t AtFITTER::AtGenfit::CenterDistance(AtTrack *trA, AtTrack *trB)
{
   std::pair<Double_t, Double_t> centerA = trA->GetGeoCenter();
   std::pair<Double_t, Double_t> centerB = trB->GetGeoCenter();
   std::cout << " Center A : " << centerA.first << " - " << centerA.second << "\n";
   std::cout << " Center B : " << centerB.first << " - " << centerB.second << "\n";
   Double_t centerDistance =
      TMath::Sqrt(TMath::Power(centerA.first - centerB.first, 2) + TMath::Power(centerA.second - centerB.second, 2));
   std::cout << " Center Distance : " << centerDistance << "\n";
   return centerDistance;
}

Bool_t AtFITTER::AtGenfit::CompareTracks(AtTrack *trA, AtTrack *trB)
{

   // Matching angle
   // Matching center
   // Vertex position
   // Track overlap

   // TODO
   Double_t angleSpread = 5.0 * TMath::DegToRad();
   Double_t distThresh = 100.0;
   Double_t distMax = 150.0;

   if (trA->GetIsMerged() || trB->GetIsMerged()) {
      // std::cout << "    Skipped : merged " << std::endl;
      return false;
   }
   if (trA->GetTrackID() == trB->GetTrackID()) {
      // std::cout << "    Skipped : Same track " << std::endl;
      return false;
   }
   if (trA->GetGeoTheta() * TMath::RadToDeg() > 90 && trB->GetGeoTheta() * TMath::RadToDeg() < 90) {
      // std::cout << "    Skipped : Diff directions " << std::endl;
      return false;
   }
   if (trA->GetGeoTheta() * TMath::RadToDeg() < 90 && trB->GetGeoTheta() * TMath::RadToDeg() > 90) {
      // std::cout << "    Skipped : Diff directions " << std::endl;
      return false;
   }

   std::cout << " Comparing track A : " << trA->GetTrackID() << " and track B : " << trB->GetTrackID() << "\n";

   if ((trB->GetGeoTheta() - angleSpread) <= trA->GetGeoTheta() &&
       trA->GetGeoTheta() <= (trB->GetGeoTheta() + angleSpread)) {

      Double_t centerDistance = CenterDistance(trA, trB);

      if (centerDistance < distThresh) {

         if (!CheckOverlap(trA, trB)) {

            Double_t iniA = 0.0;
            Double_t endA = 0.0;
            Double_t iniB = 0.0;
            Double_t endB = 0.0;
            Double_t dist = 0.0;  // 3D distance
            Double_t distR = 0.0; // Radial distance
            AtHitCluster iniClusterA;
            AtHitCluster iniClusterB;
            AtHitCluster endClusterA;
            AtHitCluster endClusterB;

            // Find distance between end points of the tracks assuming no overlap
            if (trA->GetGeoTheta() * TMath::RadToDeg() < 90) {
               iniClusterA = trA->GetHitClusterArray()->back();
               iniClusterB = trB->GetHitClusterArray()->back();
               endClusterA = trA->GetHitClusterArray()->front();
               endClusterB = trB->GetHitClusterArray()->front();
               iniA = 1000.0 - iniClusterA.GetPosition().Z();
               iniB = 1000.0 - iniClusterB.GetPosition().Z();
               endA = 1000.0 - endClusterA.GetPosition().Z();
               endB = 1000.0 - endClusterB.GetPosition().Z();

            } else if (trA->GetGeoTheta() * TMath::RadToDeg() > 90) {
               iniClusterA = trA->GetHitClusterArray()->front();
               iniClusterB = trB->GetHitClusterArray()->front();
               endClusterA = trA->GetHitClusterArray()->back();
               endClusterB = trB->GetHitClusterArray()->back();
               iniA = iniClusterA.GetPosition().Z();
               iniB = iniClusterB.GetPosition().Z();
               endA = endClusterA.GetPosition().Z();
               endB = endClusterB.GetPosition().Z();
            }
            // std::cout<<" Track A Ini : "<<iniA<<"\n";
            // std::cout<<" Track B Ini : "<<iniB<<"\n";
            // std::cout<<" Track A End : "<<endA<<"\n";
            // std::cout<<" Track B End : "<<endB<<"\n";

            if (iniA < iniB) {
               dist = std::sqrt((endClusterA.GetPosition() - iniClusterB.GetPosition()).Mag2());
               distR = TMath::Sqrt(TMath::Power(endClusterA.GetPosition().X() - iniClusterB.GetPosition().X(), 2) +
                                   TMath::Power(endClusterA.GetPosition().Y() - iniClusterB.GetPosition().Y(), 2));
            } else {
               dist = std::sqrt((endClusterB.GetPosition() - iniClusterA.GetPosition()).Mag2());
               distR = TMath::Sqrt(TMath::Power(iniClusterA.GetPosition().X() - endClusterB.GetPosition().X(), 2) +
                                   TMath::Power(iniClusterA.GetPosition().Y() - endClusterB.GetPosition().Y(), 2));
            }

            // std::cout<<" -- Distance between tracks "<<dist<<"\n";
            // std::cout<<" -- Radial distance between tracks "<<distR<<"\n";

            std::cout << " --- Tracks valid for merging! - " << trA->GetTrackID() << " - " << trB->GetTrackID() << "\n";
            return true;

         } else
            std::cout << " Track Overlap found "
                      << " - " << trA->GetTrackID() << " - " << trB->GetTrackID() << "\n";
         return false;
      }

   } // Conditions
   return false;
}

Bool_t AtFITTER::AtGenfit::CheckOverlap(AtTrack *trA, AtTrack *trB)
{
   auto &hitArrayA = trA->GetHitArray();
   auto &hitArrayB = trB->GetHitArray();

   Int_t iTBMatch = 0;

   for (auto itA = hitArrayA.begin(); itA != hitArrayA.end(); ++itA) {

      std::vector<Int_t> iTBMatches;
      auto itB = hitArrayB.begin();
      while ((itB = std::find_if(itB, hitArrayB.end(), [&itA](std::unique_ptr<AtHit> &hitB) {
                 return hitB->GetTimeStamp() == (*itA)->GetTimeStamp();
              })) != hitArrayB.end()) {

         iTBMatches.push_back(std::distance(hitArrayB.begin(), itB));
         itB++;
      }

      /*if(iTBMatches.size()>0){
      std::cout<<" TB Matches found for track A TB: "<<itA->GetTimeStamp()<<" at position
     "<<std::distance(hitArrayA.begin(), itA)<<"\n";

         for(auto itb =0;itb<iTBMatches.size();++itb)
     {
       std::cout<<" Index : "<<itb<<" - TB : "<<hitArrayB.at(itb).GetTimeStamp()<<"\n";
     }
     }*/

      iTBMatch += iTBMatches.size();
   }

   Double_t shortStraw = (hitArrayA.size() < hitArrayB.size()) ? hitArrayA.size() : hitArrayB.size();
   // TODO: % of overlap. Counted twice!
   // std::cout<<" Overlap "<<shortStraw<<" "<<iTBMatch<<"\n";
   if (iTBMatch > (shortStraw * 0.1))
      return true;
   else
      return false;
}

ClassImp(AtFITTER::AtGenfit);
