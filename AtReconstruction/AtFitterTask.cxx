#include "AtFitterTask.h"

#include "AtDigiPar.h"
#include "AtFitter.h"
#include "AtGenfit.h"
#include "AtParsers.h"
#include "AtPatternEvent.h"

#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairRun.h>
#include <FairRuntimeDb.h>
#include <FairTask.h>

#include <TClonesArray.h>
#include <TObject.h>
#include <Track.h>

#include <algorithm>
#include <iostream>

class AtTrack;

ClassImp(AtFitterTask);

AtFitterTask::AtFitterTask(std::unique_ptr<AtFITTER::AtFitter> fitter)
   : fInputBranchName("AtPatternEvent"), fOutputBranchName("AtTrackingEvent"), fIsPersistence(kFALSE),
     fTrackingEventArray(TClonesArray("AtTrackingEvent", 1)), fFitter(std::move(fitter))
{
   fTrackTransformer = std::make_unique<AtTools::AtTrackTransformer>();
   fKinematics = std::make_shared<AtTools::AtKinematics>();
}

void AtFitterTask::SetPersistence(Bool_t value)
{
   fIsPersistence = value;
}

void AtFitterTask::SetInputBranch(TString branchName)
{
   fInputBranchName = branchName;
}

void AtFitterTask::SetOutputBranch(TString branchName)
{
   fOutputBranchName = branchName;
}

InitStatus AtFitterTask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(error) << "Cannot find RootManager!";
      return kERROR;
   }

   fPatternEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject("AtPatternEvent"));
   if (fPatternEventArray == nullptr) {
      LOG(error) << "Cannot find AtPatternEvent array!";
      return kERROR;
   }

      std::cout << cGREEN << " AtFitterTask::Init - Fit parameters. "
                << "\n";
      std::cout << " Magnetic Field       : " << fMagneticField << " T\n";
      std::cout << " PDG Code             : " << fPDGCode << "\n";
      std::cout << " Mass                 : " << fMass << " amu\n";
      std::cout << " Atomic Number        : " << fAtomicNumber << "\n";
      std::cout << " Number of fit points : " << fNumFitPoints << "\n";
      std::cout << " Maximum iterations   : " << fMaxIterations << "\n";
      std::cout << " Minimum iterations   : " << fMinIterations << "\n";
      std::cout << " Maximum brho         : " << fMaxBrho << "\n";
      std::cout << " Minimum brho         : " << fMinBrho << "\n";
      std::cout << " Energy loss file     : " << fELossFile << "\n";
      std::cout << " --------------------------------------------- " << cNORMAL << "\n";

      // NB This block needs to be moved to the macro
      /*fFitter)->SetPDGCode(fPDGCode);
      dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetMass(fMass);
      dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetAtomicNumber(fAtomicNumber);
      dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetNumFitPoints(fNumFitPoints);*/

      // ioMan -> Register("genfitTrackTCA","ATTPC",fGenfitTrackArray, fIsPersistence);
      // ioMan->RegisterAny("ATTPC", fGenfitTrackVector, fIsPersistence);

      ioMan->Register(fOutputBranchName, "AtTPC", &fTrackingEventArray, fIsPersistence);

      return kSUCCESS;
}

void AtFitterTask::SetParContainers()
{
   LOG(debug) << "SetParContainers of AtFitterTask";

   FairRun *run = FairRun::Instance();
   if (!run)
      LOG(fatal) << "No analysis run!";

   FairRuntimeDb *db = run->GetRuntimeDb(); // NOLINT
   if (!db)
      LOG(fatal) << "No runtime database!";

   fPar = (AtDigiPar *)db->getContainer("AtDigiPar"); // NOLINT
   if (!fPar)
      LOG(fatal) << "AtDigiPar not found!!";
}

void AtFitterTask::Exec(Option_t *option)
{
   if (fPatternEventArray->GetEntriesFast() == 0)
      return;

   fTrackingEventArray.Clear("C");

   // fFitter->Init();

   // Global variables for AtFittedTrack
   Int_t trackID = -1;
   Float_t xiniPRA = -100;
   Float_t yiniPRA = -100;
   Float_t ziniPRA = -1000;
   Float_t EPRA = 0;
   Float_t APRA = 0;
   Float_t PhiPRA = 0;
   std::string PDG = "2212";
   //////////////////////////////

   std::cout << " Event Counter " << fEventCnt << "\n";

   AtPatternEvent &patternEvent = *(dynamic_cast<AtPatternEvent *>(fPatternEventArray->At(0)));
   std::vector<AtTrack> &tracks = patternEvent.GetTrackCand();
   std::cout << " AtFitterTask:Exec -  Number of candidate tracks : " << tracks.size() << "\n";

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
         std::cout << cRED << " Track has less than 3 clusters! " << cNORMAL << "\n";
         continue;
      }

      if (track.GetTrackID() == -1) {
         std::cout << cRED << " Track is noise! " << cNORMAL << "\n";
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
      std::cout << cBLUE << "   Track ID " << track.GetTrackID() << "\n";
      std::cout << "   Initial position : " << xiniPRA << " - " << yiniPRA << " - " << ziniPRA << " "
                << iniCluster.GetTimeStamp() << "\n";
      std::cout << "   Second position : " << secPos.X() << " - " << secPos.Y() << " - " << secPos.Z() << " "
                << secCluster.GetTimeStamp() << "\n";
      std::cout << "   End position : " << endPos.X() << " - " << endPos.Y() << " - " << zEndCal << " "
                << endCluster.GetTimeStamp() << "\n";
      std::cout << "   Theta (PRA) " << track.GetGeoTheta() * TMath::RadToDeg()
                << "   Theta (convention) : " << thetaConv << " - Phi Clus : " << phiClus * TMath::RadToDeg() << "\n";
      std::cout << "   Track center - X :  " << center.first << " - Y : " << center.second << "\n";
      std::cout << "   Track phi recalc : " << track.GetGeoPhi() * TMath::RadToDeg() << cNORMAL << "\n";

      // Skip tracks that are far from Z (to be checked against number of iterations for extrapolation)
      Double_t dist = TMath::Sqrt(iniPos.X() * iniPos.X() + iniPos.Y() * iniPos.Y());
      std::cout << KRED << "    Distance to Z " << dist << cNORMAL << "\n";
      std::cout << KGRN << "    ---- Adding track candidate " << cNORMAL << "\n";
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
            Bool_t merged =
               fFitter->MergeTracks(&candToMergePool, &mergedTrackPool, fEnableSingleVertexTrack, fClusterRadius,
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
      Double_t brho = fMagneticField * radius / TMath::Sin(theta); // Tm
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

      fFitter->Init();

      std::vector<Int_t> pdgCandFit;
      if (thetaConv > 90) {

         switch (fExpNum) {
         case e20020: pdgCandFit.push_back(1000010020); break;
         case e20009: pdgCandFit.push_back(2212); break;
         }

      } else if (thetaConv < 90 && thetaConv > 10) {

         switch (fExpNum) {
         case e20020: pdgCandFit.push_back(1000020040); break;
         case e20009: pdgCandFit.push_back(1000010020); break;
         case a1954: pdgCandFit.push_back(2212); break;
         case a1954b: pdgCandFit.push_back(2212); break;
         // case a1954b: pdgCandFit.push_back(1000010020); break;
         case a1975: pdgCandFit.push_back(2212); break;
         }

      } else if (thetaConv < 10) {

         switch (fExpNum) {
         case e20009:
            pdgCandFit.push_back(1000040100);
            break;
            // pdgCandFit.push_back(1000040110);
         }
      }

      try {

         genfit::Track *fitTrack = fFitter->FitTracks(&track);

         Int_t atomicNumber = 0;
         Double_t mass = 0;
         Double_t M_Ener = 0.0;

         // PDG needs to be defined
         Int_t pdg = pdgCandFit.at(0);

         auto fIl =
            std::find_if(ionList->begin(), ionList->end(), [&pdg](AtTools::IonFitInfo ion) { return ion._PDG == pdg; });
         if (fIl != ionList->end()) {

            int index = std::distance(ionList->begin(), fIl);
            std::cout << cBLUE << "  -  Ion info for : " << pdg << " found in " << index << cNORMAL << "\n";
            atomicNumber = ionList->at(index)._atomicNumber;
            mass = ionList->at(index)._mass;
            M_Ener = mass * 931.49401 / 1000.0;
         }

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
         /*pVal = -1;
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
         particleQ = -10.0;*/

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

               if (((Float_t)cnt / (Float_t)hitClusterArray->size()) > 0.5)
                  break;
               auto dir = (*it).GetPosition() - (*std::next(it, 1)).GetPosition();
               eloss += (*it).GetCharge();
               len += std::sqrt(dir.Mag2());
               dedx += (*it).GetCharge();
               // std::cout<<(*it).GetCharge()<<"\n";
               it++;
               ++cnt;
            }
         } else if (thetaConv > 90) {

            eloss += hitClusterArray->at(0).GetCharge();

            cnt = 1;
            for (auto iHitClus = 1; iHitClus < hitClusterArray->size(); ++iHitClus) {

               if (((Float_t)cnt / (Float_t)hitClusterArray->size()) > 0.5)
                  break;
               auto dir = hitClusterArray->at(iHitClus).GetPosition() - hitClusterArray->at(iHitClus - 1).GetPosition();
               len += std::sqrt(dir.Mag2());
               eloss += hitClusterArray->at(iHitClus).GetCharge();
               dedx += hitClusterArray->at(iHitClus).GetCharge();
               // std::cout<<len<<" - "<<eloss<<" - "<<hitClusterArray->at(iHitClus).GetCharge()<<"\n";
               ++cnt;
            }
         }

         eloss /= cnt;
         dedx /= len;

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
                  /*particleQ = fitState.getCharge();

                  fChi2 = KalmanFitStatus->getForwardChi2();
                  bChi2 = KalmanFitStatus->getBackwardChi2();
                  fNdf = KalmanFitStatus->getForwardNdf();
                  bNdf = KalmanFitStatus->getBackwardNdf();*/
                  // fitState.Print();
                  fitState.getPosMomCov(pos_res, mom_res, cov_res);
                  // trackLength = KalmanFitStatus->getTrackLen();
                  // pVal = KalmanFitStatus->getPVal();

                  // fKalmanFitter -> getChiSquNdf(gfTrack, trackRep, bChi2, fChi2, bNdf, fNdf);
                  Float_t stepXtr = -0.01;
                  Int_t minCnt = 0;
                  Int_t minCntExt = 0;

                  TVector3 pos_ini_buff = pos_res;
                  Double_t length = 0.0;
                  // mom_ext = fitState.getMom();
                  // pos_ext = fitState.getPos();

                  // Backward extrapolation
                  try {

                     for (auto iStep = 0; iStep < 200; ++iStep) {

                        trackRep->extrapolateBy(fitState, stepXtr * iStep);
                        // mom_ext_buff = fitState.getMom();
                        // pos_ext_buff = fitState.getPos();

                        length += (pos_ext_buff - pos_ini_buff).Mag();
                        pos_ini_buff = pos_ext_buff;

                        double distPOCA =
                           TMath::Sqrt(pos_ext_buff.X() * pos_ext_buff.X() + pos_ext_buff.Y() * pos_ext_buff.Y());
                        // if (fVerbosityLevel > 2){
                        /*std::cout << cYELLOW << " Extrapolation: Total Momentum : " << mom_ext_buff.Mag()
                                  << " - Position : " << pos_ext_buff.X() << "  " << pos_ext_buff.Y() << "  "
                                  << pos_ext_buff.Z() << " - distance of approach : " << distance <<" - length :
                           "<<length<< cNORMAL << "\n";*/
                        //}

                        // if(pos_ext_buff.Z()<0)
                        // break;

                        if (distPOCA < POCA) {
                           /*POCA = distPOCA;
                           POCAXtr = distPOCA;
                           mom_ext = mom_ext_buff;
                           pos_ext = pos_ext_buff;
                           distXtr = iStep * stepXtr;
                           ++minCnt;
                           minCntExt = 0;
                           nSteps = iStep;*/
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
                  // xiniFitXtr = pos_ext.X();
                  // yiniFitXtr = pos_ext.Y();
                  // ziniFitXtr = pos_ext.Z();

                  /*std::cout << cYELLOW << " Extrapolation: Total Momentum : " << mom_ext.Mag()
                            << " - Position : " << pos_ext.X() << "  " << pos_ext.Y() << "  " << pos_ext.Z()
                            << " - POCA : " << POCA << " - Steps : " << nSteps << cNORMAL << "\n";*/

                  Double_t thetaA = 0.0;
                  if (thetaConv > 90.0) {
                     thetaA = 180.0 * TMath::DegToRad() - mom_res.Theta();

                  } else {
                     thetaA = mom_res.Theta();
                  }

                  Double_t E = TMath::Sqrt(TMath::Power(mom_res.Mag(), 2) + TMath::Power(M_Ener, 2)) - M_Ener;
                  /*EFit = E * 1000.0;
                  EFitXtr =
                     1000.0 * (TMath::Sqrt(TMath::Power(mom_ext.Mag(), 2) + TMath::Power(M_Ener, 2)) - M_Ener);
                  std::cout << " Energy : " << E * 1000.0 << " - Energy Xtr : " << EFitXtr << "\n";
                  AFit = thetaA * TMath::RadToDeg();
                  PhiFit = mom_res.Phi();
                  xiniFit = pos_res.X();
                  yiniFit = pos_res.Y();
                  ziniFit = pos_res.Z();*/

               } // Kalman fit

            } // Kalman status

         } catch (std::exception &e) {
            std::cout << " " << e.what() << "\n";
            continue;
         }

         // NB: Here need the data block

      } catch (std::exception &e) {
         std::cout << " Exception fitting track !" << e.what() << "\n";
         continue;
      }

   } // Merged track loop

   ++fEventCnt;
}

std::vector<AtTrack *> AtFitterTask::FindSingleTracks(std::vector<AtTrack *> &tracks)
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

Double_t AtFitterTask::CenterDistance(AtTrack *trA, AtTrack *trB)
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

Bool_t AtFitterTask::CompareTracks(AtTrack *trA, AtTrack *trB)
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

Bool_t AtFitterTask::CheckOverlap(AtTrack *trA, AtTrack *trB)
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