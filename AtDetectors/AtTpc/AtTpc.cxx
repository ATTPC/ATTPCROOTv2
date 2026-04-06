/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtTpc.h"

#include "AtDetectorList.h"
#include "AtMCPoint.h"
#include "AtStack.h"
#include "AtVertexPropagator.h"

#include <FairDetector.h>
#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairRun.h>
#include <FairRuntimeDb.h>
#include <FairVolume.h>

#include <TClonesArray.h>
#include <TGeoManager.h>
#include <TLorentzVector.h>
#include <TVector3.h>
#include <TVirtualMC.h>
#include <TVirtualMCStack.h>

#include <iostream>

using std::cout;
using std::endl;

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

AtTpc::AtTpc() : AtTpc("AtTpc", true) {}

AtTpc::AtTpc(const char *name, Bool_t active)
   : FairDetector(name, active, kAtTpc), fTrackID(-1), fVolumeID(-1), fPos(), fMom(), fTime(-1.), fLength(-1.),
     fELoss(-1), fPosIndex(-1), fAtTpcPointCollection(new TClonesArray("AtMCPoint")), fELossAcc(-1)
{
}

AtTpc::~AtTpc()
{
   if (fAtTpcPointCollection) {
      fAtTpcPointCollection->Delete();
      delete fAtTpcPointCollection;
   }
}

void AtTpc::Initialize()
{
   FairDetector::Initialize();
   FairRuntimeDb *rtdb = FairRun::Instance()->GetRuntimeDb();
   rtdb->getContainer("AtTpcGeoPar");
}

void AtTpc::trackEnteringVolume(const StepState &step)
{
   auto AZ = DecodePdG(step.pdg);
   fELoss = 0.;
   fELossAcc = 0.;
   fTime = step.timeNs;
   fLength = step.trackLength;
   fPosIn = step.pos;
   fMomIn = step.mom;
   fTrackID = step.trackID;

   // Position of the first hit of the beam in the TPC volume ( For tracking purposes in the TPC)
   if (fIsBeamTrack && IsReactionVolume(fVolName))
      InPos = fPosIn;

   Int_t VolumeID = 0;

   if (fIsBeamTrack)
      LOG(debug) << cGREEN << " AtTPC: Beam Event ";
   else
      LOG(debug) << cBLUE << " AtTPC: Reaction/Decay Event ";

   LOG(debug) << " AtTPC: First hit in Volume " << fVolName;
   LOG(debug) << " PID PdG : " << step.pdg;
   LOG(debug) << " Atomic Mass : " << AZ.first;
   LOG(debug) << " Atomic Number : " << AZ.second;
   LOG(debug) << " Volume ID " << step.volumeID;
   LOG(debug) << " Track ID : " << fTrackID;
   LOG(debug) << " Position : " << fPosIn.X() << " " << fPosIn.Y() << "  " << fPosIn.Z();
   LOG(debug) << " Momentum : " << fMomIn.X() << " " << fMomIn.Y() << "  " << fMomIn.Z();
   LOG(debug) << " Total relativistic energy " << step.totalEnergy;
   LOG(debug) << " Mass of the Beam particle (gAVTP) : " << AtVertexPropagator::Instance()->GetBeamMass();
   LOG(debug) << " Mass of the Tracked particle (transport) : " << step.trackMass; // NB: with electrons
   LOG(debug) << " Initial energy of the beam particle in this volume : "
              << ((step.totalEnergy - AtVertexPropagator::Instance()->GetBeamMass() * 0.93149401) *
                  1000.); // Relativistic Mass
   LOG(debug) << " Total energy of the current track (gMC) : "
              << ((step.totalEnergy - step.trackMass) * 1000.); // Relativistic Mass
   LOG(debug) << " ==================================================== " << cNORMAL;
}

void AtTpc::getTrackParametersFromStep(const StepState &step)
{
   fELoss = step.energyLoss;
   fELossAcc += fELoss;
   fTime = step.timeNs;
   fLength = step.trackLength;
   fPosIn = step.pos;
   fMomIn = step.mom;
   fTrackID = step.trackID;
}

void AtTpc::getTrackParametersWhileExiting(const StepState &step)
{
   fTrackID = step.trackID;
   fPosOut = step.posOut;
   fMomOut = step.momOut;

   if (step.exiting && IsReactionVolume(fVolName) && fIsBeamTrack)
      resetVertex();
}

void AtTpc::resetVertex()
{
   AtVertexPropagator::Instance()->ResetVertex();
   LOG(info) << cRED << " - AtTpc Warning : Beam punched through the AtTPC. Reseting Vertex! " << cNORMAL << std::endl;
}

void AtTpc::correctPosOut()
{
   const Double_t *oldpos = nullptr;
   const Double_t *olddirection = nullptr;
   Double_t newpos[3];
   Double_t newdirection[3];
   Double_t safety = 0;

   gGeoManager->FindNode(fPosOut.X(), fPosOut.Y(), fPosOut.Z());
   oldpos = gGeoManager->GetCurrentPoint();
   olddirection = gGeoManager->GetCurrentDirection();

   for (Int_t i = 0; i < 3; i++) {
      newdirection[i] = -1 * olddirection[i];
   }

   gGeoManager->SetCurrentDirection(newdirection);
   safety = gGeoManager->GetSafeDistance(); // Get distance to boundry?
   gGeoManager->SetCurrentDirection(-newdirection[0], -newdirection[1], -newdirection[2]);

   for (Int_t i = 0; i < 3; i++) {
      newpos[i] = oldpos[i] - (3 * safety * olddirection[i]);
   }

   fPosOut.SetX(newpos[0]);
   fPosOut.SetY(newpos[1]);
   fPosOut.SetZ(newpos[2]);
}

bool AtTpc::reactionOccursHere()
{
   bool atEnergyLoss = fELossAcc * 1000 > AtVertexPropagator::Instance()->GetRndELoss();
   bool isPrimaryBeam = fIsBeamTrack;
   bool isInRightVolume = IsReactionVolume(fVolName);
   return atEnergyLoss && isPrimaryBeam && isInRightVolume;
}

Bool_t AtTpc::ProcessHits(FairVolume *vol)
{
   /** This method is called from the MC stepping */

   auto *stack = dynamic_cast<AtStack *>(gMC->GetStack());
   StepState step;
   step.trackID = gMC->GetStack()->GetCurrentTrackNumber();
   step.pdg = gMC->TrackPid();
   step.volumeName = gMC->CurrentVolName();
   step.volumeID = vol->getMCid();
   step.detCopyID = vol->getCopyNo();
   step.beamTrack = step.trackID == 0;
   step.entering = gMC->IsTrackEntering();
   step.exiting = gMC->IsTrackExiting();
   step.stopping = gMC->IsTrackStop();
   step.disappeared = gMC->IsTrackDisappeared();
   step.energyLoss = gMC->Edep();
   step.timeNs = gMC->TrackTime() * 1.0e09;
   step.trackLength = gMC->TrackLength();
   step.totalEnergy = gMC->Etot();
   step.trackMass = gMC->TrackMass();
   gMC->TrackPosition(step.pos);
   gMC->TrackMomentum(step.mom);

   if (step.exiting || step.stopping || step.disappeared) {
      gMC->TrackPosition(step.posOut);
      gMC->TrackMomentum(step.momOut);
      if (step.exiting) {
         fPosOut = step.posOut;
         correctPosOut();
         step.posOut = fPosOut;
      }
   }

   bool stopTrack = ProcessStep(step);
   if (stopTrack)
      gMC->StopTrack();

   // Increment number of AtTpc det points in TParticle
   stack->AddPoint(kAtTpc);
   return kTRUE;
}

bool AtTpc::ProcessStep(const StepState &step)
{
   fVolName = step.volumeName;
   fVolumeID = step.volumeID;
   fDetCopyID = step.detCopyID;
   fIsBeamTrack = step.beamTrack;

   if (step.entering)
      trackEnteringVolume(step);

   getTrackParametersFromStep(step);

   if (step.exiting || step.stopping || step.disappeared)
      getTrackParametersWhileExiting(step);

   addHit(step);

   if (reactionOccursHere()) {
      startReactionEvent(step);
      return true;
   }

   return false;
}

void AtTpc::startReactionEvent(const StepState &step)
{
   AtVertexPropagator::Instance()->ResetVertex();

   const TLorentzVector &StopPos = step.pos;
   const TLorentzVector &StopMom = step.mom;
   Double_t StopEnergy = ((step.totalEnergy - AtVertexPropagator::Instance()->GetBeamMass() * 0.93149401) * 1000.);

   LOG(info) << "AtTpc: triggering reaction handoff at z=" << StopPos.Z() << " cm with accumulated loss "
             << fELossAcc * 1000. << " MeV and residual energy " << StopEnergy << " MeV";

   LOG(debug) << cYELLOW << " Beam energy loss before reaction : " << fELossAcc * 1000;
   LOG(debug) << " Mass of the Tracked particle : " << step.trackMass;
   LOG(debug) << " Mass of the Beam particle (gAVTP)  : " << AtVertexPropagator::Instance()->GetBeamMass();
   LOG(debug) << " Total energy of the Beam particle before reaction : " << StopEnergy << cNORMAL; // Relativistic Mass

   AtVertexPropagator::Instance()->SetVertex(StopPos.X(), StopPos.Y(), StopPos.Z(), InPos.X(), InPos.Y(), InPos.Z(),
                                             StopMom.Px(), StopMom.Py(), StopMom.Pz(), StopEnergy);
}

void AtTpc::addHit(const StepState &step)
{
   auto AZ = DecodePdG(step.pdg);

   Double_t EIni = 0;
   Double_t AIni = 0;

   // We assume that the beam-like particle is fTrackID==0 since it is the first one added in the
   // primary generator
   if (fTrackID == 0) {
      EIni = 0;
      AIni = 0;
   } else {
      EIni = AtVertexPropagator::Instance()->GetTrackEnergy(fTrackID);
      AIni = AtVertexPropagator::Instance()->GetTrackAngle(fTrackID);
   }

   AddHit(fTrackID, fVolumeID, fVolName, fDetCopyID, TVector3(fPosIn.X(), fPosIn.Y(), fPosIn.Z()),
          TVector3(fMomIn.Px(), fMomIn.Py(), fMomIn.Pz()), fTime, fLength, fELoss, EIni, AIni, AZ.first, AZ.second);
}

bool AtTpc::IsReactionVolume(const TString &volumeName) const
{
   return volumeName.Contains("drift_volume") || volumeName.Contains("cell");
}

void AtTpc::EndOfEvent()
{

   fAtTpcPointCollection->Clear();
}

void AtTpc::Register()
{
   FairRootManager::Instance()->Register("AtTpcPoint", "AtTpc", fAtTpcPointCollection, kTRUE);
}

TClonesArray *AtTpc::GetCollection(Int_t iColl) const
{
   if (iColl == 0) {
      return fAtTpcPointCollection;
   } else {
      return nullptr;
   }
}

void AtTpc::Reset()
{
   fAtTpcPointCollection->Clear();
}

void AtTpc::Print(Option_t *option) const
{
   Int_t nHits = fAtTpcPointCollection->GetEntriesFast();
   LOG(info) << "AtTPC: " << nHits << " points registered in this event";
}

void AtTpc::ConstructGeometry()
{
   TString fileName = GetGeometryFileName();
   if (fileName.EndsWith(".geo")) {
      LOG(info) << "Constructing AtTPC geometry from ASCII file " << fileName;
      // ConstructASCIIGeometry();
   } else if (fileName.EndsWith(".root")) {
      LOG(info) << "Constructing AtTPC geometry from ROOT file " << fileName;
      ConstructRootGeometry();
   } else {
      std::cout << "Geometry format not supported." << std::endl;
   }
}

Bool_t AtTpc::CheckIfSensitive(std::string name)
{

   TString tsname = name;
   if (tsname.Contains("drift_volume") || tsname.Contains("window") || tsname.Contains("cell")) {
      LOG(info) << " AtTPC geometry: Sensitive volume found: " << tsname;
      return kTRUE;
   }
   return kFALSE;
}

AtMCPoint *
AtTpc::AddHit(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Double_t time, Double_t length, Double_t eLoss)
{
   TClonesArray &clref = *fAtTpcPointCollection;
   Int_t size = clref.GetEntriesFast();
   return new (clref[size]) AtMCPoint(trackID, detID, pos, mom, time, length, eLoss);
}

// -----   Private method AddHit   --------------------------------------------
AtMCPoint *AtTpc::AddHit(Int_t trackID, Int_t detID, TString VolName, Int_t detCopyID, TVector3 pos, TVector3 mom,
                         Double_t time, Double_t length, Double_t eLoss, Double_t EIni, Double_t AIni, Int_t A, Int_t Z)
{
   TClonesArray &clref = *fAtTpcPointCollection;
   Int_t size = clref.GetEntriesFast();
   if (fVerboseLevel > 1)
      LOG(info) << "AtTPC: Adding Point at (" << pos.X() << ", " << pos.Y() << ", " << pos.Z() << ") cm,  detector "
                << detID << ", track " << trackID << ", energy loss " << eLoss * 1e06 << " keV";

   return new (clref[size])
      AtMCPoint(trackID, detID, pos, mom, time, length, eLoss, VolName, detCopyID, EIni, AIni, A, Z);
}

std::pair<Int_t, Int_t> AtTpc::DecodePdG(Int_t PdG_Code)
{
   Int_t A = PdG_Code / 10 % 1000;
   Int_t Z = PdG_Code / 10000 % 1000;

   std::pair<Int_t, Int_t> nucleus;

   if (PdG_Code == 2212) {
      nucleus.first = 1;
      nucleus.second = 1;
   } else if (PdG_Code == 2112) {
      nucleus.first = 1;
      nucleus.second = 0;
   } else {
      nucleus.first = A;
      nucleus.second = Z;
   }

   return nucleus;
}

ClassImp(AtTpc)
