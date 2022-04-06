/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtTpc.h"

#include <FairDetector.h>
#include <TLorentzVector.h>
#include <TVector3.h>
#include <TVirtualMCStack.h>
#include <stddef.h>
#include <iostream>

#include "AtTpcGeoPar.h"
#include "AtVertexPropagator.h"
#include "AtDetectorList.h"
#include "AtMCPoint.h"
#include "AtStack.h"
#include <FairVolume.h>
#include <FairRootManager.h>
#include <FairRun.h>
#include <FairRuntimeDb.h>
#include "TClonesArray.h"
#include "TVirtualMC.h"
#include "TGeoManager.h"
#include "fairlogger/Logger.h"

using std::cout;
using std::endl;

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"
#define cBLUE "\033[1;34m"

AtTpc::AtTpc()
   : FairDetector("AtTpc", kTRUE, kAtTpc), fTrackID(-1), fVolumeID(-1), fPos(), fMom(), fTime(-1.), fLength(-1.),
     fELoss(-1), fPosIndex(-1), fAtTpcPointCollection(new TClonesArray("AtMCPoint")), fELossAcc(-1)
{
}

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
   AtTpcGeoPar *par = (AtTpcGeoPar *)(rtdb->getContainer("AtTpcGeoPar"));
}

void AtTpc::trackEnteringVolume()
{
   auto AZ = DecodePdG(gMC->TrackPid());
   fELoss = 0.;
   fELossAcc = 0.;
   fTime = gMC->TrackTime() * 1.0e09;
   fLength = gMC->TrackLength();
   gMC->TrackPosition(fPosIn);
   gMC->TrackMomentum(fMomIn);
   fTrackID = gMC->GetStack()->GetCurrentTrackNumber();

   // Position of the first hit of the beam in the TPC volume ( For tracking purposes in the TPC)
   if (gAtVP->GetBeamEvtCnt() % 2 != 0 && fTrackID == 0 && (fVolName == "drift_volume" || fVolName == "cell"))
      InPos = fPosIn;

   Int_t VolumeID;

   if (gAtVP->GetBeamEvtCnt() % 2 != 0)
      LOG(debug) << cGREEN << " AtTPC: Beam Event ";
   else if (gAtVP->GetDecayEvtCnt() % 2 == 0)
      LOG(debug) << cBLUE << " AtTPC: Reaction/Decay Event ";

   LOG(debug) << " AtTPC: First hit in Volume " << fVolName;
   LOG(debug) << " Particle : " << gMC->ParticleName(gMC->TrackPid());
   LOG(debug) << " PID PdG : " << gMC->TrackPid();
   LOG(debug) << " Atomic Mass : " << AZ.first;
   LOG(debug) << " Atomic Number : " << AZ.second;
   LOG(debug) << " Volume ID " << gMC->CurrentVolID(VolumeID);
   LOG(debug) << " Track ID : " << fTrackID;
   LOG(debug) << " Position : " << fPosIn.X() << " " << fPosIn.Y() << "  " << fPosIn.Z();
   LOG(debug) << " Momentum : " << fMomIn.X() << " " << fMomIn.Y() << "  " << fMomIn.Z();
   LOG(debug) << " Total relativistic energy " << gMC->Etot();
   LOG(debug) << " Mass of the Beam particle (gAVTP) : " << gAtVP->GetBeamMass();
   LOG(debug) << " Mass of the Tracked particle (gMC) : " << gMC->TrackMass(); // NB: with electrons
   LOG(debug) << " Initial energy of the beam particle in this volume : "
              << ((gMC->Etot() - gAtVP->GetBeamMass() * 0.93149401) * 1000.); // Relativistic Mass
   LOG(debug) << " Total energy of the current track (gMC) : "
              << ((gMC->Etot() - gMC->TrackMass()) * 1000.); // Relativistic Mass
   LOG(debug) << " ==================================================== " << cNORMAL;
}

void AtTpc::getTrackParametersFromMC()
{
   fELoss = gMC->Edep();
   fELossAcc += fELoss;
   fTime = gMC->TrackTime() * 1.0e09;
   fLength = gMC->TrackLength();
   gMC->TrackPosition(fPosIn);
   gMC->TrackMomentum(fMomIn);
   fTrackID = gMC->GetStack()->GetCurrentTrackNumber();
}

void AtTpc::getTrackParametersWhileExiting()
{
   fTrackID = gMC->GetStack()->GetCurrentTrackNumber();
   gMC->TrackPosition(fPosOut);
   gMC->TrackMomentum(fMomOut);

   // Correct fPosOut
   if (gMC->IsTrackExiting()) {
      correctPosOut();
      if ((fVolName == "drift_volume" || fVolName == "cell") && gAtVP->GetBeamEvtCnt() % 2 != 0 && fTrackID == 0)
         resetVertex();
   }
}

void AtTpc::resetVertex()
{
   gAtVP->ResetVertex();
   LOG(INFO) << cRED << " - AtTpc Warning : Beam punched through the AtTPC. Reseting Vertex! " << cNORMAL << std::endl;
}

void AtTpc::correctPosOut()
{
   const Double_t *oldpos;
   const Double_t *olddirection;
   Double_t newpos[3];
   Double_t newdirection[3];
   Double_t safety;

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
   bool atEnergyLoss = fELossAcc * 1000 > gAtVP->GetRndELoss();
   bool isPrimaryBeam = gAtVP->GetBeamEvtCnt() % 2 != 0 && fTrackID == 0;
   bool isInRightVolume = fVolName == "drift_volume" || fVolName == "cell";
   return atEnergyLoss && isPrimaryBeam && isInRightVolume;
}
Bool_t AtTpc::ProcessHits(FairVolume *vol)
{
   /** This method is called from the MC stepping */

   AtStack *stack = (AtStack *)gMC->GetStack();
   fVolName = gMC->CurrentVolName();
   fVolumeID = vol->getMCid();
   fDetCopyID = vol->getCopyNo();

   if (gMC->IsTrackEntering())
      trackEnteringVolume();

   getTrackParametersFromMC();

   if (gMC->IsTrackExiting() || gMC->IsTrackStop() || gMC->IsTrackDisappeared())
      getTrackParametersWhileExiting();

   addHit();

   // Reaction Occurs here
   if (reactionOccursHere())
      startReactionEvent();

   // Increment number of AtTpc det points in TParticle
   stack->AddPoint(kAtTpc);
   return kTRUE;
}

void AtTpc::startReactionEvent()
{

   gMC->StopTrack();
   gAtVP->ResetVertex();

   TLorentzVector StopPos;
   TLorentzVector StopMom;
   gMC->TrackPosition(StopPos);
   gMC->TrackMomentum(StopMom);
   Double_t StopEnergy = ((gMC->Etot() - gAtVP->GetBeamMass() * 0.93149401) * 1000.);

   LOG(debug) << cYELLOW << " Beam energy loss before reaction : " << fELossAcc * 1000;
   LOG(debug) << " Mass of the Tracked particle : " << gMC->TrackMass();
   LOG(debug) << " Mass of the Beam particle (gAVTP)  : " << gAtVP->GetBeamMass();
   LOG(debug) << " Total energy of the Beam particle before reaction : " << StopEnergy << cNORMAL; // Relativistic Mass

   gAtVP->SetVertex(StopPos.X(), StopPos.Y(), StopPos.Z(), InPos.X(), InPos.Y(), InPos.Z(), StopMom.Px(), StopMom.Py(),
                    StopMom.Pz(), StopEnergy);
}

void AtTpc::addHit()
{
   auto AZ = DecodePdG(gMC->TrackPid());

   Double_t EIni, AIni;

   // We assume that the beam-like particle is fTrackID==0 since it is the first one added in the
   // primary generator
   if (gAtVP->GetBeamEvtCnt() % 2 != 0 && fTrackID == 0) {
      EIni = 0;
      AIni = 0;
   } else if (gAtVP->GetDecayEvtCnt() % 2 == 0) {
      EIni = gAtVP->GetTrackEnergy(fTrackID);
      AIni = gAtVP->GetTrackAngle(fTrackID);
   }

   AddHit(fTrackID, fVolumeID, fVolName, fDetCopyID, TVector3(fPosIn.X(), fPosIn.Y(), fPosIn.Z()),
          TVector3(fMomIn.Px(), fMomIn.Py(), fMomIn.Pz()), fTime, fLength, fELoss, EIni, AIni, AZ.first, AZ.second);
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
      return NULL;
   }
}

void AtTpc::Reset()
{
   fAtTpcPointCollection->Clear();
}

void AtTpc::Print(Option_t *option) const
{
   Int_t nHits = fAtTpcPointCollection->GetEntriesFast();
   LOG(INFO) << "AtTPC: " << nHits << " points registered in this event";
}

void AtTpc::ConstructGeometry()
{
   TString fileName = GetGeometryFileName();
   if (fileName.EndsWith(".geo")) {
      LOG(INFO) << "Constructing AtTPC geometry from ASCII file " << fileName;
      // ConstructASCIIGeometry();
   } else if (fileName.EndsWith(".root")) {
      LOG(INFO) << "Constructing AtTPC geometry from ROOT file " << fileName;
      ConstructRootGeometry();
   } else {
      std::cout << "Geometry format not supported." << std::endl;
   }
}

Bool_t AtTpc::CheckIfSensitive(std::string name)
{

   TString tsname = name;
   if (tsname.Contains("drift_volume") || tsname.Contains("window") || tsname.Contains("cell")) {
      LOG(INFO) << " AtTPC geometry: Sensitive volume found: " << tsname;
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
      LOG(INFO) << "AtTPC: Adding Point at (" << pos.X() << ", " << pos.Y() << ", " << pos.Z() << ") cm,  detector "
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
