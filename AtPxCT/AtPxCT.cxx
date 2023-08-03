/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtPxCT.h"

#include "AtDetectorList.h"
#include "AtMCPoint.h"
#include "AtStack.h"

#include <FairLogger.h> // for Logger, LOG
#include <FairRootManager.h>
#include <FairVolume.h>

#include <TClonesArray.h>
#include <TLorentzVector.h>
#include <TVirtualMC.h>
#include <TVirtualMCStack.h> // for TVirtualMCStack

#include <iostream>
using std::cout;
using std::endl;

AtPxCT::AtPxCT()
   : FairDetector("AtPxCT", kTRUE, kAtPxCT), fTrackID(-1), fVolumeID(-1), fDetCopyID(-1), fTime(-1.), fLength(-1.),
     fELoss(-1), fAtPxCTPointCollection(new TClonesArray("AtMCPoint")), fELossAcc(-1)
{
}

AtPxCT::AtPxCT(const char *name, Bool_t active)
   : FairDetector(name, active, kAtPxCT), fTrackID(-1), fVolumeID(-1), fDetCopyID(-1), fTime(-1.), fLength(-1.),
     fELoss(-1), fAtPxCTPointCollection(new TClonesArray("AtMCPoint")), fELossAcc(-1)
{
}

AtPxCT::~AtPxCT()
{
   if (fAtPxCTPointCollection) {
      fAtPxCTPointCollection->Delete();
      delete fAtPxCTPointCollection;
   }
}

void AtPxCT::Initialize()
{
   FairDetector::Initialize();
   // AtPxCTGeoPar* par=(AtPxCTGeoPar*)(rtdb->getContainer("AtPxCTGeoPar"));
   LOG(INFO) << "AtPxCT: initialisation";
}

Bool_t AtPxCT::ProcessHits(FairVolume *vol)
{
   /** This method is called from the MC stepping */

   auto *stack = dynamic_cast<AtStack *>(gMC->GetStack());
   fVolName = gMC->CurrentVolName();

   TLorentzVector fPosIn;
   TLorentzVector fMomIn;

   if (gMC->IsTrackEntering()) {
      fELoss = 0.;
      fELossAcc = 0.;
      fTime = gMC->TrackTime() * 1.0e09;
   }

   //
   fELoss = gMC->Edep(); // in GeV
   fELossAcc += fELoss;

   // Set additional parameters at exit of active volume. Create Point.
   if (gMC->IsTrackExiting() || gMC->IsTrackStop() || gMC->IsTrackDisappeared()) {
      fTrackID = gMC->GetStack()->GetCurrentTrackNumber();
      fVolumeID = vol->getMCid();
      fDetCopyID = vol->getCopyNo();
      fLength = gMC->TrackLength();
      gMC->TrackPosition(fPosIn);
      gMC->TrackMomentum(fMomIn);

      if (fELossAcc == 0.)
         return kFALSE;

      AddPoint(fTrackID, fVolumeID, TVector3(fPosIn.X(), fPosIn.Y(), fPosIn.Z()),
               TVector3(fMomIn.X(), fMomIn.Y(), fMomIn.Z()), fTime, fLength, fELossAcc);

      stack->AddPoint(kAtPxCT);
   }
   return kTRUE;
}

void AtPxCT::EndOfEvent()
{
   Print();
   Reset();
}

void AtPxCT::Register()
{

   /** This will create a branch in the output tree called
       AtPxCTPoint, setting the last parameter to kFALSE means:
       this collection will not be written to the file, it will exist
       only during the simulation.
   */

   FairRootManager::Instance()->Register("AtMCPoint", "AtPxCT", fAtPxCTPointCollection, kTRUE);
}

TClonesArray *AtPxCT::GetCollection(Int_t iColl) const
{
   if (iColl == 0) {
      return fAtPxCTPointCollection;
   } else {
      return nullptr;
   }
}

void AtPxCT::Reset()
{
   fAtPxCTPointCollection->Clear();
}

void AtPxCT::Print(Option_t *option) const
{
   Int_t nHits = fAtPxCTPointCollection->GetEntriesFast();
   LOG(INFO) << "PxCT: " << nHits << " points registered in this event";
}

void AtPxCT::ConstructGeometry()
{
   TString fileName = GetGeometryFileName();
   if (fileName.EndsWith(".geo")) {
      LOG(INFO) << "Constructing PxCT geometry from ASCII file " << fileName;
      // ConstructASCIIGeometry();
   } else if (fileName.EndsWith(".root")) {
      LOG(INFO) << "Constructing PxCT geometry from ROOT file " << fileName;
      ConstructRootGeometry();
   } else {
      std::cout << "Geometry format not supported." << std::endl;
   }
}

Bool_t AtPxCT::CheckIfSensitive(std::string name)
{
   TString tsname = name;
   if (tsname.Contains("Crystal_")) {
      LOG(INFO) << " PxCT geometry: Sensitive volume found: " << tsname;
      return kTRUE;
   }
   return kFALSE;
}

// -----   Private method AddPoint   --------------------------------------------
AtMCPoint *
AtPxCT::AddPoint(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Double_t time, Double_t length, Double_t eLoss)
{
   TClonesArray &clref = *fAtPxCTPointCollection;
   Int_t size = clref.GetEntriesFast();
   if (fVerboseLevel > 1)
      LOG(INFO) << "PxCT: Adding Point in detector " << detID << ", track " << trackID << ", energy loss "
                << eLoss * 1e06 << " keV";

   auto point = new (clref[size]) AtMCPoint(trackID, detID, pos, mom, time, length, eLoss);
   point->SetVolName(fVolName);
   point->SetDetCopyID(fDetCopyID);
   return point;
}

ClassImp(AtPxCT)
