/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtDeGAi.h"

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

AtDeGAi::AtDeGAi()
   : FairDetector("AtDeGAi", kTRUE, kAtDeGAi), fTrackID(-1), fVolumeID(-1), fDetCopyID(-1), fTime(-1.), fLength(-1.),
     fELoss(-1), fAtDeGAiPointCollection(new TClonesArray("AtMCPoint")), fELossAcc(-1)
{
}

AtDeGAi::AtDeGAi(const char *name, Bool_t active)
   : FairDetector(name, active, kAtDeGAi), fTrackID(-1), fVolumeID(-1), fDetCopyID(-1), fTime(-1.), fLength(-1.),
     fELoss(-1), fAtDeGAiPointCollection(new TClonesArray("AtMCPoint")), fELossAcc(-1)
{
}

AtDeGAi::~AtDeGAi()
{
   if (fAtDeGAiPointCollection) {
      fAtDeGAiPointCollection->Delete();
      delete fAtDeGAiPointCollection;
   }
}

void AtDeGAi::Initialize()
{
   FairDetector::Initialize();
   // AtDeGAiGeoPar* par=(AtDeGAiGeoPar*)(rtdb->getContainer("AtDeGAiGeoPar"));
   LOG(INFO) << "AtDeGAi: initialisation";
}

Bool_t AtDeGAi::ProcessHits(FairVolume *vol)
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

      stack->AddPoint(kAtDeGAi);
   }
   return kTRUE;
}

void AtDeGAi::EndOfEvent()
{
   Print();
   Reset();
}

void AtDeGAi::Register()
{

   /** This will create a branch in the output tree called
       AtDeGAiPoint, setting the last parameter to kFALSE means:
       this collection will not be written to the file, it will exist
       only during the simulation.
   */

   FairRootManager::Instance()->Register("AtMCPoint", "AtDeGAi", fAtDeGAiPointCollection, kTRUE);
}

TClonesArray *AtDeGAi::GetCollection(Int_t iColl) const
{
   if (iColl == 0) {
      return fAtDeGAiPointCollection;
   } else {
      return nullptr;
   }
}

void AtDeGAi::Reset()
{
   fAtDeGAiPointCollection->Clear();
}

void AtDeGAi::Print(Option_t *option) const
{
   Int_t nHits = fAtDeGAiPointCollection->GetEntriesFast();
   LOG(INFO) << "DeGAi: " << nHits << " points registered in this event";
}

void AtDeGAi::ConstructGeometry()
{
   TString fileName = GetGeometryFileName();
   if (fileName.EndsWith(".geo")) {
      LOG(INFO) << "Constructing DeGAi geometry from ASCII file " << fileName;
      // ConstructASCIIGeometry();
   } else if (fileName.EndsWith(".root")) {
      LOG(INFO) << "Constructing DeGAi geometry from ROOT file " << fileName;
      ConstructRootGeometry();
   } else {
      std::cout << "Geometry format not supported." << std::endl;
   }
}

Bool_t AtDeGAi::CheckIfSensitive(std::string name)
{
   TString tsname = name;
   if (tsname.Contains("Crystal_")) {
      LOG(INFO) << " DeGAi geometry: Sensitive volume found: " << tsname;
      return kTRUE;
   }
   return kFALSE;
}

// -----   Private method AddPoint   --------------------------------------------
AtMCPoint *AtDeGAi::AddPoint(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Double_t time, Double_t length,
                             Double_t eLoss)
{
   TClonesArray &clref = *fAtDeGAiPointCollection;
   Int_t size = clref.GetEntriesFast();
   if (fVerboseLevel > 1)
      LOG(INFO) << "DeGAi: Adding Point in detector " << detID << ", track " << trackID << ", energy loss "
                << eLoss * 1e06 << " keV";

   auto point = new (clref[size]) AtMCPoint(trackID, detID, pos, mom, time, length, eLoss);
   point->SetVolName(fVolName);
   point->SetDetCopyID(fDetCopyID);
   return point;
}

ClassImp(AtDeGAi)
