/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtSeGA.h"

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

AtSeGA::AtSeGA()
   : FairDetector("AtSeGA", kTRUE, kAtSeGA), fTrackID(-1), fVolumeID(-1), fDetCopyID(-1), fTime(-1.), fLength(-1.),
     fELoss(-1), fAtSeGAPointCollection(new TClonesArray("AtMCPoint")), fELossAcc(-1)
{
}

AtSeGA::AtSeGA(const char *name, Bool_t active)
   : FairDetector(name, active, kAtSeGA), fTrackID(-1), fVolumeID(-1), fDetCopyID(-1), fTime(-1.), fLength(-1.),
     fELoss(-1), fAtSeGAPointCollection(new TClonesArray("AtMCPoint")), fELossAcc(-1)
{
}

AtSeGA::~AtSeGA()
{
   if (fAtSeGAPointCollection) {
      fAtSeGAPointCollection->Delete();
      delete fAtSeGAPointCollection;
   }
}

void AtSeGA::Initialize()
{
   FairDetector::Initialize();
   // AtSeGAGeoPar* par=(AtSeGAGeoPar*)(rtdb->getContainer("AtSeGAGeoPar"));
   LOG(info) << "AtSeGA: initialisation";
}

Bool_t AtSeGA::ProcessHits(FairVolume *vol)
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

      stack->AddPoint(kAtSeGA);
   }
   return kTRUE;
}

void AtSeGA::EndOfEvent()
{
   Print();
   Reset();
}

void AtSeGA::Register()
{

   /** This will create a branch in the output tree called
       AtSeGAPoint, setting the last parameter to kFALSE means:
       this collection will not be written to the file, it will exist
       only during the simulation.
   */

   FairRootManager::Instance()->Register("AtMCPoint", "AtSeGA", fAtSeGAPointCollection, kTRUE);
}

TClonesArray *AtSeGA::GetCollection(Int_t iColl) const
{
   if (iColl == 0) {
      return fAtSeGAPointCollection;
   } else {
      return nullptr;
   }
}

void AtSeGA::Reset()
{
   fAtSeGAPointCollection->Clear();
}

void AtSeGA::Print(Option_t *option) const
{
   Int_t nHits = fAtSeGAPointCollection->GetEntriesFast();
   LOG(info) << "SEGA: " << nHits << " points registered in this event";
}

void AtSeGA::ConstructGeometry()
{
   TString fileName = GetGeometryFileName();
   if (fileName.EndsWith(".geo")) {
      LOG(info) << "Constructing SEGA geometry from ASCII file " << fileName;
      // ConstructASCIIGeometry();
   } else if (fileName.EndsWith(".root")) {
      LOG(info) << "Constructing SEGA geometry from ROOT file " << fileName;
      ConstructRootGeometry();
   } else {
      std::cout << "Geometry format not supported." << std::endl;
   }
}

Bool_t AtSeGA::CheckIfSensitive(std::string name)
{
   TString tsname = name;
   if (tsname.Contains("Crystal_")) {
      LOG(info) << " SeGA geometry: Sensitive volume found: " << tsname;
      return kTRUE;
   }
   return kFALSE;
}

// -----   Private method AddPoint   --------------------------------------------
AtMCPoint *
AtSeGA::AddPoint(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Double_t time, Double_t length, Double_t eLoss)
{
   TClonesArray &clref = *fAtSeGAPointCollection;
   Int_t size = clref.GetEntriesFast();
   if (fVerboseLevel > 1)
      LOG(info) << "SEGA: Adding Point in detector " << detID << ", track " << trackID << ", energy loss "
                << eLoss * 1e06 << " keV";

   auto point = new (clref[size]) AtMCPoint(trackID, detID, pos, mom, time, length, eLoss);
   point->SetVolName(fVolName);
   point->SetDetCopyID(fDetCopyID);
   return point;
}

ClassImp(AtSeGA)
