/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtApollo.h"

#include "AtApolloPoint.h"
#include "AtDetectorList.h"
#include "AtStack.h"

#include <FairDetector.h>
#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairVolume.h>

#include <TClonesArray.h>
#include <TLorentzVector.h>
#include <TVector3.h>
#include <TVirtualMC.h>
#include <TVirtualMCStack.h>

#include <iostream>

using std::cout;
using std::endl;

AtApollo::AtApollo()
   : FairDetector("AtApollo", kTRUE, kAtApollo), fTrackID(-1), fVolumeID(-1), fDetCopyID(-1), fTime(-1.), fLength(-1.),
     fELoss(-1), fAtApolloPointCollection(new TClonesArray("AtApolloPoint")), fELossAcc(-1)
{
}

AtApollo::AtApollo(const char *name, Bool_t active)
   : FairDetector(name, active, kAtApollo), fTrackID(-1), fVolumeID(-1), fDetCopyID(-1), fTime(-1.), fLength(-1.),
     fELoss(-1), fAtApolloPointCollection(new TClonesArray("AtApolloPoint")), fELossAcc(-1)
{
}

AtApollo::~AtApollo()
{
   if (fAtApolloPointCollection) {
      fAtApolloPointCollection->Delete();
      delete fAtApolloPointCollection;
   }
}

void AtApollo::Initialize()
{
   FairDetector::Initialize();
   // FairRuntimeDb *rtdb = FairRun::Instance()->GetRuntimeDb();
   //  AtApolloGeoPar* par=(AtApolloGeoPar*)(rtdb->getContainer("AtApolloGeoPar"));
   LOG(info) << "AtApollo: initialisation";
}

Bool_t AtApollo::ProcessHits(FairVolume *vol)
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
               TVector3(fMomIn.X(), fMomIn.Y(), fMomIn.Z()), fDetCopyID, fTime, fLength, fELossAcc);

      stack->AddPoint(kAtApollo);
   }
   return kTRUE;
}

void AtApollo::EndOfEvent()
{
   Print();
   Reset();
}

void AtApollo::Register()
{

   /** This will create a branch in the output tree called
       AtApolloPoint, setting the last parameter to kFALSE means:
       this collection will not be written to the file, it will exist
       only during the simulation.
   */

   FairRootManager::Instance()->Register("AtApolloPoint", "AtApollo", fAtApolloPointCollection, kTRUE);
}

TClonesArray *AtApollo::GetCollection(Int_t iColl) const
{
   if (iColl == 0) {
      return fAtApolloPointCollection;
   } else {
      return nullptr;
   }
}

void AtApollo::Reset()
{
   fAtApolloPointCollection->Clear();
}

void AtApollo::Print(Option_t *option) const
{
   Int_t nHits = fAtApolloPointCollection->GetEntriesFast();
   LOG(info) << "APOLLO: " << nHits << " points registered in this event";
}

void AtApollo::ConstructGeometry()
{
   TString fileName = GetGeometryFileName();
   if (fileName.EndsWith(".geo")) {
      LOG(info) << "Constructing APOLLO geometry from ASCII file " << fileName;
      // ConstructASCIIGeometry();
   } else if (fileName.EndsWith(".root")) {
      LOG(info) << "Constructing APOLLO geometry from ROOT file " << fileName;
      ConstructRootGeometry();
   } else {
      std::cout << "Geometry format not supported." << std::endl;
   }
}

Bool_t AtApollo::CheckIfSensitive(std::string name)
{
   TString tsname = name;
   if (tsname.Contains("Crystal_")) {
      LOG(info) << " APOLLO geometry: Sensitive volume found: " << tsname;
      return kTRUE;
   }
   return kFALSE;
}

// -----   Private method AddPoint   --------------------------------------------
AtApolloPoint *AtApollo::AddPoint(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Int_t crystalID,
                                  Double_t time, Double_t length, Double_t eLoss)
{
   TClonesArray &clref = *fAtApolloPointCollection;
   Int_t size = clref.GetEntriesFast();
   if (fVerboseLevel > 1)
      LOG(info) << "APOLLO: Adding Point in detector " << detID << ", track " << trackID << ", energy loss "
                << eLoss * 1e06 << " keV";

   return new (clref[size]) AtApolloPoint(trackID, detID, pos, mom, crystalID, time, length, eLoss);
}

ClassImp(AtApollo)
