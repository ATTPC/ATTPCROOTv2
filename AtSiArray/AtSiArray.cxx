#include "AtSiArray.h"

#include <FairDetector.h>
#include <TLorentzVector.h>
#include <TVector3.h>
#include <TVirtualMCStack.h>
#include <stddef.h>
#include <iostream>

#include "AtSiArrayGeoPar.h"
#include "AtSiPoint.h"
#include "AtDetectorList.h"
#include "AtStack.h"
#include <FairVolume.h>
#include <FairRootManager.h>
#include <FairRun.h>
#include <FairRuntimeDb.h>
#include <TClonesArray.h>
#include <TVirtualMC.h>
#include <TGeoManager.h>
#include "fairlogger/Logger.h"

using std::cout;
using std::endl;

AtSiArray::AtSiArray()
   : FairDetector("AtSiArray", kTRUE, kAtSiArray), fTrackID(-1), fVolumeID(-1), fPos(), fMom(), fTime(-1.),
     fLength(-1.), fELoss(-1), fPosIndex(-1), fAtSiArrayPointCollection(new TClonesArray("AtSiPoint")), fELossAcc(-1)
{
   // LOG(INFO)<<" AtSiArray detector initialized ";
}

AtSiArray::AtSiArray(const char *name, Bool_t active)
   : FairDetector(name, active, kAtSiArray), fTrackID(-1), fVolumeID(-1), fPos(), fMom(), fTime(-1.), fLength(-1.),
     fELoss(-1), fPosIndex(-1), fAtSiArrayPointCollection(new TClonesArray("AtSiPoint")), fELossAcc(-1)
{
   // LOG(INFO)<<" AtSiArray detector initialized ";
}

AtSiArray::~AtSiArray()
{
   if (fAtSiArrayPointCollection) {
      fAtSiArrayPointCollection->Delete();
      delete fAtSiArrayPointCollection;
   }
}

void AtSiArray::Initialize()
{
   FairDetector::Initialize();
   FairRuntimeDb *rtdb = FairRun::Instance()->GetRuntimeDb();
   AtSiArrayGeoPar *par = (AtSiArrayGeoPar *)(rtdb->getContainer("AtSiArrayGeoPar"));
}

Bool_t AtSiArray::ProcessHits(FairVolume *vol)
{
   /** This method is called from the MC stepping */

   AtStack *stack = static_cast<AtStack *>(TVirtualMC::GetMC()->GetStack());
   std::pair<Int_t, Int_t> AZ;
   AZ = DecodePdG(gMC->TrackPid());
   fVolName = gMC->CurrentVolName();
   Int_t VolumeID;

   LOG(debug) << "In AtSiArray::ProcessHits";
   // Set parameters at entrance of volume. Reset ELoss.
   if (TVirtualMC::GetMC()->IsTrackEntering()) {

      fELoss = 0.;
      fTime = TVirtualMC::GetMC()->TrackTime() * 1.0e09;
      fLength = TVirtualMC::GetMC()->TrackLength();
      TVirtualMC::GetMC()->TrackPosition(fPosIn);
      TVirtualMC::GetMC()->TrackMomentum(fMomIn);

      std::cout << " AtSiArray: Track is entering "
                << "\n";
      LOG(INFO) << " HELIOS: First hit in Volume " << fVolName;
      LOG(INFO) << " Particle : " << gMC->ParticleName(gMC->TrackPid());
      LOG(INFO) << " PID PdG : " << gMC->TrackPid();
      LOG(INFO) << " Atomic Mass : " << AZ.first;
      LOG(INFO) << " Atomic Number : " << AZ.second;
      LOG(INFO) << " Volume ID " << gMC->CurrentVolID(VolumeID);
      LOG(INFO) << " Track ID : " << fTrackID;
      LOG(INFO) << " Position In : " << fPosIn.X() << " " << fPosIn.Y() << "  " << fPosIn.Z() << std::endl;
      LOG(INFO) << " Position Out : " << fPosOut.X() << " " << fPosOut.Y() << "  " << fPosOut.Z() << std::endl;
      LOG(INFO) << " Momentum In: " << fMomIn.X() << " " << fMomIn.Y() << "  " << fMomIn.Z() << std::endl;
      // LOG(INFO)<<" Total relativistic energy " <<gMC->Etot()<< FairLogger::endl;
      // LOG(INFO)<<" Mass of the Tracked particle (gAVTP) : "<<gAtVP->GetBeamMass()<<std::endl;
      // LOG(INFO)<<" Mass of the Tracked particle (gMC) : "<<gMC->TrackMass()<<std::endl;
      // LOG(INFO)<<" Initial energy of the current particle in this volume : "<<((gMC->Etot() - gMC->TrackMass()) *
      // 1000.)<<FairLogger::endl;// Relativistic Mass
   }

   // Sum energy loss for all steps in the active volume
   fELoss = TVirtualMC::GetMC()->Edep();
   // std::cout<<" Energy loss : "<<fELoss*1000<<"\n";
   gMC->TrackPosition(fPosIn);
   gMC->TrackMomentum(fMomIn);
   fTrackID = gMC->GetStack()->GetCurrentTrackNumber();
   fVolumeID = vol->getMCid();
   fTime = gMC->TrackTime() * 1.0e09;
   fLength = gMC->TrackLength();

   // Create AtSiArrayPoint at exit of active volume
   if (TVirtualMC::GetMC()->IsTrackExiting() || TVirtualMC::GetMC()->IsTrackStop() ||
       TVirtualMC::GetMC()->IsTrackDisappeared()) {
      fTrackID = TVirtualMC::GetMC()->GetStack()->GetCurrentTrackNumber();
      fVolumeID = vol->getMCid();
      TVirtualMC::GetMC()->TrackPosition(fPosOut);
      TVirtualMC::GetMC()->TrackMomentum(fMomOut);

      /*if (fELoss == 0.) {
          return kFALSE;
      }*/

      if (gMC->IsTrackExiting()) {

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
         // TGeoNode *bla = gGeoManager->FindNextBoundary(2);
         safety = gGeoManager->GetSafeDistance();

         gGeoManager->SetCurrentDirection(-newdirection[0], -newdirection[1], -newdirection[2]);

         for (Int_t i = 0; i < 3; i++) {
            newpos[i] = oldpos[i] - (3 * safety * olddirection[i]);
         }

         fPosOut.SetX(newpos[0]);
         fPosOut.SetY(newpos[1]);
         fPosOut.SetZ(newpos[2]);

         std::cout << " AtSiArray: Track is exiting "
                   << "\n";
         LOG(INFO) << " HELIOS: First hit in Volume " << fVolName;
         LOG(INFO) << " Particle : " << gMC->ParticleName(gMC->TrackPid());
         LOG(INFO) << " PID PdG : " << gMC->TrackPid();
         LOG(INFO) << " Atomic Mass : " << AZ.first;
         LOG(INFO) << " Atomic Number : " << AZ.second;
         LOG(INFO) << " Volume ID " << gMC->CurrentVolID(VolumeID);
         LOG(INFO) << " Track ID : " << fTrackID;
         LOG(INFO) << " Position In : " << fPosIn.X() << " " << fPosIn.Y() << "  " << fPosIn.Z() << std::endl;
         LOG(INFO) << " Position Out : " << fPosOut.X() << " " << fPosOut.Y() << "  " << fPosOut.Z() << std::endl;
         LOG(INFO) << " Momentum In: " << fMomIn.X() << " " << fMomIn.Y() << "  " << fMomIn.Z() << std::endl;
         LOG(INFO) << " Momentum Out: " << fMomOut.X() << " " << fMomOut.Y() << "  " << fMomOut.Z() << std::endl;
      }

      /* AddHit(fTrackID,
              fVolumeID,
              fVolName,
              fDetCopyID,
              TVector3(fPosIn.X(), fPosIn.Y(), fPosIn.Z()),
              TVector3(fPosOut.X(), fPosOut.Y(), fPosOut.Z()),
              TVector3(fMomIn.Px(), fMomIn.Py(), fMomIn.Pz()),
              TVector3(fMomOut.Px(), fMomOut.Py(), fMomOut.Pz()),
              fTime,
              fLength,
              fELoss,
              0.0,
              0.0,
              AZ.first,
              AZ.second);




       AtStack* stack = static_cast<AtStack*>(TVirtualMC::GetMC()->GetStack());
       stack->AddPoint(kAtSiArray);*/
   }

   AddHit(fTrackID, fVolumeID, fVolName, fDetCopyID, TVector3(fPosIn.X(), fPosIn.Y(), fPosIn.Z()),
          TVector3(fPosOut.X(), fPosOut.Y(), fPosOut.Z()), TVector3(fMomIn.Px(), fMomIn.Py(), fMomIn.Pz()),
          TVector3(fMomOut.Px(), fMomOut.Py(), fMomOut.Pz()), fTime, fLength, fELoss, 0.0, 0.0, AZ.first, AZ.second);

   stack->AddPoint(kAtSiArray);

   return kTRUE;
}

void AtSiArray::EndOfEvent()
{

   fAtSiArrayPointCollection->Clear();
}

void AtSiArray::Register()
{

   /** This will create a branch in the output tree called
       AtTpcPoint, setting the last parameter to kFALSE means:
       this collection will not be written to the file, it will exist
       only during the simulation.
   */

   FairRootManager::Instance()->Register("AtSiArrayPoint", "AtSiArray", fAtSiArrayPointCollection, kTRUE);
}

TClonesArray *AtSiArray::GetCollection(Int_t iColl) const
{
   if (iColl == 0) {
      return fAtSiArrayPointCollection;
   } else {
      return NULL;
   }
}

void AtSiArray::Reset()
{
   fAtSiArrayPointCollection->Clear();
}

void AtSiArray::Print(Option_t *option) const
{
   Int_t nHits = fAtSiArrayPointCollection->GetEntriesFast();
   LOG(INFO) << "Si Array: " << nHits << " points registered in this event";
}

void AtSiArray::ConstructGeometry()
{
   TString fileName = GetGeometryFileName();
   if (fileName.EndsWith(".geo")) {
      LOG(INFO) << "Constructing Si Array geometry from ASCII file " << fileName;
      // ConstructASCIIGeometry();
   } else if (fileName.EndsWith(".root")) {
      LOG(INFO) << "Constructing Si Array geometry from ROOT file " << fileName;
      ConstructRootGeometry();
   } else {
      std::cout << "Geometry format not supported." << std::endl;
   }
}

Bool_t AtSiArray::CheckIfSensitive(std::string name)
{

   TString tsname = name;
   if (tsname.Contains("silicon")) {
      LOG(INFO) << " Si Array geometry: Sensitive volume found: " << tsname;
      return kTRUE;
   }
   return kFALSE;
}

AtSiPoint *AtSiArray::AddHit(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Double_t time, Double_t length,
                             Double_t eLoss)
{
   TClonesArray &clref = *fAtSiArrayPointCollection;
   Int_t size = clref.GetEntriesFast();
   return new (clref[size]) AtSiPoint(trackID, detID, pos, mom, time, length, eLoss);
}

// -----   Private method AddHit   --------------------------------------------
AtSiPoint *AtSiArray::AddHit(Int_t trackID, Int_t detID, TString VolName, Int_t detCopyID, TVector3 posIn,
                             TVector3 posOut, TVector3 momIn, TVector3 momOut, Double_t time, Double_t length,
                             Double_t eLoss, Double_t EIni, Double_t AIni, Int_t A, Int_t Z)
{
   TClonesArray &clref = *fAtSiArrayPointCollection;
   Int_t size = clref.GetEntriesFast();
   // std::cout<< "AtTPC: Adding Point at (" << posIn.X() << ", " << posIn.Y() << ", " << posIn.Z() << ") cm,  detector
   // " << detID << ", track " << trackID
   //<< ", energy loss " << eLoss << " MeV" <<" with accumulated Energy Loss : "<<fELossAcc<<" MeV "<< std::endl;
   if (fVerboseLevel > 1)
      LOG(INFO) << "Si Array: Adding Point at (" << posIn.X() << ", " << posIn.Y() << ", " << posIn.Z()
                << ") cm,  detector " << detID << ", track " << trackID << ", energy loss " << eLoss * 1e06 << " keV";

   return new (clref[size]) AtSiPoint(trackID, detID, posIn, posOut, momIn, momOut, time, length, eLoss, VolName,
                                      detCopyID, EIni, AIni, A, Z);
}

std::pair<Int_t, Int_t> AtSiArray::DecodePdG(Int_t PdG_Code)
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

ClassImp(AtSiArray);
