/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             * 
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *  
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtTpc.h"

#include "AtTpcPoint.h"
#include "AtTpcGeo.h"
#include "AtTpcGeoPar.h"

#include "FairVolume.h"
#include "FairGeoVolume.h"
#include "FairGeoNode.h"
#include "FairRootManager.h"
#include "FairGeoLoader.h"
#include "FairGeoInterface.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"
#include "AtDetectorList.h"
#include "AtStack.h"

#include "TClonesArray.h"
#include "TVirtualMC.h"
#include "TGeoManager.h"
#include "TGeoBBox.h"
#include "TGeoCompositeShape.h"
#include "TGeoTube.h"
#include "TGeoMaterial.h"
#include "TGeoMedium.h"

#include <iostream>
using std::cout;
using std::endl;

AtTpc::AtTpc()
  : FairDetector("AtTpc", kTRUE, kAtTpc),
    fTrackID(-1),
    fVolumeID(-1),
    fPos(),
    fMom(),
    fTime(-1.),
    fLength(-1.),
    fELoss(-1),
    fAtTpcPointCollection(new TClonesArray("AtTpcPoint"))
{
}

AtTpc::AtTpc(const char* name, Bool_t active)
  : FairDetector(name, active, kAtTpc),
    fTrackID(-1),
    fVolumeID(-1),
    fPos(),
    fMom(),
    fTime(-1.),
    fLength(-1.),
    fELoss(-1),
    fAtTpcPointCollection(new TClonesArray("AtTpcPoint"))
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
  FairRuntimeDb* rtdb= FairRun::Instance()->GetRuntimeDb();
  AtTpcGeoPar* par=(AtTpcGeoPar*)(rtdb->getContainer("AtTpcGeoPar"));
}

Bool_t  AtTpc::ProcessHits(FairVolume* vol)
{
  /** This method is called from the MC stepping */

/*
  //Set parameters at entrance of volume. Reset ELoss.
   if ( gMC->IsTrackEntering() ) {
    fELoss  = 0.;
    fTime   = gMC->TrackTime() * 1.0e09;
    fLength = gMC->TrackLength();
    gMC->TrackPosition(fPos);
    gMC->TrackMomentum(fMom);
   }

  // Sum energy loss for all steps in the active volume
  fELoss += gMC->Edep();

  // Create AtTpcPoint at exit of active volume
  if ( gMC->IsTrackExiting()    ||
       gMC->IsTrackStop()       ||
       gMC->IsTrackDisappeared()   ) {
    fTrackID  = gMC->GetStack()->GetCurrentTrackNumber();
    fVolumeID = vol->getMCid();
    if (fELoss == 0. ) { return kFALSE; }
    AddHit(fTrackID, fVolumeID, TVector3(fPos.X(),  fPos.Y(),  fPos.Z()),
           TVector3(fMom.Px(), fMom.Py(), fMom.Pz()), fTime, fLength,
           fELoss);

    // Increment number of AtTpc det points in TParticle
    AtStack* stack = (AtStack*) gMC->GetStack();
    stack->AddPoint(kAtTpc);
  }*/


     //Set parameters at entrance of volume. Reset ELoss.
//  if ( gMC->IsTrackEntering() ) {


//=========================================================================//
 /*  if (!(gMC -> IsTrackInside()))
      return kFALSE;

    fELoss = gMC -> Edep();

    if (fELoss == 0)
      return kFALSE;

//    fELoss  = 0;
    fTime   = gMC->TrackTime() * 1.0e09;
    fLength = gMC->TrackLength();
    gMC->TrackPosition(fPos);
    gMC->TrackMomentum(fMom);
//  }

  // Sum energy loss for all steps in the active volume
//  fELoss += gMC->Edep();

  // Create STMCPoint at exit of active volume
//  if ( gMC->IsTrackExiting()    ||
//       gMC->IsTrackStop()       ||
//       gMC->IsTrackDisappeared()   ) {
    fTrackID  = gMC->GetStack()->GetCurrentTrackNumber();
    fVolumeID = vol->getMCid();
//    if (fELoss == 0. ) { return kFALSE; }
    AddHit(fTrackID, fVolumeID, TVector3(fPos.X(),  fPos.Y(),  fPos.Z()),
           TVector3(fMom.Px(), fMom.Py(), fMom.Pz()), fTime, fLength,
           fELoss);

     // Increment number of AtTpc det points in TParticle
    AtStack* stack = (AtStack*) gMC->GetStack();
    stack->AddPoint(kAtTpc);
    Print();
   

  return kTRUE;*/
//============================================================================//

   if (gMC->IsTrackEntering())
    {
        fELoss = 0.;
        fTime = gMC->TrackTime() * 1.0e09;
        fLength = gMC->TrackLength();
        gMC->TrackPosition(fPosIn);
        gMC->TrackMomentum(fMomIn);
       // LOG(INFO) << "ATTPC: Position of the first hit" << FairLogger::endl;
    }

    // Sum energy loss for all steps in the active volume
    fELoss += gMC->Edep();
    fTime = gMC->TrackTime() * 1.0e09;
    fLength = gMC->TrackLength();
    gMC->TrackPosition(fPosIn);
    gMC->TrackMomentum(fMomIn);
    fTrackID  = gMC->GetStack()->GetCurrentTrackNumber();
    fVolumeID = vol->getMCid();

    // Set additional parameters at exit of active volume. Create R3BTraPoint.
    if (gMC->IsTrackExiting() || gMC->IsTrackStop() || gMC->IsTrackDisappeared())
      {
	
        fTrackID = gMC->GetStack()->GetCurrentTrackNumber();
        fVolumeID = vol->getMCid();
        fDetCopyID = vol->getCopyNo(); 
        gMC->TrackPosition(fPosOut);
        gMC->TrackMomentum(fMomOut);
//        if (fELoss == 0.)
//            return kFALSE;

      if (gMC->IsTrackExiting())
        {
		//LOG(INFO) << "ATTPC: Position of the first hit" << FairLogger::endl;
            const Double_t* oldpos;
            const Double_t* olddirection;
            Double_t newpos[3];
            Double_t newdirection[3];
            Double_t safety;

            gGeoManager->FindNode(fPosOut.X(), fPosOut.Y(), fPosOut.Z());
            oldpos = gGeoManager->GetCurrentPoint();
            olddirection = gGeoManager->GetCurrentDirection();

            for (Int_t i = 0; i < 3; i++)
            {
                newdirection[i] = -1 * olddirection[i];
            }

            gGeoManager->SetCurrentDirection(newdirection);
            //   TGeoNode *bla = gGeoManager->FindNextBoundary(2);
            safety = gGeoManager->GetSafeDistance();

            gGeoManager->SetCurrentDirection(-newdirection[0], -newdirection[1], -newdirection[2]);

            for (Int_t i = 0; i < 3; i++)
            {
                newpos[i] = oldpos[i] - (3 * safety * olddirection[i]);
            }

            fPosOut.SetX(newpos[0]);
            fPosOut.SetY(newpos[1]);
            fPosOut.SetZ(newpos[2]);
         }

	}// if track 

		AddHit(fTrackID,
		       fVolumeID,
		       fDetCopyID, 
		       TVector3(fPosIn.X(), fPosIn.Y(), fPosIn.Z()),
		       TVector3(fPosOut.X(), fPosOut.Y(), fPosOut.Z()),
		       TVector3(fMomIn.Px(), fMomIn.Py(), fMomIn.Pz()),
		       TVector3(fMomOut.Px(), fMomOut.Py(), fMomOut.Pz()),
		       fTime,
		       fLength,
		       fELoss);

		 

		

		// Increment number of AtTpc det points in TParticle
	    	AtStack* stack = (AtStack*) gMC->GetStack();
	    	stack->AddPoint(kAtTpc);

		//Print();

       // ResetParameters();
         // Reset();
    

    return kTRUE;

}

void AtTpc::EndOfEvent()
{

  fAtTpcPointCollection->Clear();

}



void AtTpc::Register()
{

  /** This will create a branch in the output tree called
      AtTpcPoint, setting the last parameter to kFALSE means:
      this collection will not be written to the file, it will exist
      only during the simulation.
  */

  FairRootManager::Instance()->Register("AtTpcPoint", "AtTpc",
                                        fAtTpcPointCollection, kTRUE);

}


TClonesArray* AtTpc::GetCollection(Int_t iColl) const
{
  if (iColl == 0) { return fAtTpcPointCollection; }
  else { return NULL; }
}

void AtTpc::Reset()
{
  fAtTpcPointCollection->Clear();
}

void AtTpc::Print(Option_t* option) const
{
    Int_t nHits = fAtTpcPointCollection->GetEntriesFast();
    LOG(INFO) << "ATTPC: " << nHits << " points registered in this event" << FairLogger::endl;
}

/*void AtTpc::ConstructGeometry()
{
    
    TGeoVolume *top=gGeoManager->GetTopVolume();
    TGeoMedium *Si =gGeoManager->GetMedium("Si");
    TGeoMedium *Carbon = gGeoManager->GetMedium("Carbon");
    
    if(Si==0){
        TGeoMaterial *matSi     = new TGeoMaterial("Si", 28.0855, 14, 2.33);
        Si     = new TGeoMedium("Si", 2, matSi);
    }
    if(Carbon==0){
        TGeoMaterial *matCarbon    = new TGeoMaterial("C", 12.011, 6.0, 2.265);
        Carbon     = new TGeoMedium("C", 3, matCarbon);
    }

    
    TGeoVolume *det1= gGeoManager->MakeTubs("Det1",Si,5,80,0.1,0,360);
    AddSensitiveVolume(det1);
    TGeoRotation r1;
    r1.SetAngles(0,0,0);
    TGeoTranslation t1(0, 0, 0);
    TGeoCombiTrans c1(t1, r1);
    TGeoHMatrix *h1 = new TGeoHMatrix(c1);
    top->AddNode(det1,1,h1);
    det1->SetLineColor(kGreen);
    
    TGeoVolume *passive1= gGeoManager->MakeTubs("Pass1",Si,5,120,10,0,360);
    TGeoRotation rp1;
    rp1.SetAngles(0,0,0);
    TGeoTranslation tp1(0, 0, 20);
    TGeoCombiTrans cp1(tp1, rp1);
    TGeoHMatrix *hp1 = new TGeoHMatrix(cp1);
    top->AddNode(passive1,1,hp1);
    passive1->SetLineColor(kBlue);
    
    
    
    TGeoVolume *det2= gGeoManager->MakeTubs("Det2",Si,5,150,0.1,0,360);
    AddSensitiveVolume(det2);
    TGeoRotation r2;
    r2.SetAngles(0,0,0);
    TGeoTranslation t2(0, 0, 70);
    TGeoCombiTrans c2(t2, r2);
    TGeoHMatrix *h2 = new TGeoHMatrix(c2);
    top->AddNode(det2,1,h2);
    det2->SetLineColor(kGreen);
    
    TGeoVolume *det3= gGeoManager->MakeTubs("Det3",Si,5,150,0.1,0,360);
    AddSensitiveVolume(det3);
    TGeoRotation r3;
    r3.SetAngles(0,0,0);
    TGeoTranslation t3(0, 0, 150);
    TGeoCombiTrans c3(t3, r3);
    TGeoHMatrix *h3 = new TGeoHMatrix(c3);
    top->AddNode(det3,1,h3);
    det3->SetLineColor(kGreen);
    
    
}*/

void AtTpc::ConstructGeometry()
{
  TString fileName=GetGeometryFileName();
  if (fileName.EndsWith(".geo")) {
    LOG(INFO)<<"Constructing ATTPC geometry from ASCII file "<<fileName<<FairLogger::endl;
    //ConstructASCIIGeometry();
  } else if (fileName.EndsWith(".root")) {
    LOG(INFO)<<"Constructing ATTPC geometry from ROOT file "<<fileName<<FairLogger::endl;
    ConstructRootGeometry();
  } else {
    std::cout << "Geometry format not supported." << std::endl;
  }
}

Bool_t AtTpc::CheckIfSensitive(std::string name)
{
  
  TString tsname = name;
  if (tsname.Contains("drift_volume")) {
    LOG(INFO)<<" ATTPC geometry: Sensitive volume found: "<<tsname<<FairLogger::endl;
    return kTRUE;
  }
  return kFALSE;
}

AtTpcPoint* AtTpc::AddHit(Int_t trackID, Int_t detID,
                                      TVector3 pos, TVector3 mom,
                                      Double_t time, Double_t length,
                                      Double_t eLoss)
{
  TClonesArray& clref = *fAtTpcPointCollection;
  Int_t size = clref.GetEntriesFast();
  return new(clref[size]) AtTpcPoint(trackID, detID, pos, mom,
         time, length, eLoss);
}

// -----   Private method AddHit   --------------------------------------------
AtTpcPoint* AtTpc::AddHit(Int_t trackID,
                            Int_t detID,
                            Int_t detCopyID, // added by Marc
                            TVector3 posIn,
                            TVector3 posOut,
                            TVector3 momIn,
                            TVector3 momOut,
                            Double_t time,
                            Double_t length,
                            Double_t eLoss)
{
    TClonesArray& clref = *fAtTpcPointCollection;
    Int_t size = clref.GetEntriesFast();
    if (fVerboseLevel > 1)
        LOG(INFO) << "ATTPC: Adding Point at (" << posIn.X() << ", " << posIn.Y() << ", " << posIn.Z() << ") cm,  detector " << detID << ", track " << trackID
                  << ", energy loss " << eLoss * 1e06 << " keV" << FairLogger::endl;
    return new (clref[size]) AtTpcPoint(trackID,
                                         detID,
                                         detCopyID,
                                         posIn,
                                         posOut, // detCopyID added by Marc
                                         momIn,
                                         momOut,
                                         time,
                                         length,
                                         eLoss);
}

ClassImp(AtTpc)
