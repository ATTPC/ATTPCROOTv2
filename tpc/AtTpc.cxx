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
#include "ATVertexPropagator.h"

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
#include "TParticle.h"
#include "TRandom.h"
#include "TRandom3.h"

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
    fPosIndex(-1),
    fAtTpcPointCollection(new TClonesArray("AtTpcPoint")),
    fELossAcc(-1)
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
    fPosIndex(-1),
    fAtTpcPointCollection(new TClonesArray("AtTpcPoint")),
    fELossAcc(-1)
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


	      AtStack* stack = (AtStack*) gMC->GetStack();
        fVolName = gMC->CurrentVolName();
        std::pair<Int_t,Int_t> AZ;


        //std::cout<<" Current Event : "<<gMC->CurrentEvent()<<std::endl;

       /* std::cout<<" Current Track Number : "<<stack->GetCurrentTrackNumber()<<std::endl;
	TParticle* beam_part0 = stack->GetParticle(stack->GetCurrentTrackNumber());
        std::cout<<" Current particle mass  "<<beam_part0->GetMass()<<std::endl;*/

    if (gMC->IsTrackEntering())
    {
         fELoss = 0.;
         fELossAcc = 0.;
         fTime = gMC->TrackTime() * 1.0e09;
         fLength = gMC->TrackLength();
         gMC->TrackPosition(fPosIn);
         gMC->TrackMomentum(fMomIn);
         fTrackID  = gMC->GetStack()->GetCurrentTrackNumber();
         if(gATVP->GetBeamEvtCnt()%2!=0 && fTrackID==0 && (fVolName=="drift_volume" || fVolName=="cell") )InPos = fPosIn; // Position of the first hit of the beam in the TPC volume ( For tracking purposes in the TPC)
         Int_t VolumeID;
         if(gATVP->GetBeamEvtCnt()%2!=0) LOG(INFO) << " ATTPC: Beam Event " <<FairLogger::endl;
         else if(gATVP->GetDecayEvtCnt()%2==0) LOG(INFO) << " ATTPC: Reaction/Decay Event " <<FairLogger::endl;
         AZ = DecodePdG(gMC->TrackPid());
         LOG(INFO) << " ATTPC: First hit in Volume " <<fVolName<< FairLogger::endl;
         LOG(INFO) << " Particle : "<<gMC->ParticleName(gMC->TrackPid())<<FairLogger::endl;
         LOG(INFO) << " PID PdG : "<<gMC->TrackPid()<<FairLogger::endl;
         LOG(INFO) << " Atomic Mass : "<<AZ.first<<FairLogger::endl;
         LOG(INFO) << " Atomic Number : "<<AZ.second<<FairLogger::endl;
         LOG(INFO)<<" Volume ID "<<gMC->CurrentVolID(VolumeID)<<FairLogger::endl;
         LOG(INFO)<<" Track ID : "<<fTrackID<<FairLogger::endl;
         LOG(INFO)<<" Total relativistic energy " <<gMC->Etot()<< FairLogger::endl;
         LOG(INFO)<<" Mass of the Tracked particle (gAVTP) : "<<gATVP->GetBeamMass()<<std::endl;
         LOG(INFO)<<" Mass of the Tracked particle (gMC) : "<<gMC->TrackMass()<<std::endl;
         LOG(INFO)<<" Initial energy of the current particle in this volume : "<<((gMC->Etot() - gMC->TrackMass()) * 1000.)<<FairLogger::endl;// Relativistic Mass
         //LOG(INFO)<<" Total energy of the current track (gMC) : "<<((gMC->Etot() - gMC->TrackMass()) * 1000.)<<FairLogger::endl;// Relativistic Mass

              /*
               std::cout<<" Recoil Energy : "<<gATVP->GetRecoilE()<<std::endl;
      	 std::cout<<" Scattered Energy : "<<gATVP->GetScatterE()<<std::endl;
               std::cout<<" Recoil Angle : "<<gATVP->GetRecoilA()<<std::endl;
      	 std::cout<<" Scattered Angle : "<<gATVP->GetScatterA()<<std::endl;*/
    }

      //
      fELoss = gMC->Edep();
      fELossAcc+= fELoss;
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

            if( (fVolName=="drift_volume" || fVolName=="cell") && gATVP->GetBeamEvtCnt()%2!=0 && fTrackID==0 ){
		            gATVP->ResetVertex(); 
                LOG(INFO)<<" - AtTpc Warning : Beam punched through the ATTPC. Reseting Vertex! "<<std::endl;
		        }

         }

	}// if track



              	if( gATVP->GetBeamEvtCnt()%2!=0 && fTrackID==0) { // We assume that the beam-like particle is fTrackID==0 since it is the first one added
              														//  in the Primary Generator

                             // std::cout<<" Current Decay particle count : "<<gATVP->GetDecayEvtCnt()<<std::endl;
              		//std::cout<<" Current Beam particle count :  "<<gATVP->GetBeamEvtCnt()<<std::endl;
                             // std::cout<<" fTrackID : "<<fTrackID<<std::endl;
              		//std::cout<<" Recoil Energy : "<<gATVP->GetRecoilE()<<std::endl;

              		AddHit(fTrackID,
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

              	}
              	else if(gATVP->GetDecayEvtCnt()%2==0 && fTrackID==1)
              	{

               		AddHit(fTrackID,
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
              		       gATVP->GetScatterE(),
              		       gATVP->GetScatterA(),
                         AZ.first,
                         AZ.second);




              	}else if(gATVP->GetDecayEvtCnt()%2==0 && fTrackID==2)
              	{

               		AddHit(fTrackID,
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
                         gATVP->GetRecoilE(),
              		       gATVP->GetRecoilA(),
                         AZ.first,
                         AZ.second);



              	}
                else if(gATVP->GetDecayEvtCnt()%2==0 && fTrackID==3)
                {

                      AddHit(fTrackID,
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
                     gATVP->GetBURes2E(),
                     gATVP->GetBURes2A(),
                     AZ.first,
                     AZ.second);



                }
                else if(gATVP->GetDecayEvtCnt()%2==0 && fTrackID==4)
                {
                      AddHit(fTrackID,
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
                     0,
                     0,
                     AZ.first,
                     AZ.second);
                }
              else if(gATVP->GetDecayEvtCnt()%2==0 && fTrackID==5)
                {
                      AddHit(fTrackID,
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
                     0,
                     0,
                     AZ.first,
                     AZ.second);
                }

	//std::cout<<" Energy Loss : "<<fELoss*1000<<std::endl;

         //LOG(INFO)<<" Total Energy of the tracked particle : "<<gMC->Etot()<<std::endl;
         //LOG(INFO)<<" Mass of the tracked particle : "<<gMC->TrackMass()<<std::endl;
         //LOG(INFO)<<" Mass of the Beam from global vertex pointer : "<<gATVP->GetBeamMass()<<std::endl;
        // LOG(INFO)<<" Total energy of the current track : "<<((gMC->Etot() - gMC->TrackMass()) * 1000.)<<FairLogger::endl;
         //LOG(INFO)<<" Total energy of the current track : "<<((gMC->Etot() - gATVP->GetBeamMass()) * 1000.)<<FairLogger::endl;
         //std::cout<<" Track ID : "<<fTrackID<<std::endl;
         //std::cout<<" Energy Loss : "<<fELoss*1000<<std::endl;


        /*  TRandom3 r;
	  Double_t Er = r.Rndm();
          std::cout<<" Random Energy : "<<Er<<std::endl;*/

	  /*TRandom* r = gMC->GetRandom();
          Double_t Er = r->Rndm();
	  Double_t ErBeam = Er*35.0;*/
          //std::cout<<" Random Energy in AtTpc  : "<<gATVP->GetRndELoss()<<std::endl;

		//Double_t Er = gRandom->Uniform(0.,gATVP->GetBeamNomE());
  		//std::cout<<" Nominal energy of the beam : "<<gATVP->GetBeamNomE()<<std::endl;
                //std::cout<<" Random Stopping Energy  : "<<Er<<std::endl;




        	if(fELossAcc*1000>gATVP->GetRndELoss()  &&   (gATVP->GetBeamEvtCnt()%2!=0 && fTrackID==0) && (fVolName=="drift_volume" || fVolName=="cell")){
        	       LOG(INFO)<<" Beam energy loss before reaction : "<<fELossAcc*1000<<FairLogger::endl;
        	       gMC->StopTrack();
                 gATVP->ResetVertex();
                 TLorentzVector StopPos;
                 TLorentzVector StopMom;
                 gMC->TrackPosition(StopPos);
        	       gMC->TrackMomentum(StopMom);
                 LOG(INFO)<<" Mass of the Tracked particle : "<<gMC->TrackMass()<<std::endl;
                 LOG(INFO)<<" Mass of the Beam from global vertex pointer : "<<gATVP->GetBeamMass()<<std::endl;
        	      // LOG(INFO)<<" Total energy of the current track : "<<((gMC->Etot() - gMC->TrackMass()) * 1000.)<<FairLogger::endl;// Relativistic Mass
                 Double_t StopEnergy = ((gMC->Etot() - gATVP->GetBeamMass()*0.93149401) * 1000.);
                 LOG(INFO)<<" Total energy of the Beam particle before reaction : "<<StopEnergy<<FairLogger::endl;// Relativistic Mass
                 gATVP->SetVertex(StopPos.X(),StopPos.Y(),StopPos.Z(),InPos.X(),InPos.Y(),InPos.Z(),StopMom.Px(),StopMom.Py(),StopMom.Pz(),StopEnergy);
        	 // std::cout<<" Entrance Position 2 - X : "<<InPos.X()<<" - Y : "<<InPos.Y()<<" - Z : "<<InPos.Z()<<std::endl;
                 //  std::cout<<" Stop Position - X : "<<StopPos.X()<<" - Y : "<<StopPos.Y()<<" - Z : "<<StopPos.Z()<<std::endl;

        	}
        		// Increment number of AtTpc det points in TParticle



	    	stack->AddPoint(kAtTpc);





		/*std::cout<<" Current Track Number : "<<stack->GetCurrentTrackNumber()<<std::endl;
		stack->Print(1);
		TParticle* beam_part0 = stack->GetParticle(0);
                std::cout<<" Beam particle 0 mass  "<<beam_part0->GetMass()<<std::endl;
		TParticle* beam_part1 = stack->GetParticle(1);
                std::cout<<" Beam particle 1 mass  "<<beam_part1->GetMass()<<std::endl;
		TParticle* beam_part2 = stack->GetParticle(2);
                std::cout<<" Beam particle 2 mass  "<<beam_part2->GetMass()<<std::endl;
		TParticle* beam_part3 = stack->GetParticle(3);
                std::cout<<" Beam particle 3 mass  "<<beam_part3->GetMass()<<std::endl;*/


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
  if (tsname.Contains("drift_volume") || tsname.Contains("window") || tsname.Contains("cell")) {
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
                            TString VolName,
                            Int_t detCopyID,
                            TVector3 posIn,
                            TVector3 posOut,
                            TVector3 momIn,
                            TVector3 momOut,
                            Double_t time,
                            Double_t length,
                            Double_t eLoss,
			                      Double_t EIni,
			                      Double_t AIni,
                            Int_t A,
                            Int_t Z)
{
    TClonesArray& clref = *fAtTpcPointCollection;
    Int_t size = clref.GetEntriesFast();
   // std::cout<< "ATTPC: Adding Point at (" << posIn.X() << ", " << posIn.Y() << ", " << posIn.Z() << ") cm,  detector " << detID << ", track " << trackID
    //<< ", energy loss " << eLoss << " MeV" <<" with accumulated Energy Loss : "<<fELossAcc<<" MeV "<< std::endl;
    if (fVerboseLevel > 1)
       LOG(INFO) << "ATTPC: Adding Point at (" << posIn.X() << ", " << posIn.Y() << ", " << posIn.Z() << ") cm,  detector " << detID << ", track " << trackID
                  << ", energy loss " << eLoss * 1e06 << " keV" << FairLogger::endl;


    return new (clref[size]) AtTpcPoint(trackID,
                                         detID,
                                         VolName,
                                         detCopyID,
                                         posIn,
                                         posOut,
                                         momIn,
                                         momOut,
                                         time,
                                         length,
                                         eLoss,
					                               EIni,
					                               AIni,
                                         A,
                                         Z);
}

std::pair<Int_t,Int_t> AtTpc::DecodePdG(Int_t PdG_Code)
{
        Int_t A = PdG_Code/10%1000;
        Int_t Z = PdG_Code/10000%1000;

        std::pair<Int_t,Int_t> nucleus;

        if(PdG_Code==2212){
          nucleus.first  = 1;
          nucleus.second = 1;
        }else if(PdG_Code==2112)
        {
          nucleus.first  = 1;
          nucleus.second = 0;
        }else{
          nucleus.first  = A;
          nucleus.second = Z;
        }

        return nucleus;

}

ClassImp(AtTpc)
