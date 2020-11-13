#include "AtSiArray.h"

#include "AtSiArrayPoint.h"
#include "AtSiArrayGeo.h"
#include "AtSiArrayGeoPar.h"

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

AtSiArray::AtSiArray()
  : FairDetector("AtSiArray", kTRUE, kAtSiArray),
    fTrackID(-1),
    fVolumeID(-1),
    fPos(),
    fMom(),
    fTime(-1.),
    fLength(-1.),
    fELoss(-1),
    fPosIndex(-1),
    fAtSiArrayPointCollection(new TClonesArray("AtSiArrayPoint")),
    fELossAcc(-1)
{
}

AtSiArray::AtSiArray(const char* name, Bool_t active)
  : FairDetector(name, active, kAtSiArray),
    fTrackID(-1),
    fVolumeID(-1),
    fPos(),
    fMom(),
    fTime(-1.),
    fLength(-1.),
    fELoss(-1),
    fPosIndex(-1),
    fAtSiArrayPointCollection(new TClonesArray("AtSiArrayPoint")),
    fELossAcc(-1)
{
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
  FairRuntimeDb* rtdb= FairRun::Instance()->GetRuntimeDb();
  AtSiArrayGeoPar* par=(AtSiArrayGeoPar*)(rtdb->getContainer("AtSiArrayGeoPar"));

}

Bool_t  AtSiArray::ProcessHits(FairVolume* vol)
{
  /** This method is called from the MC stepping */

    LOG(debug) << "In AtSiArray::ProcessHits";
    // Set parameters at entrance of volume. Reset ELoss.
    if (TVirtualMC::GetMC()->IsTrackEntering()) {
        fELoss = 0.;
        fTime = TVirtualMC::GetMC()->TrackTime() * 1.0e09;
        fLength = TVirtualMC::GetMC()->TrackLength();
        TVirtualMC::GetMC()->TrackPosition(fPos);
        TVirtualMC::GetMC()->TrackMomentum(fMom);
    }

    // Sum energy loss for all steps in the active volume
    fELoss += TVirtualMC::GetMC()->Edep();

    // Create AtSiArrayPoint at exit of active volume
    if (TVirtualMC::GetMC()->IsTrackExiting() || TVirtualMC::GetMC()->IsTrackStop()
        || TVirtualMC::GetMC()->IsTrackDisappeared()) {
        fTrackID = TVirtualMC::GetMC()->GetStack()->GetCurrentTrackNumber();
        fVolumeID = vol->getMCid();
        if (fELoss == 0.) {
            return kFALSE;
        }
        AddHit(fTrackID,
               fVolumeID,
               TVector3(fPos.X(), fPos.Y(), fPos.Z()),
               TVector3(fMom.Px(), fMom.Py(), fMom.Pz()),
               fTime,
               fLength,
               fELoss);

        
        AtStack* stack = static_cast<AtStack*>(TVirtualMC::GetMC()->GetStack());
        stack->AddPoint(kAtSiArray);
    }

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

  FairRootManager::Instance()->Register("AtSiArrayPoint", "AtSiArray",
                                        fAtSiArrayPointCollection, kTRUE);

}

TClonesArray* AtSiArray::GetCollection(Int_t iColl) const
{
  if (iColl == 0) { return fAtSiArrayPointCollection; }
  else { return NULL; }
}

void AtSiArray::Reset()
{
  fAtSiArrayPointCollection->Clear();
}

void AtSiArray::Print(Option_t* option) const
{
    Int_t nHits = fAtSiArrayPointCollection->GetEntriesFast();
    LOG(INFO) << "Si Array: " << nHits << " points registered in this event" << FairLogger::endl;
}

void AtSiArray::ConstructGeometry()
{
  TString fileName=GetGeometryFileName();
  if (fileName.EndsWith(".geo")) {
    LOG(INFO)<<"Constructing Si Array geometry from ASCII file "<<fileName<<FairLogger::endl;
    //ConstructASCIIGeometry();
  } else if (fileName.EndsWith(".root")) {
    LOG(INFO)<<"Constructing Si Array geometry from ROOT file "<<fileName<<FairLogger::endl;
    ConstructRootGeometry();
  } else {
    std::cout << "Geometry format not supported." << std::endl;
  }
}

Bool_t AtSiArray::CheckIfSensitive(std::string name)
{

  TString tsname = name;
  if (tsname.Contains("silicon")) {
    LOG(INFO)<<" Si Array geometry: Sensitive volume found: "<<tsname<<FairLogger::endl;
    return kTRUE;
  }
  return kFALSE;
}

AtSiArrayPoint* AtSiArray::AddHit(Int_t trackID, Int_t detID,
                                      TVector3 pos, TVector3 mom,
                                      Double_t time, Double_t length,
                                      Double_t eLoss)
{
  TClonesArray& clref = *fAtSiArrayPointCollection;
  Int_t size = clref.GetEntriesFast();
  return new(clref[size]) AtSiArrayPoint(trackID, detID, pos, mom,
         time, length, eLoss);
}

// -----   Private method AddHit   --------------------------------------------
AtSiArrayPoint* AtSiArray::AddHit(Int_t trackID,
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
    TClonesArray& clref = *fAtSiArrayPointCollection;
    Int_t size = clref.GetEntriesFast();
   // std::cout<< "ATTPC: Adding Point at (" << posIn.X() << ", " << posIn.Y() << ", " << posIn.Z() << ") cm,  detector " << detID << ", track " << trackID
    //<< ", energy loss " << eLoss << " MeV" <<" with accumulated Energy Loss : "<<fELossAcc<<" MeV "<< std::endl;
    if (fVerboseLevel > 1)
       LOG(INFO) << "Si Array: Adding Point at (" << posIn.X() << ", " << posIn.Y() << ", " << posIn.Z() << ") cm,  detector " << detID << ", track " << trackID
                  << ", energy loss " << eLoss * 1e06 << " keV" << FairLogger::endl;


    return new (clref[size]) AtSiArrayPoint(trackID,
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

std::pair<Int_t,Int_t> AtSiArray::DecodePdG(Int_t PdG_Code)
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

ClassImp(AtSiArray)


