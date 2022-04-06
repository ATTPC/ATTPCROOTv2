/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#ifndef AtTPC_H
#define AtTPC_H

#include <Rtypes.h>
#include <RtypesCore.h>
#include <TString.h>
#include <string>
#include <utility>

#include "FairDetector.h"
#include "TVector3.h"
#include "TLorentzVector.h"

class AtMCPoint;
class FairVolume;
class TClonesArray;
class TBuffer;
class TClass;
class TList;
class TMemberInspector;

class AtTpc : public FairDetector {
private:
   /** Track information to be stored until the track leaves the
   active volume.
   */
   TLorentzVector fPos; //!  position at entrance
   TLorentzVector fMom; //!  momentum at entrance

   Int_t fTrackID;                 //!  track index
   Int_t fVolumeID;                //!  volume id
   Int_t fDetCopyID;               //!  Det volume id  // added by Marc
   Int_t fsector;                  //!  volume id
   TLorentzVector fPosIn, fPosOut; //!  position
   TLorentzVector fMomIn, fMomOut; //!  momentum
   Double32_t fTime_in;            //!  time when entering active volume
   Double32_t fTime_out;           //!  time when exiting active volume
   Double32_t fTime;               //!  time
   Double32_t fLength_in;          //!  length when entering active volume
   Double32_t fLength_out;         //!  length when exiting active volume
   Double32_t fLength;             //!  length
   Double32_t fELoss;              //!  energy loss
   Int_t fPosIndex;                //!
   TClonesArray *fTraCollection;   //!  The hit collection
   Bool_t kGeoSaved;               //!
   TList *flGeoPar;                //!
   TString fVolName;
   Double32_t fELossAcc;
   TLorentzVector InPos;

   /** container for data points */

   TClonesArray *fAtTpcPointCollection; //!

public:
   /**      Name :  Detector Name
    *       Active: kTRUE for active detectors (ProcessHits() will be called)
    *               kFALSE for inactive detectors
    */
   AtTpc(const char *Name, Bool_t Active);
   AtTpc();
   virtual ~AtTpc();

   /** From FairDetector **/
   virtual void Initialize() override;
   virtual Bool_t ProcessHits(FairVolume *v = 0) override;
   virtual void Register() override;
   virtual TClonesArray *GetCollection(Int_t iColl) const override;
   virtual void Reset() override;
   virtual void Print(Option_t *option = "") const override;
   virtual void EndOfEvent() override;

   /** From FairModule **/
   virtual void ConstructGeometry() override;
   virtual Bool_t CheckIfSensitive(std::string name) override;

   AtMCPoint *
   AddHit(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Double_t time, Double_t length, Double_t eLoss);

   AtMCPoint *AddHit(Int_t trackID, Int_t detID, TString VolName, Int_t detCopyID, TVector3 pos, TVector3 mom,
                     Double_t time, Double_t length, Double_t eLoss, Double_t EIni, Double_t AIni, Int_t A, Int_t Z);

private:
   std::pair<Int_t, Int_t> DecodePdG(Int_t PdG_Code);

   void trackEnteringVolume();
   void getTrackParametersFromMC();
   void getTrackParametersWhileExiting();
   void correctPosOut();
   void resetVertex();
   void addHit();
   bool reactionOccursHere();
   void startReactionEvent();

   AtTpc(const AtTpc &);
   AtTpc &operator=(const AtTpc &);

   ClassDefOverride(AtTpc, 2)
};

#endif // NEWDETECTOR_H
