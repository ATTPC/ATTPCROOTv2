/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#ifndef DEGAI_H
#define DEGAI_H

#include <FairDetector.h>

#include <Rtypes.h>  // for Int_t, Bool_t, Double32_t, Double_t, THash...
#include <TString.h> // for TString
#include <TVector3.h>

#include <string> // for string
class TBuffer;
class TClass;
class TMemberInspector;
class FairVolume;
class TClonesArray;
class AtMCPoint;

class AtDeGAi : public FairDetector {
private:
   /** Track information to be stored until the track leaves the
   active volume.
   */

   Int_t fTrackID;       //!  track index
   Int_t fVolumeID;      //!  volume id
   TString fVolName;     //!  volume name
   Int_t fDetCopyID;     //!  Det volume id
   Double32_t fTime;     //!  time
   Double32_t fLength;   //!  length
   Double32_t fELoss;    //!  energy loss
   Double32_t fELossAcc; //!  accumulated energy loss

   /** container for data points */
   TClonesArray *fAtDeGAiPointCollection; //!

public:
   /**      Name :  Detector Name
    *       Active: kTRUE for active detectors (ProcessHits() will be called)
    *               kFALSE for inactive detectors
    */
   AtDeGAi(const char *Name, Bool_t Active);

   /**      default constructor    */
   AtDeGAi();

   /**       destructor     */
   virtual ~AtDeGAi();

   /**      Initialization of the detector is done here    */
   virtual void Initialize();

   /**       this method is called for each step during simulation
    *       (see FairMCApplication::Stepping())
    */
   virtual Bool_t ProcessHits(FairVolume *v = 0);

   /**       Registers the produced collections in FAIRRootManager.     */
   virtual void Register();

   /** Gets the produced collections */
   virtual TClonesArray *GetCollection(Int_t iColl) const;

   /**      has to be called after each event to reset the containers      */
   virtual void Reset();

   /** Virtual method Print
    **
    ** Screen output of hit collection.
    **/
   virtual void Print(Option_t *option = "") const;

   /**      Create the detector geometry        */
   void ConstructGeometry();

   Bool_t CheckIfSensitive(std::string name);

   /** The following methods can be implemented if you need to make
    *  any optional action in your detector during the transport.
    */

   virtual void CopyClones(TClonesArray *cl1, TClonesArray *cl2, Int_t offset) { ; }
   virtual void SetSpecialPhysicsCuts() { ; }
   virtual void EndOfEvent();
   virtual void FinishPrimary() { ; }
   virtual void FinishRun() { ; }
   virtual void BeginPrimary() { ; }
   virtual void PostTrack() { ; }
   virtual void PreTrack() { ; }
   virtual void BeginEvent() { ; }

private:
   /**      This method is an example of how to add your own point
    *       of type AtMCPoint to the clones array
    */
   AtMCPoint *
   AddPoint(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Double_t tof, Double_t length, Double_t eLoss);
   AtDeGAi(const AtDeGAi &);
   AtDeGAi &operator=(const AtDeGAi &);

   ClassDef(AtDeGAi, 1)
};

#endif // DEGAI_H
