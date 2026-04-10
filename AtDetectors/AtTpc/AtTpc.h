/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#ifndef AtTPC_H
#define AtTPC_H

#include <FairDetector.h>

#include <Rtypes.h>
#include <TLorentzVector.h>
#include <TString.h>
#include <TVector3.h>

#include <string>
#include <utility>

class AtMCPoint;
class FairVolume;
class TClonesArray;
class TBuffer;
class TClass;
class TList;
class TMemberInspector;

class AtTpc : public FairDetector {
public:
   struct StepState {
      int trackID = -1;
      int pdg = 0;
      TString volumeName;
      int volumeID = -1;
      int detCopyID = -1;
      bool beamTrack = false;
      bool entering = false;
      bool exiting = false;
      bool stopping = false;
      bool disappeared = false;
      double energyLoss = 0.0;     // GeV
      double timeNs = 0.0;         // ns
      double trackLength = 0.0;    // cm
      double totalEnergy = 0.0;    // GeV
      double trackMass = 0.0;      // GeV/c^2
      TLorentzVector pos;
      TLorentzVector mom;
      TLorentzVector posOut;
      TLorentzVector momOut;
   };

private:
   /** Track information to be stored until the track leaves the
   active volume.
   */
   TLorentzVector fPos; //!  position at entrance
   TLorentzVector fMom; //!  momentum at entrance

   Int_t fTrackID;                 //!  track index
   Int_t fVolumeID;                //!  volume id
   Int_t fDetCopyID{};             //!  Det volume id  // added by Marc
   Int_t fsector{};                //!  volume id
   TLorentzVector fPosIn, fPosOut; //!  position
   TLorentzVector fMomIn, fMomOut; //!  momentum
   Double32_t fTime_in{};          //!  time when entering active volume
   Double32_t fTime_out{};         //!  time when exiting active volume
   Double32_t fTime;               //!  time
   Double32_t fLength_in{};        //!  length when entering active volume
   Double32_t fLength_out{};       //!  length when exiting active volume
   Double32_t fLength;             //!  length
   Double32_t fELoss;              //!  energy loss
   Int_t fPosIndex;                //!
   TClonesArray *fTraCollection{}; //!  The hit collection
   Bool_t kGeoSaved{};             //!
   TList *flGeoPar{};              //!
   TString fVolName;
   Double32_t fELossAcc;
   TLorentzVector InPos;
   bool fIsBeamTrack = false;
   bool fStopOnReactionVolumeExit{false};

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

   /**
    * Process a detector step from a transport-neutral snapshot.
    *
    * Expected call sequence per volume traversal:
    *  1. One step with entering=true — resets fELossAcc to 0 and captures entry position/momentum.
    *  2. Zero or more steps with entering=false, exiting=false — accumulate energy loss in fELossAcc.
    *  3. One step with exiting=true (or stopping/disappeared) — captures exit position/momentum
    *     and may trigger resetVertex() for beam tracks leaving a reaction volume.
    *
    * Two entering=true steps without an intervening exiting=true step will silently reset
    * the accumulated energy loss, discarding data from the first volume traversal.
    *
    * Returns true when the transport should stop at this step (reaction fired or beam exited
    * a reaction volume with fStopOnReactionVolumeExit enabled).
    */
   bool ProcessStep(const StepState &step);

   /// When true, ProcessStep returns true (stop transport) when any particle exits a reaction volume.
   /// Used by SimpleSim; defaults to false to preserve Geant4 behavior.
   void SetStopOnReactionVolumeExit(bool val) { fStopOnReactionVolumeExit = val; }

   /// Canonical check for whether a volume name is a sensitive detector volume.
   static bool IsSensitiveVolume(const std::string &name);

private:
   std::pair<Int_t, Int_t> DecodePdG(Int_t PdG_Code);

   void trackEnteringVolume(const StepState &step);
   void getTrackParametersFromStep(const StepState &step);
   void getTrackParametersWhileExiting(const StepState &step);
   void correctPosOut();
   void resetVertex();
   void addHit(const StepState &step);
   bool reactionOccursHere();
   void startReactionEvent(const StepState &step);
   bool IsReactionVolume(const TString &volumeName) const;

   AtTpc(const AtTpc &);
   AtTpc &operator=(const AtTpc &);

   ClassDefOverride(AtTpc, 2)
};

#endif // NEWDETECTOR_H
