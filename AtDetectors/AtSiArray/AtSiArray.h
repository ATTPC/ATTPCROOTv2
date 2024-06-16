#ifndef AtSIARRAY_H
#define AtSIARRAY_H

#include <FairDetector.h>

#include <Rtypes.h>
#include <TLorentzVector.h>
#include <TString.h>
#include <TVector3.h>

#include <string>
#include <utility>

class AtSiPoint;
class FairVolume;
class TClonesArray;
class TBuffer;
class TClass;
class TList;
class TMemberInspector;

class AtSiArray : public FairDetector {

public:
   /**      Name :  Detector Name
    *       Active: kTRUE for active detectors (ProcessHits() will be called)
    *               kFALSE for inactive detectors
    */
   AtSiArray(const char *Name, Bool_t Active);

   /**      default constructor    */
   AtSiArray();

   /**       destructor     */
   virtual ~AtSiArray();

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

   /**      This method is an example of how to add your own point
    *       of type AtTpcPoint to the clones array
    */
   AtSiPoint *
   AddHit(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Double_t time, Double_t length, Double_t eLoss);

   AtSiPoint *AddHit(Int_t trackID, Int_t detID, TString VolName, Int_t detCopyID, TVector3 posIn, TVector3 pos_out,
                     TVector3 momIn, TVector3 momOut, Double_t time, Double_t length, Double_t eLoss, Double_t EIni,
                     Double_t AIni, Int_t A, Int_t Z);

   std::pair<Int_t, Int_t> DecodePdG(Int_t PdG_Code);

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
   /** Track information to be stored until the track leaves the
   active volume.
   */
   // Int_t          fTrackID;           //!  track index
   // Int_t          fVolumeID;          //!  volume id
   TLorentzVector fPos; //!  position at entrance
   TLorentzVector fMom; //!  momentum at entrance
                        // Double32_t     fTime;              //!  time
   // Double32_t     fLength;            //!  length
   // Double32_t     fELoss;             //!  energy loss

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

   /** container for data points */

   TClonesArray *fAtSiArrayPointCollection; //!

   AtSiArray(const AtSiArray &);
   AtSiArray &operator=(const AtSiArray &);

   ClassDef(AtSiArray, 1)
};

#endif // AtSIARRAY_H
