#ifndef AtEVENT_H
#define AtEVENT_H

#include "TROOT.h"
#include "TObject.h"
#include "FairLogger.h"

#include <map>
#include <vector>

#include "AtHit.h"
#include "AtPad.h"

using std::map;
using hitVector = std::vector<AtHit>;
using padVector = std::vector<AtPad>;
using traceArray = std::array<Float_t, 512>;

class AtEvent : public TNamed {
private:
   Int_t fEventID;
   Bool_t fIsGood;
   Bool_t fIsInGate = false;
   Double_t fEventCharge = -100;
   Double_t fRhoVariance = 0;
   ULong_t fTimestamp = 0;

   hitVector fHitArray;
   padVector fAuxPadArray;
   map<Int_t, Int_t> fMultiplicityMap;

   traceArray fMeshSig;

public:
   AtEvent();
   AtEvent(Int_t eventID, Bool_t isGood);
   AtEvent(const AtEvent &copy) = default;
   ~AtEvent() = default;

   // Adds a new hit to the hit array, and returns a referece to the new hit to be
   // filled. This is done to avoid the create and subsequent copy of a hit and also
   // avoid dealing with the memory managment
   template <typename... Ts>
   AtHit &AddHit(Ts &&... params)
   {
      LOG(debug) << "Adding hit with ID " << fHitArray.size() << " to event " << fEventID;
      fHitArray.emplace_back(fHitArray.size(), std::forward<Ts>(params)...);
      return fHitArray.back();
   }
   /*AtHit &AddHit(const XYZPoint &loc, Double_t charge);
   AtHit &AddHit(Int_t padNum, const XYZPoint &loc, Double_t charge);
   AtHit &AddHit();
   */

   // Adds a new auxiliary pad to the auxiliary pad array, and returns a referece to
   // the new auxiliary pad to be filled. This is done to avoid the create and subsequent
   // copy of a hit and also avoid dealing with the memory managment
   AtPad &AddAuxPad();
   // Copies passed aux pad into the event's auxiliary pad array
   void AddAuxPad(const AtPad &auxPad) { fAuxPadArray.push_back(auxPad); }

   void SetEventID(Int_t evtid) { fEventID = evtid; }
   void SetTimestamp(ULong_t timestamp) { fTimestamp = timestamp; }
   void SetEventCharge(Double_t Qevent) { fEventCharge = Qevent; }
   void SetRhoVariance(Double_t RhoVariance) { fRhoVariance = RhoVariance; }
   void SetIsGood(Bool_t value) { fIsGood = value; }
   void SetIsInGate(Bool_t value) { fIsInGate = value; }

   void SetMultiplicityMap(const std::map<Int_t, Int_t> &MultiMap);
   void SetMeshSignal(const traceArray &mesharray);
   void SetMeshSignal(Int_t idx, Float_t val);

   const AtHit &GetHit(Int_t hitNo) const { return fHitArray.at(hitNo); }
   const hitVector &GetHitArray() const { return fHitArray; }
   const padVector &GetAuxPadArray() const { return fAuxPadArray; }
   Int_t GetEventID() const { return fEventID; }
   Long_t GetTimestamp() const { return fTimestamp; }
   Int_t GetNumHits() const { return fHitArray.size(); }
   Double_t GetEventCharge() const { return fEventCharge; }
   Double_t GetRhoVariance() const { return fRhoVariance; }
   const traceArray &GetMesh() const { return fMeshSig; }
   Int_t GetHitPadMult(Int_t PadNum); // Returns the multiplicity of the pad where this hit belongs to

   Bool_t IsGood() const { return fIsGood; }
   Bool_t IsInGate() const { return fIsInGate; }

   void SortHitArray();
   void SortHitArrayTime();

   ClassDef(AtEvent, 4);
};

// Bool_t operator<(const AtHit &s1, const AtHit &s2);

#endif
