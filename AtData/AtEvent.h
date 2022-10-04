#ifndef AtEVENT_H
#define AtEVENT_H
// IWYU pragma: no_include <ext/alloc_traits.h>

#include "AtAuxPad.h"
#include "AtHit.h"

#include <FairLogger.h>

#include <Rtypes.h>
#include <TNamed.h>

#include <algorithm>
#include <array>
#include <map>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

class TBuffer;
class TClass;
class TMemberInspector;

class AtRawEvent;

class AtEvent : public TNamed {
public:
   using AuxPadVector = std::vector<AtAuxPad>;
   using TraceArray = std::array<Float_t, 512>;
   using HitPtr = std::unique_ptr<AtHit>;
   using HitVector = std::vector<HitPtr>;

private:
   Int_t fEventID;
   Bool_t fIsGood;
   Bool_t fIsInGate;
   ULong_t fTimestamp;
   Double_t fEventCharge = -100;
   Double_t fRhoVariance = 0;

   HitVector fHitArray;
   std::vector<AtAuxPad> fAuxPadArray;
   std::map<Int_t, Int_t> fMultiplicityMap;

   TraceArray fMeshSig{};

public:
   AtEvent();
   AtEvent(Int_t eventID, Bool_t isGood, Bool_t isInGate = false, ULong_t timestamp = 0);
   AtEvent(const AtEvent &copy);
   AtEvent(const AtRawEvent &copy);
   ~AtEvent() = default;

   void Clear(Option_t *opt = nullptr) override;

   // Copies everything except the hit array from the passed AtEvent
   void CopyFrom(const AtEvent &event);

   /**
    * @brief Create a new hit in this event.
    * Adds a new hit, calling a constructor of AtHit using the passed parameters.
    * Will set the hitID to the next availible if it was not set by the
    * AtHit constructor. Allowing this function to handle hitIDs will ensure they
    * remain unique within an event.
    *
    * @param params Parameters to be perfect-forwarded to the constructor of AtHit.
    * @return Reference to added hit
    */
   template <typename... Ts>
   AtHit &AddHit(Ts &&...params)
   {
      fHitArray.emplace_back(std::make_unique<AtHit>(std::forward<Ts>(params)...));
      if (fHitArray.back()->GetHitID() == -1)
         fHitArray.back()->SetHitID(fHitArray.size() - 1);
      LOG(debug) << "Adding hit with ID " << fHitArray.back()->GetHitID() << " to event " << fEventID;

      return *(fHitArray.back());
   }

   /**
    * @brief Move a hit into this event.
    * Adds a new hit, moving a unique pointer in.
    * Will set the hitID to the next availible if it was not set by the
    * AtHit constructor (ie is still -1). Allowing this function to handle hitIDs will ensure they
    * remain unique within an event.
    *
    * @param[in] ptr unique_ptr to the AtHit to add to the event.
    * @return Reference to added hit
    */
   template <typename T, typename = std::enable_if_t<std::is_base_of<AtHit, std::decay_t<T>>::value>>
   AtHit &AddHit(std::unique_ptr<T> ptr)
   {
      fHitArray.push_back(std::move(ptr));
      if (fHitArray.back()->GetHitID() == -1)
         fHitArray.back()->SetHitID(fHitArray.size() - 1);
      LOG(debug) << "Adding hit with ID " << fHitArray.back()->GetHitID() << " to event " << fEventID;

      return *(fHitArray.back());
   }

   // Copies passed aux pad into the event's auxiliary pad array
   void AddAuxPad(AtAuxPad auxPad) { fAuxPadArray.push_back(std::move(auxPad)); }

   void SetEventID(Int_t evtid) { fEventID = evtid; }
   void SetTimestamp(ULong_t timestamp) { fTimestamp = timestamp; }
   void SetEventCharge(Double_t Qevent) { fEventCharge = Qevent; }
   void SetRhoVariance(Double_t RhoVariance) { fRhoVariance = RhoVariance; }
   void SetIsGood(Bool_t value) { fIsGood = value; }
   void SetIsInGate(Bool_t value) { fIsInGate = value; }

   void SetMultiplicityMap(std::map<Int_t, Int_t> MultiMap) { fMultiplicityMap = std::move(MultiMap); }
   void SetMeshSignal(TraceArray mesharray) { fMeshSig = std::move(mesharray); }
   void SetMeshSignal(Int_t idx, Float_t val);

   Int_t GetEventID() const { return fEventID; }
   Long_t GetTimestamp() const { return fTimestamp; }
   Int_t GetNumHits() const { return fHitArray.size(); }
   Double_t GetEventCharge() const { return fEventCharge; }
   Double_t GetRhoVariance() const { return fRhoVariance; }
   const TraceArray &GetMesh() const { return fMeshSig; }
   Int_t GetHitPadMult(Int_t PadNum); // Returns the multiplicity of the pad where this hit belongs to

   const AtHit &GetHit(Int_t hitNo) const { return *fHitArray.at(hitNo); }
   [[deprecated("Use GetHits()")]] std::vector<AtHit> GetHitArray() const;
   const HitVector &GetHits() const { return fHitArray; }
   const AuxPadVector &GetAuxPads() const { return fAuxPadArray; }
   const std::map<Int_t, Int_t> &GetMultiMap() { return fMultiplicityMap; }

   Bool_t IsGood() const { return fIsGood; }
   Bool_t IsInGate() const { return fIsInGate; }

   void SortHitArray();
   void SortHitArrayTime();

   ClassDefOverride(AtEvent, 5);
};

#endif
