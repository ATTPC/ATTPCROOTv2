/*********************************************************************
 *   AtTPC AtRawEvent Stores a RawEvent composed by the AtTC pads    *
 *   Author: Y. Ayyad            				     *
 *   Log: 07-03-2015 17:16 JST					     *
 *   Adapted from STRawEvent from SPiRITROOT by G. Jhang             *
 *   Edited by Adam Anthony 2/17/2020                                *
 *                                                                   *
 *********************************************************************/

#ifndef AtRAWEVENT_H
#define AtRAWEVENT_H

#include "AtAuxPad.h"
#include "AtPadReference.h" // IWYU pragma: keep

#include <Rtypes.h>
#include <TNamed.h>

#include <cstddef>
#include <functional> // for hash
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map> // for unordered_map
#include <utility>
#include <vector>
class AtPad;
class TBuffer;
class TClass;
class TMemberInspector;

class AtRawEvent : public TNamed {
private:
   using AuxPadMap = std::map<std::string, AtAuxPad>;
   using FpnMap = std::unordered_map<AtPadReference, AtPad>;
   using AtPadPtr = std::unique_ptr<AtPad>;
   using PadVector = std::vector<AtPadPtr>;

   ULong_t fEventID = -1;
   PadVector fPadList;
   AuxPadMap fAuxPadMap;
   FpnMap fFpnMap;

   std::vector<ULong64_t> fTimestamp;

   Bool_t fIsGood = true;
   Bool_t fIsInGate = false;

   std::multimap<Int_t, std::size_t> fSimMCPointMap; //<! Monte Carlo Point - Hit map for kinematics

   friend class AtFilterTask;
   friend class AtFilterFFT;

public:
   AtRawEvent();
   AtRawEvent(AtRawEvent &&obj) = default;
   AtRawEvent(const AtRawEvent &object);
   AtRawEvent &operator=(AtRawEvent object);
   ~AtRawEvent() = default;

   friend void swap(AtRawEvent &first, AtRawEvent &second)
   {
      using std::swap;

      swap(first.fEventID, second.fEventID);
      swap(first.fPadList, second.fPadList);
      swap(first.fAuxPadMap, second.fAuxPadMap);
      swap(first.fFpnMap, second.fFpnMap);
      swap(first.fTimestamp, second.fTimestamp);
      swap(first.fIsGood, second.fIsGood);
      swap(first.fIsInGate, second.fIsInGate);
      swap(first.fSimMCPointMap, second.fSimMCPointMap);
   };

   /// Copy everything but the data (pads, aux pads, and MCPointMap) to this event
   void CopyAllButData(const AtRawEvent *event);

   void Clear(Option_t *opt = nullptr) override;

   /**
    * @brief Create a new pad in this event.
    *
    * Adds a new pad, calling a constructor of AtPad using the passed parameters.
    *
    * @param params Parameters to perfect-forward to the constructor of AtPad
    * @return Pointer to newly created pad
    */
   template <typename... Ts>
   AtPad *AddPad(Ts &&...params)
   {
      fPadList.push_back(std::make_unique<AtPad>(std::forward<Ts>(params)...));
      return fPadList.back().get();
   }

   /**
    * @brief Move a pad into the event
    *
    * Moves a std::unique_ptr of AtPad, or any type derived from AtPad, into the event.
    *
    * @param params std::unique_ptr of pad to move into the event
    * @return Pointer to moved pad
    */
   template <typename T, typename = std::enable_if_t<std::is_base_of<AtPad, std::decay_t<T>>::value>>
   AtPad *AddPad(std::unique_ptr<T> ptr)
   {
      fPadList.push_back(std::move(ptr));
      return fPadList.back().get();
   }
   /**
    * @brief Add new auxilary pad (AtAuxPad) to event
    * @param Name of new auxiliary pad
    * @return Returns a pointer to the newly added pad, or existing pad if auxName is already used,
    * bool returned is true if insert occurred.
    */
   std::pair<AtAuxPad *, bool> AddAuxPad(std::string auxName);

   AtPad *AddFPN(const AtPadReference &ref);

   void RemovePad(Int_t padNum);
   void SetSimMCPointMap(std::multimap<Int_t, std::size_t> map) { fSimMCPointMap = std::move(map); }

   // setters
   void SetEventID(ULong_t evtid) { fEventID = evtid; }
   void SetIsGood(Bool_t value) { fIsGood = value; }
   void SetTimestamp(ULong64_t timestamp, int index = 0);
   void SetNumberOfTimestamps(int numTS) { fTimestamp.resize(numTS, 0); }
   void SetIsExtGate(Bool_t value) { fIsInGate = value; }

   // getters
   ULong_t GetEventID() const { return fEventID; }
   Int_t GetNumPads() const { return fPadList.size(); }
   Int_t GetNumAuxPads() const { return fAuxPadMap.size(); }
   ULong64_t GetTimestamp(int index = 0) const { return index < fTimestamp.size() ? fTimestamp.at(index) : 0; }
   const std::vector<ULong64_t> &GetTimestamps() const { return fTimestamp; }
   Bool_t IsGood() const { return fIsGood; }
   Bool_t GetIsExtGate() const { return fIsInGate; }

   AtPad *GetPad(Int_t padNum) { return const_cast<AtPad *>(const_cast<const AtRawEvent *>(this)->GetPad(padNum)); }
   const AtPad *GetPad(Int_t padNum) const;
   AtPad *GetFpn(const AtPadReference &ref)
   {
      return const_cast<AtPad *>(const_cast<const AtRawEvent *>(this)->GetFpn(ref));
   }
   const AtPad *GetFpn(const AtPadReference &ref) const;
   AtAuxPad *GetAuxPad(std::string auxPad)
   {
      return const_cast<AtAuxPad *>(const_cast<const AtRawEvent *>(this)->GetAuxPad(std::move(auxPad)));
   }
   const AtAuxPad *GetAuxPad(std::string auxPad) const;
   const PadVector &GetPads() const { return fPadList; }
   PadVector &GetPads() { return const_cast<PadVector &>(const_cast<const AtRawEvent *>(this)->GetPads()); }

   const AuxPadMap &GetAuxPads() const { return fAuxPadMap; }
   const FpnMap &GetFpnPads() const { return fFpnMap; }
   std::multimap<Int_t, std::size_t> &GetSimMCPointMap() { return fSimMCPointMap; }

   ClassDefOverride(AtRawEvent, 6);
};

#endif
