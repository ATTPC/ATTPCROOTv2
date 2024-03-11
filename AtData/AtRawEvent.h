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

#include "AtBaseEvent.h"
#include "AtGenericTrace.h" // IWYU pragma: keep
#include "AtPadReference.h" // IWYU pragma: keep

#include <Rtypes.h>

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

class AtRawEvent : public AtBaseEvent {
private:
   using AtPadPtr = std::unique_ptr<AtPad>;
   using FpnMap = std::unordered_map<AtPadReference, AtPad>;
   using PadVector = std::vector<AtPadPtr>;
   using AtGenTracePtr = std::unique_ptr<AtGenericTrace>;
   using GenTraceVector = std::vector<AtGenTracePtr>;

   PadVector fPadList;
   FpnMap fFpnMap;
   GenTraceVector fGTraceList;

   std::multimap<Int_t, std::size_t> fSimMCPointMap; //<! Monte Carlo Point - Hit map for kinematics

   friend class AtFilterTask;
   friend class AtFilterFFT;

public:
   AtRawEvent() : AtBaseEvent("AtRawEvent"){};
   AtRawEvent(AtRawEvent &&obj) = default;
   AtRawEvent(const AtRawEvent &object);
   AtRawEvent(const AtBaseEvent &object) : AtBaseEvent(object) { SetName("AtRawEvent"); }
   AtRawEvent &operator=(AtRawEvent object);
   virtual ~AtRawEvent() = default;

   friend void swap(AtRawEvent &first, AtRawEvent &second)
   {
      using std::swap;
      swap(dynamic_cast<AtBaseEvent &>(first), dynamic_cast<AtBaseEvent &>(second));
      swap(first.fPadList, second.fPadList);
      swap(first.fFpnMap, second.fFpnMap);
      swap(first.fSimMCPointMap, second.fSimMCPointMap);
   };

   /// Copy everything but the data (pads, aux pads, and MCPointMap) to this event
   // void CopyAllButData(const AtRawEvent *event);

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
   AtPad *AddPad(Ts &&... params)
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

   AtPad *AddFPN(const AtPadReference &ref);

   void RemovePad(Int_t padNum);
   void SetSimMCPointMap(std::multimap<Int_t, std::size_t> map) { fSimMCPointMap = std::move(map); }

   template <typename... Ts>
   AtGenericTrace *AddGenericTrace(Ts &&... params)
   {
      fGTraceList.push_back(std::make_unique<AtGenericTrace>(std::forward<Ts>(params)...));
      return fGTraceList.back().get();
   }

   AtGenericTrace *AddGenericTrace(std::unique_ptr<AtGenericTrace> ptr)
   {
      fGTraceList.push_back(std::move(ptr));
      return fGTraceList.back().get();
   }

   // getters
   Int_t GetNumPads() const { return fPadList.size(); }
   Int_t GetNumAuxPads() const { return fAuxPadMap.size(); }
   AtPad *GetPad(Int_t padNum) { return const_cast<AtPad *>(const_cast<const AtRawEvent *>(this)->GetPad(padNum)); }
   const AtPad *GetPad(Int_t padNum) const;
   AtPad *GetFpn(const AtPadReference &ref)
   {
      return const_cast<AtPad *>(const_cast<const AtRawEvent *>(this)->GetFpn(ref));
   }
   const AtPad *GetFpn(const AtPadReference &ref) const;
   const PadVector &GetPads() const { return fPadList; }
   PadVector &GetPads() { return const_cast<PadVector &>(const_cast<const AtRawEvent *>(this)->GetPads()); }

   const GenTraceVector &GetGenTraces() const { return fGTraceList; }

   const FpnMap &GetFpnPads() const { return fFpnMap; }
   std::multimap<Int_t, std::size_t> &GetSimMCPointMap() { return fSimMCPointMap; }

   ClassDefOverride(AtRawEvent, 7);
};

#endif
