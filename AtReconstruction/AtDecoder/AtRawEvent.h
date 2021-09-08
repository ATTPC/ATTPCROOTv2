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

#include "TROOT.h"
#include "TObject.h"

#include "AtPad.h"

#include <vector>
#include <iostream>
#include <map>
#include <iterator>

using AuxPadMap = std::map<std::string, AtPad>;
using PadVector = std::vector<AtPad>;

class AtRawEvent : public TNamed {
private:
   ULong_t fEventID;
   PadVector fPadList;
   AuxPadMap fAuxPadMap;
   std::vector<ULong64_t> fTimestamp;

   Bool_t fIsGood;
   Bool_t fIsinGate;

   std::multimap<Int_t, std::size_t> fSimMCPointMap; //<! Monte Carlo Point - Hit map for kinematics

public:
   AtRawEvent();
   AtRawEvent(AtRawEvent *object);

   void Clear();

   // setters
   void SetEventID(ULong_t evtid);
   AtPad &AddPad(int padNum);
   std::pair<AuxPadMap::iterator, bool> AddAuxPad(std::string auxName);
   void SetIsGood(Bool_t value);
   void RemovePad(Int_t padNum);
   void SetTimestamp(ULong64_t timestamp, int index = 0);
   void SetNumberOfTimestamps(int numTS);
   void SetIsExtGate(Bool_t value);
   void SetSimMCPointMap(std::multimap<Int_t, std::size_t> map);

   // getters
   ULong_t GetEventID() const;
   Int_t GetNumPads() const;
   Int_t GetNumAuxPads() const;
   ULong64_t GetTimestamp(int index = 0) const;
   const std::vector<ULong64_t> *GetTimestamps() const;
   Bool_t IsGood() const;
   Bool_t GetIsExtGate() const;

   PadVector &GetPads();
   AuxPadMap &GetAuxPads();

   AtPad *GetPad(Int_t padNum);
   AtPad *GetAuxPad(std::string auxPad);

   std::multimap<Int_t, std::size_t> &GetSimMCPointMap();

   ClassDef(AtRawEvent, 4);
};

#endif
