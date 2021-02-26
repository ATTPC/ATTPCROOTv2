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

class AtRawEvent : public TNamed {
public:
   AtRawEvent();
   AtRawEvent(AtRawEvent *object);
   ~AtRawEvent();

   void Clear();

   // setters
   void SetEventID(ULong_t evtid);
   void SetPad(AtPad *pad);
   void SetIsGood(Bool_t value);
   void RemovePad(Int_t padNo);
   void SetTimestamp(ULong64_t timestamp, int index = 0);
   void SetNumberOfTimestamps(int numTS);
   void SetIsExtGate(Bool_t value);
   void SetSimMCPointMap(std::multimap<Int_t, std::size_t> map);

   // getters
   ULong_t GetEventID();
   Int_t GetNumPads();
   ULong64_t GetTimestamp(int index = 0);
   std::vector<ULong64_t> *GetTimestamps();
   Bool_t IsGood();
   Bool_t GetIsExtGate();

   std::vector<AtPad> *GetPads();

   AtPad *GetPad(Int_t padNo);
   AtPad *GetPad(Int_t PadNum, Bool_t &IsValid);

   std::multimap<Int_t, std::size_t> &GetSimMCPointMap();

private:
   ULong_t fEventID;
   std::vector<AtPad> fPadArray;
   std::vector<ULong64_t> fTimestamp;

   Bool_t fIsGood;
   Bool_t fIsinGate;

   std::multimap<Int_t, std::size_t> fSimMCPointMap; //<! Monte Carlo Point - Hit map for kinematics

   ClassDef(AtRawEvent, 3);
};

#endif
