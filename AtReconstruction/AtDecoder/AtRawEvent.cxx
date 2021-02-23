/*********************************************************************
 *   AtTPC AtRawEvent Stores a RawEvent composed by the AtTC pads    *
 *   Author: Y. Ayyad            				     *
 *   Log: 07-03-2015 17:16 JST					     *
 *   Adapted from STRawEvent from SPiRITROOT by G. Jhang             *
 *   Edited by Adam Anthony 2/17/2020                                *
 *********************************************************************/

#include <iostream>
#include <iomanip>

#include "AtRawEvent.h"
#include "AtPad.h"

ClassImp(AtRawEvent);

AtRawEvent::AtRawEvent() : TNamed("AtRawEvent", "Raw event container")
{
   fEventID = -1;
   fPadArray.reserve(10240); // TODO Prototype size is smaller we do not need such size
   SetNumberOfTimestamps(1);
   fIsGood = kTRUE;
   fIsinGate = kFALSE;

   fSimMCPointMap.clear();
}

AtRawEvent::AtRawEvent(AtRawEvent *object) : TNamed("AtRawEvent", "Raw event container")
{
   fEventID = object->GetEventID();
   fPadArray = *(object->GetPads());
   fIsGood = object->IsGood();

   fSimMCPointMap = object->GetSimMCPointMap();
   fTimestamp = *(object->GetTimestamps());
}

AtRawEvent::~AtRawEvent() {}

void AtRawEvent::Clear()
{
   fEventID = 0;
   fPadArray.clear();

   fIsGood = kTRUE;
}

// setters
void AtRawEvent::SetEventID(ULong_t evtid)
{
   fEventID = evtid;
}
void AtRawEvent::SetPad(AtPad *pad)
{
   fPadArray.push_back(*pad);
}
void AtRawEvent::SetIsGood(Bool_t value)
{
   fIsGood = value;
}
void AtRawEvent::SetTimestamp(ULong64_t timestamp, int index)
{
   fTimestamp.at(index) = timestamp;
}
void AtRawEvent::SetNumberOfTimestamps(int numTS)
{
   fTimestamp.resize(numTS, 0);
}
void AtRawEvent::SetIsExtGate(Bool_t value)
{
   fIsinGate = value;
}
void AtRawEvent::SetSimMCPointMap(std::multimap<Int_t, std::size_t> map)
{
   fSimMCPointMap = map;
};

void AtRawEvent::RemovePad(Int_t padNo)
{
   if (!(padNo < GetNumPads()))
      return;

   fPadArray.erase(fPadArray.begin() + padNo);
}

// getters

ULong_t AtRawEvent::GetEventID()
{
   return fEventID;
}
Int_t AtRawEvent::GetNumPads()
{
   return fPadArray.size();
}
Bool_t AtRawEvent::IsGood()
{
   return fIsGood;
}
ULong64_t AtRawEvent::GetTimestamp(int index)
{
   return fTimestamp.at(index);
}
std::vector<ULong64_t> *AtRawEvent::GetTimestamps()
{
   return &fTimestamp;
}
AtPad *AtRawEvent::GetPad(Int_t padNo)
{
   return (padNo < GetNumPads() ? &fPadArray[padNo] : NULL);
}
std::vector<AtPad> *AtRawEvent::GetPads()
{
   return &fPadArray;
}
Bool_t AtRawEvent::GetIsExtGate()
{
   return fIsinGate;
}
std::multimap<Int_t, std::size_t> &AtRawEvent::GetSimMCPointMap()
{
   return fSimMCPointMap;
}

AtPad *AtRawEvent::GetPad(Int_t PadNum, Bool_t &IsValid)
{
   for (std::vector<AtPad>::iterator it = fPadArray.begin(); it != fPadArray.end(); ++it) {
      // std::cout<<" AtRawEvent::GetPad : "<<(*it).GetPadNum()<<std::endl;

      if ((*it).GetPadNum() == PadNum) {
         IsValid = kTRUE;
         return &(*it);
      }
   } // End loop over all valid pads

   IsValid = kFALSE;
   return NULL;
}
