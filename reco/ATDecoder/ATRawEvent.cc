/*********************************************************************
 *   ATTPC ATRawEvent Stores a RawEvent composed by the ATTC pads    *
 *   Author: Y. Ayyad            				     *
 *   Log: 07-03-2015 17:16 JST					     *
 *   Adapted from STRawEvent from SPiRITROOT by G. Jhang             *
 *   Edited by Adam Anthony 2/17/2020                                *
 *********************************************************************/

#include <iostream>
#include <iomanip>

#include "ATRawEvent.hh"
#include "ATPad.hh"

ClassImp(ATRawEvent);

ATRawEvent::ATRawEvent()
  :TNamed("ATRawEvent", "Raw event container")
{
  fEventID = -1;
  fPadArray.reserve(10240);// TODO Prototype size is smaller we do not need such size

  fIsGood = kTRUE;
}

ATRawEvent::ATRawEvent(ATRawEvent *object)
  :TNamed("ATRawEvent", "Raw event container")
{
  fEventID = object -> GetEventID();
  fPadArray = *(object -> GetPads());

  fIsGood = object -> IsGood();
}

ATRawEvent::~ATRawEvent()
{
}

void ATRawEvent::Clear()
{
  fEventID = 0;
  fPadArray.clear();

  fIsGood = kTRUE;
}


// setters
void ATRawEvent::SetEventID(ULong_t evtid) { fEventID = evtid; }
void ATRawEvent::SetPad(ATPad *pad)      { fPadArray.push_back(*pad); }
void ATRawEvent::SetIsGood(Bool_t value) { fIsGood = value; }
void ATRawEvent::SetTimestamp(ULong_t timestamp) { fTimestamp = timestamp; }

void ATRawEvent::RemovePad(Int_t padNo)
{
  if (!(padNo < GetNumPads()))
    return;

  fPadArray.erase(fPadArray.begin() + padNo);
}

// getters
ULong_t   ATRawEvent::GetEventID()         { return fEventID; }
Int_t   ATRawEvent::GetNumPads()         { return fPadArray.size(); }
Bool_t  ATRawEvent::IsGood()             { return fIsGood; }
ULong_t ATRawEvent::GetTimestamp()       { return fTimestamp; }
ATPad  *ATRawEvent::GetPad(Int_t padNo)  { return (padNo < GetNumPads() ? &fPadArray[padNo] : NULL); }
std::vector<ATPad> *ATRawEvent::GetPads()     { return &fPadArray; }

ATPad *ATRawEvent::GetPad(Int_t PadNum, Bool_t& IsValid)
{
  for(std::vector<ATPad>::iterator it = fPadArray.begin(); it != fPadArray.end(); ++it)
  {
    //std::cout<<" ATRawEvent::GetPad : "<<(*it).GetPadNum()<<std::endl;

    if( (*it).GetPadNum() == PadNum )
    {
      IsValid = kTRUE;
      return &(*it);
    }
  }// End loop over all valid pads

  IsValid = kFALSE;
  return NULL;
}
