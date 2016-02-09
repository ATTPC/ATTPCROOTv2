#include <iostream>
#include <iomanip>

#include "VMERawEvent.hh"
#include "ATRawIC.hh"

ClassImp(VMERawEvent);

VMERawEvent::VMERawEvent()
:TNamed("VMERawEvent", "VME Raw event container")
{
    fEventID = -1;
 
}

VMERawEvent::VMERawEvent(VMERawEvent *object)
:TNamed("VMERawEvent", "VME Raw event container")
{
    fEventID = object -> GetEventID();
    fIC=*(object -> GetRawIC());
}


VMERawEvent::~VMERawEvent()
{
}


void  VMERawEvent::SetEventID(Int_t evtid)       { fEventID = evtid; }
void  VMERawEvent::SetRawIC(ATRawIC *rawic)       { fIC=*rawic; }

Int_t   VMERawEvent::GetEventID()                  { return fEventID; }
ATRawIC* VMERawEvent::GetRawIC()                   { return &fIC;}

