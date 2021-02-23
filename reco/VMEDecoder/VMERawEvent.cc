#include <iostream>
#include <iomanip>

#include "VMERawEvent.h"
#include "AtRawIC.h"

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
void  VMERawEvent::SetRawIC(AtRawIC *rawic)       { fIC=*rawic; }

Int_t   VMERawEvent::GetEventID()                  { return fEventID; }
AtRawIC* VMERawEvent::GetRawIC()                   { return &fIC;}

