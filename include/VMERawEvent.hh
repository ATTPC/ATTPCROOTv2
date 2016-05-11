#ifndef VMERAWEVENT_H
#define VMERAWEVENT_H

#include "TROOT.h"
#include "TObject.h"

#include "ATRawIC.hh"

#include <vector>

class VMERawEvent : public TNamed {
public:
    VMERawEvent();
    VMERawEvent(VMERawEvent *object);
    ~VMERawEvent();

    void SetEventID(Int_t evtid);
    void SetRawIC(ATRawIC *rawic);

    Int_t GetEventID();
    ATRawIC* GetRawIC();
    

private:
    Int_t fEventID;
    ATRawIC fIC;


  ClassDef(VMERawEvent, 1);
};

#endif
