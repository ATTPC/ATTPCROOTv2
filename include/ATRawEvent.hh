/*********************************************************************
 *   ATTPC ATRawEvent Stores a RawEvent composed by the ATTC pads    *
 *   Author: Y. Ayyad            				                     *
 *   Log: 07-03-2015 17:16 JST					                     *
 *   Adapted from STRawEvent from SPiRITROOT by G. Jhang             *
 *								                                     *
 *********************************************************************/

#ifndef ATRAWEVENT_H
#define ATRAWEVENT_H

#include "TROOT.h"
#include "TObject.h"

#include "ATPad.hh"

#include <vector>

class ATRawEvent : public TNamed {
public:
    ATRawEvent();
    ATRawEvent(ATRawEvent *object);
    ~ATRawEvent();

    void Clear();

    // setters
    void SetEventID(Int_t evtid);
    void SetPad(ATPad *pad);
    void SetIsGood(Bool_t value);
    void RemovePad(Int_t padNo);


    // getters
    Int_t GetEventID();
    Int_t GetNumPads();
    Bool_t IsGood();

    std::vector<ATPad> *GetPads();

    ATPad *GetPad(Int_t padNo);
    ATPad *GetPad(Int_t PadNum,Bool_t& IsValid);


private:
    Int_t fEventID;
    std::vector<ATPad> fPadArray;

    Bool_t fIsGood;

    ClassDef(ATRawEvent, 1);
};

#endif
