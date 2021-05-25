#ifndef VMECORE_H
#define VMECORE_H

#include "TObject.h"
#include "TString.h"
#include "VMEDecoder.h"
#include "VMERawEvent.h"

class VMECore : public TObject {

public:
   VMECore();
   ~VMECore();

   void Initialize();
   Bool_t AddData(TString filename);
   Bool_t SetData(Int_t value);
   // AtRawEvent *GetRawEvent(Int_t eventID = -1);
   VMERawEvent *GetRawVMEEvent(
      Int_t eventID = -1); // TODO: For the moment no class has been created to cointain the Raw Event from the VME

   inline void SetICChannel(Int_t val) { fICChannel = val; }
   inline void SetMeshChannel(Int_t val) { fMeshChannel = val; }
   inline void SetTriggerChannel(Int_t val) { fTriggerChannel = val; }

private:
   VMEDecoder *fVMEDecoderPtr;
   Bool_t fIsData;
   UInt_t fPrevEventNo;
   UInt_t fCurrEventNo;

   Int_t fICChannel;
   Int_t fMeshChannel;
   Int_t fTriggerChannel;
   VMERawEvent *fVMERawEventPtr;

   ClassDef(VMECore, 1);
};

#endif
