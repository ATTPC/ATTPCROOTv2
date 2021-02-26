#ifndef VMEDECODER_H
#define VMEDECODER_H

#include "TObject.h"
#include "TROOT.h"
#include "TString.h"

class VMEDecoder : public TObject {
public:
   VMEDecoder();
   VMEDecoder(TString filename);
   ~VMEDecoder();

   Bool_t AddData(TString filename);
   Bool_t SetData(Int_t index);
   Bool_t SetNextFile();
   Int_t GetNextEvent();
   ULong64_t GetInitBuffHeader(Int_t event);
   UInt_t GetSubEventNum();
   UInt_t GetTimeStamp();
   UInt_t GetCoinReg();
   std::vector<UInt_t> GetScalerBuff();

   void SetDebugMode() { fDebug = kTRUE; }
   inline UInt_t GetStackID() { return fStackID; }
   inline UInt_t GetContinuationBit() { return fContinuationBit; }
   inline UInt_t CoinReg() { return fCoinReg; }
   inline std::vector<UInt_t> GetScalers() { return fScalerArray; }

   Int_t *GetRawfADC(Int_t chIdx);

private:
   void Initialize();
   std::ifstream fData;
   ULong64_t fFileSize;
   std::vector<TString> fDataList;
   Bool_t fEOF;
   Int_t fCurrentDataID;

   Bool_t fDebug;
   Bool_t fIsScaler;

   Int_t fStackID;
   Int_t fContinuationBit;

   std::vector<UInt_t> fScalerArray;
   Int_t fRawAdc[8 * 512];
   UInt_t fCoinReg;
   UInt_t fTimeStamp;

   ClassDef(VMEDecoder, 1);
};

#endif
