// =================================================
//  AtCore Class
//  Original author : Genie Jhang ( geniejhang@majimak.com )
//  Adapted for AtTPCROOT by Y. Ayyad (ayyadlim@nscl.msu.edu)
// =================================================

#ifndef _ATCORE2_H_
#define _ATCORE2_H_

#include <TObject.h>
#include <TString.h>
#include <TClonesArray.h>

#include "AtRawEvent.h"
#include "AtMap.h"
#include "AtTpcMap.h"
#include "AtTpcProtoMap.h"
#include "AtPedestal.h"
#include "AtGadgetIIMap.h"

#include "GETDecoder2.h"

#include <mutex>
#include <tuple>

class AtCore2 : public TObject {
private:
   // Bool_t GetIsAuxChannel(Int_t val);

   Int_t fNumTbs;
   Int_t fNumCobo;

   GETDecoder2 *fDecoderPtr[4];
   Bool_t fIsData;

   AtPedestal *fPedestalPtr[4];
   Bool_t fIsNegativePolarity;
   Double_t fFPNSigmaThreshold;

   AtRawEvent *fRawEventPtr;
   // TClonesArray *fPadArray;

   Int_t fCurrentEventID[4];
   Int_t fTargetFrameID;

   Bool_t fIsSeparatedData;

   std::shared_ptr<AtMap> fMap;
   // Bool_t kEnableAuxChannel;

   Bool_t kDebug;
   std::mutex fRawEventMutex;

public:
   AtCore2();
   AtCore2(std::shared_ptr<AtMap> map);
   AtCore2(std::shared_ptr<AtMap> map, Int_t numCobos = 4);
   AtCore2(TString filename, std::shared_ptr<AtMap> map);
   AtCore2(TString filename, Int_t numTbs, Int_t windowNumTbs = 512, Int_t windowStartTb = 0);
   ~AtCore2();

   void Initialize();

   // setters
   Bool_t AddData(TString filename, Int_t coboIdx = 0);
   void SetPositivePolarity(Bool_t value = kTRUE);
   Bool_t SetData(Int_t value);
   void SetDiscontinuousData(Bool_t value = kTRUE);
   Int_t GetNumData(Int_t coboIdx = 0);
   TString GetDataName(Int_t index, Int_t coboIdx = 0);
   void SetNumTbs(Int_t value);
   void SetFPNPedestal(Double_t sigmaThreshold = 5);
   void SetMap(std::shared_ptr<AtMap> map) { fMap = map; }

   void SetUseSeparatedData(Bool_t value = kTRUE);

   void ProcessCobo(Int_t coboIdx);
   void ProcessBasicCobo(Int_t coboIdx);
   void ProcessLayeredFrame(GETLayeredFrame *layeredFrame);
   void ProcessBasicFrame(GETBasicFrame *basicFrame);

   Bool_t SetWriteFile(TString filename, Int_t coboIdx = 0, Bool_t overwrite = kFALSE);
   void WriteData();

   // getters
   AtRawEvent *GetRawEvent(Long64_t eventID = -1); ///< Returns STRawEvent object filled with the data
   Int_t GetEventID();                             ///< Returns the current event ID
   Int_t GetNumTbs(Int_t coboIdx = 0);             ///< Returns the number of time buckets of the data

   Int_t GetFPNChannel(Int_t chIdx);
   void SetPseudoTopologyFrame(Int_t asadMask, Bool_t check = kFALSE);
   void SetNumCobo(Int_t numCobo);

   ClassDef(AtCore2, 3);
};

#endif
