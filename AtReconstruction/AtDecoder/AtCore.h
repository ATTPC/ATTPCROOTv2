/*********************************************************************
 *   AtTPC Unpacker and Decoder Core Class	AtCore                   *
 *   Author: Y. Ayyad            				                     *
 *   Log: 04-03-2015 17:16 JST					                     *
 *   Adapted from STCore from SPiRITROOT by G. Jhang                  *
 *								                                     *
 *********************************************************************/

#ifndef AtCORE_H
#define AtCORE_H

#ifndef __CINT__ // Boost
#include <boost/multi_array.hpp>
#endif //__CINT__

#include "TObject.h"
#include "TString.h"
#include "GETDecoder.h"
#include "GETFrame.h"
#include "AtTpcMap.h"
#include "AtTpcProtoMap.h"
#include "AtPad.h"
#include "AtRawEvent.h"

class AtCore : public TObject {

public:
   AtCore();
   AtCore(Int_t opt);
   AtCore(TString filename, Int_t numTbs);

   ~AtCore();

   void Initialize();
   Bool_t AddData(TString filename);
   Bool_t SetData(Int_t value);
   Bool_t SetAtTPCMap(Char_t const *lookup);
   Bool_t SetProtoGeoFile(TString geofile); // Only for Prototype Map
   Bool_t SetProtoMapFile(TString mapfile); // Only for Prototype Map
   void SetPositivePolarity(Bool_t value = kTRUE);
   AtRawEvent *GetRawEvent(Int_t eventID = -1); // TODO It returns a pointer to AtRawEvent
   void SetNumTbs(Int_t value);
   // inline void SetDebugMode(Bool_t Debug){kDebug=Debug;}
   void SetDebugMode(Bool_t Debug);
   void SetInternalPedestal(Int_t startTb = 10, Int_t averageTbs = 20);
   void SetFPNPedestal(Double_t sigmaThreshold = 5);

   AtTpcMap *fAtMapPtr;

#ifndef __CINT__
   typedef boost::multi_array<double, 3> multiarray;
   typedef multiarray::index index;
   multiarray AtPadCoordArr;
#endif //__CINT__

   Bool_t kDebug;

   enum EPedestalMode { kNoPedestal, kPedestalInternal, kPedestalExternal, kPedestalFPN, kPedestalBothIE };

private:
   GETDecoder *fGETDecoderPtr;
   Bool_t fIsData;
   Bool_t fIsInternalPedestal;
   Bool_t fIsFPNPedestal;
   Bool_t fIsProtoGeoSet;
   Bool_t fIsProtoMapSet;
   Int_t fNumTbs;
   Int_t fStartTb;
   Int_t fAverageTbs;

   AtRawEvent *fRawEventPtr;
   EPedestalMode fPedestalMode;
   Double_t fFPNSigmaThreshold;
   Double_t fPedestalRMSFactor;
   UInt_t fPrevEventNo;
   UInt_t fCurrEventNo;

   Int_t fCurrFrameNo;
   Int_t kOpt;

   ClassDef(AtCore, 1);
};

#endif
