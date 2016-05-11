/*********************************************************************
*   ATTPC Unpacker and Decoder Core Class	ATCore                   *
*   Author: Y. Ayyad            				                     *
*   Log: 04-03-2015 17:16 JST					                     *
*   Adapted from STCore from SPiRITROOT by G. Jhang                  *
*								                                     *
*********************************************************************/

#ifndef ATCORE_H
#define ATCORE_H

#ifndef __CINT__ // Boost 
#include <boost/multi_array.hpp>
#endif //__CINT__

#include "TObject.h"
#include "TString.h"
#include "GETDecoder.hh"
#include "GETFrame.hh"
#include "AtTpcMap.h"
#include "AtTpcProtoMap.h"
#include "ATPad.hh"
#include "ATRawEvent.hh"


class ATCore : public TObject  {

 public:
    ATCore();
    ATCore(Int_t opt);
    ATCore(TString filename, Int_t numTbs);

    ~ATCore();

     void Initialize();
     Bool_t AddData(TString filename);
     Bool_t SetData(Int_t value);
     Bool_t SetATTPCMap(Char_t const *lookup);
     Bool_t SetProtoGeoFile(TString geofile); // Only for Prototype Map
     Bool_t SetProtoMapFile(TString mapfile);  // Only for Prototype Map   
     void SetPositivePolarity(Bool_t value = kTRUE);
     ATRawEvent *GetRawEvent(Int_t eventID = -1);// TODO It returns a pointer to ATRawEvent
     void SetNumTbs(Int_t value);
     //inline void SetDebugMode(Bool_t Debug){kDebug=Debug;}
     void SetDebugMode(Bool_t Debug);
     void SetInternalPedestal(Int_t startTb = 10, Int_t averageTbs = 20);
     void SetFPNPedestal(Double_t sigmaThreshold = 5);
     
    

     AtTpcMap *fAtMapPtr;

     #ifndef __CINT__
     typedef boost::multi_array<double,3> multiarray;
     typedef multiarray::index index;
     multiarray AtPadCoordArr;
     #endif //__CINT__
    
     Bool_t kDebug;

     enum EPedestalMode { kNoPedestal, kPedestalInternal, kPedestalExternal, kPedestalFPN, kPedestalBothIE };

 private:
    GETDecoder* fGETDecoderPtr;
    Bool_t fIsData;
    Bool_t fIsInternalPedestal;
    Bool_t fIsFPNPedestal;
    Bool_t fIsProtoGeoSet;
    Bool_t fIsProtoMapSet;
    Int_t fNumTbs;
    Int_t fStartTb;
    Int_t fAverageTbs;
    
    ATRawEvent *fRawEventPtr;
    EPedestalMode fPedestalMode;    
    Double_t fFPNSigmaThreshold;
    Double_t fPedestalRMSFactor;
    UInt_t fPrevEventNo;
    UInt_t fCurrEventNo;
    
    Int_t fCurrFrameNo;
    Int_t kOpt;



    ClassDef(ATCore, 1);

    
};

#endif
