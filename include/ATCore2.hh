// =================================================
//  ATCore Class
//  Original author : Genie Jhang ( geniejhang@majimak.com )
//  Adapted for ATTPCROOT by Y. Ayyad (ayyadlim@nscl.msu.edu)
// =================================================

#ifndef _ATCORE2_H_
#define _ATCORE2_H_

#include "TObject.h"
#include "TString.h"
#include "TClonesArray.h"

#include "ATRawEvent.hh"
#include "AtTpcMap.h"
#include "AtTpcProtoMap.h"
#include "ATPedestal.hh"
//#include "STGainCalibration.hh"
//#include "STPlot.hh"

#include "GETDecoder2.hh"

#include <tuple>

//class STPlot;

class ATCore2 : public TObject {
  public:
    ATCore2();
    ATCore2(Int_t opt);
    ATCore2(TString filename,Int_t opt);
    ATCore2(TString filename, Int_t numTbs, Int_t windowNumTbs = 512, Int_t windowStartTb = 0);
    ~ATCore2();

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
    Bool_t SetATTPCMap(Char_t const *lookup);
    Bool_t SetProtoGeoFile(TString geofile); // Only for Prototype Map
    Bool_t SetProtoMapFile(TString mapfile);  // Only for Prototype Map
    Bool_t SetInhibitMaps(TString inimap, TString lowgmap, TString xtalkmap);

    //Bool_t SetGainCalibrationData(TString filename, TString dataType = "f");
    //void SetGainReference(Int_t row, Int_t layer);
    //void SetGainReference(Double_t constant, Double_t linear, Double_t quadratic = 0.);

    //Bool_t SetUAMap(TString filename);
    //Bool_t SetAGETMap(TString filename);

    void SetUseSeparatedData(Bool_t value = kTRUE);

    void ProcessCobo(Int_t coboIdx);
    void ProcessLayeredFrame(GETLayeredFrame *layeredFrame);
    void ProcessBasicFrame(GETBasicFrame *basicFrame);

    Bool_t SetWriteFile(TString filename, Int_t coboIdx = 0, Bool_t overwrite = kFALSE);
    void WriteData();

    // getters
    ATRawEvent *GetRawEvent(Long64_t eventID = -1);       ///< Returns STRawEvent object filled with the data
    Int_t GetEventID();                                   ///< Returns the current event ID
    Int_t GetNumTbs(Int_t coboIdx = 0);                   ///< Returns the number of time buckets of the data

    //STMap *GetSTMap();
    //STPlot *GetSTPlot();

    Int_t GetFPNChannel(Int_t chIdx);
    void SetPseudoTopologyFrame(Int_t asadMask, Bool_t check = kFALSE);

    AtTpcMap *fAtMapPtr;

    #ifndef __CINT__
    typedef boost::multi_array<double,3> multiarray;
    typedef multiarray::index index;
    multiarray AtPadCoordArr;
    #endif //__CINT__

    Bool_t kDebug;

  private:

    //STPlot *fPlotPtr;

    Int_t fNumTbs;

    GETDecoder2 *fDecoderPtr[10];
    Bool_t fIsData;

    ATPedestal *fPedestalPtr[10];
    Bool_t fIsNegativePolarity;
    Double_t fFPNSigmaThreshold;
    Bool_t fIsProtoGeoSet;
    Bool_t fIsProtoMapSet;

    //STGainCalibration *fGainCalibrationPtr;
    //Bool_t fIsGainCalibrationData;

    ATRawEvent *fRawEventPtr;
    TClonesArray *fPadArray;

    Int_t fCurrentEventID[12];
    Int_t fTargetFrameID;

    Bool_t fIsSeparatedData;
    Int_t kOpt;

    TString fIniMap;
    TString fLowgMap;
    TString fXtalkMap;

  ClassDef(ATCore2, 1);
};

#endif
