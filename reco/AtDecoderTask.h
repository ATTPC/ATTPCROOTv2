#ifndef AtDECODERTASK_H_
#define AtDECODERTASK_H_


#include "FairTask.h"
#include "FairLogger.h"


#include "AtCore.h"
#include "AtTpcMap.h"
#include "AtRawEvent.h"

#include "AtDigiPar.h"


#include "TClonesArray.h"
#include "TString.h"


#include <vector>

using std::vector;


class AtDecoderTask : public FairTask {
  public:
    /// Constructor
    AtDecoderTask();
    /// Destructor
    ~AtDecoderTask();


    void SetNumTbs(Int_t numTbs);

    void AddData(TString filename);

    void SetData(Int_t value);

    void SetProtoMap(TString mapfile);//only for prototype

    Bool_t SetMap(Char_t const *map);

    void SetGeo(TString geofile); //only for prototype

    void SetMapOpt(Int_t value);

    void SetInternalPedestal(Int_t startTb = 3, Int_t averageTbs = 20);

 //   void SetPedestalData(TString filename, Double_t rmsFactor = 0);

    void SetFPNPedestal(Double_t rms = 5);

    void SetPersistence(Bool_t value = kTRUE);

    void SetPositivePolarity(Bool_t value = kFALSE);

    void SetDebugMode(Bool_t value);

    void SetGetRawEventMode(Int_t value);

    /// Initializing the task. This will be called when Init() method invoked from FairRun.
    virtual InitStatus Init();

    virtual void SetParContainers();

    virtual void Exec(Option_t *opt);

  private:
    FairLogger *fLogger;

    AtCore *fDecoder;

    vector<TString> fDataList;
    Int_t fDataNum;

      Bool_t fUseInternalPedestal;
      Int_t fStartTb;
      Int_t fAverageTbs;
      TString fPedestalFile;
      TString fGeoFile;
      TString fProtoMapFile;
  //  Double_t fPedestalRMSFactor;
      Bool_t fUseFPNPedestal;
      Bool_t fIsPositive;
      Bool_t fDebug;
      Double_t fFPNPedestalRMS;     /// RMS cut of baseline matching part selection
      Int_t fGetRawEventMode;

   // TString fGainCalibrationFile;
  //  Double_t fGainConstant;
  //  Double_t fGainSlope;

  //  TString fSignalDelayFile;
    Int_t fNumTbs;

    Char_t const *fMap;

    Bool_t fIsPersistence;
    Int_t fOpt;

    AtDigiPar *fPar;
    TClonesArray *fRawEventArray;

    Int_t fEventID;

  ClassDef(AtDecoderTask, 1);
};

#endif
