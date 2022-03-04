// =================================================
//  AtCore Class
//  Original author : Genie Jhang ( geniejhang@majimak.com )
//  Adapted for AtTPCROOT by Y. Ayyad (ayyadlim@nscl.msu.edu)
// =================================================

#ifndef _ATCORESPECMAT_H_
#define _ATCORESPECMAT_H_

#include "TClonesArray.h"
#include "TFile.h"
#include "TObject.h"
#include "TString.h"
#include "TTree.h"
#include "TTreeReader.h"
#include "TTreeReaderArray.h"

#include "AtMap.h"
#include "AtPedestal.h"
#include "AtRawEvent.h"
#include "AtSpecMATMap.h"
#include "AtTpcMap.h"
#include "AtTpcProtoMap.h"

#include "GETDecoder2.h"

#include <tuple>

class AtCoreSpecMAT : public TObject {
public:
  AtCoreSpecMAT();
  AtCoreSpecMAT(Int_t opt);
  AtCoreSpecMAT(TString filename, Int_t opt);
  AtCoreSpecMAT(TString filename, Int_t numTbs, Int_t windowNumTbs = 512,
                Int_t windowStartTb = 0);
  ~AtCoreSpecMAT();

  void Initialize();

  // setters
  Bool_t AddData(TString filename, Int_t coboIdx = 0);
  void SetPositivePolarity(Bool_t *value);
  Bool_t SetData(Int_t value);
  Int_t GetNumData(Int_t coboIdx = 0);
  TString GetDataName(Int_t index, Int_t coboIdx = 0);
  void SetNumTbs(Int_t value);
  void SetFPNPedestal(Double_t sigmaThreshold = 5);
  Bool_t SetAtTpcMap(Char_t const *lookup);
  Bool_t SetInhibitMaps(TString inimap, TString lowgmap, TString xtalkmap);

  // Bool_t SetGainCalibrationData(TString filename, TString dataType = "f");
  // void SetGainReference(Int_t row, Int_t layer);
  // void SetGainReference(Double_t constant, Double_t linear, Double_t
  // quadratic = 0.);

  // Bool_t SetUAMap(TString filename);
  // Bool_t SetAGETMap(TString filename);

  void GetFPNChannelsFromROOTFILE(Long64_t EventNr);
  void ProcessROOTFILE(Long64_t EventNr);

  Bool_t SetWriteFile(TString filename, Int_t coboIdx = 0,
                      Bool_t overwrite = kFALSE);
  void WriteData();

  // getters
  AtRawEvent *
  GetRawEvent(Long64_t eventID =
                  -1); ///< Returns STRawEvent object filled with the data
  Int_t GetEventID();  ///< Returns the current event ID
  Int_t GetNumTbs(
      Int_t coboIdx = 0); ///< Returns the number of time buckets of the data

  // STMap *GetSTMap();
  // STPlot *GetSTPlot();

  Int_t GetFPNChannel(Int_t chIdx);
  void SetPseudoTopologyFrame(Int_t asadMask, Bool_t check = kFALSE);
  void SetAuxChannel(std::vector<Int_t> AuxCh);
  void SetNumCobo(Int_t numCobo);
  void SetIsPadPlaneCobo(
      Bool_t *IsPadPlane); // Sets whether cobo belongs reads out padplane
                           // signals or scintillator signals

  AtMap *fAtMapPtr;

#ifndef __CINT__
  typedef boost::multi_array<double, 3> multiarray;
  typedef multiarray::index index;
  multiarray AtPadCoordArr;
#endif //__CINT__

  Bool_t kDebug;

private:
  Bool_t GetIsAuxChannel(Int_t val);

  Int_t fNumTbs;

  GETDecoder2 *fDecoderPtr[16];
  Bool_t fIsData;

  AtPedestal *fPedestalPtr[16];
  Bool_t fIsNegativePolarity[4];
  Double_t fFPNSigmaThreshold;

  // STGainCalibration *fGainCalibrationPtr;
  // Bool_t fIsGainCalibrationData;

  AtRawEvent *fRawEventPtr;
  TClonesArray *fPadArray;

  Int_t fCurrentEventID[16];
  Int_t fTargetFrameID;

  Bool_t fIsSeparatedData;
  Int_t kOpt;

  TString fIniMap;
  TString fLowgMap;
  TString fXtalkMap;

  TString fFileName;

  Bool_t kEnableAuxChannel;
  std::vector<Int_t> fAuxChannels;

  Int_t fNumCobo;

  Int_t fFPNChannels[4][4][4][4][512];
  Bool_t fIsPadPlaneCobo[4];

  ClassDef(AtCoreSpecMAT, 1);
};

#endif
