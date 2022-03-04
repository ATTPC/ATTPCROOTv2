#include "AtDecoderSpecMATTask.h"

// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRunAna.h"
#include "FairRunOnline.h"
#include "FairRuntimeDb.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(AtDecoderSpecMATTask);

AtDecoderSpecMATTask::AtDecoderSpecMATTask() {
  fLogger = FairLogger::GetLogger();

  fDecoder = NULL;
  fDataNum = 0;
  fOpt = 0;

  fFPNPedestalRMS = -1;

  fExternalNumTbs = kFALSE;
  fNumTbs = 512;

  // fUseGainCalibration = kFALSE;
  // fGainCalibrationFile = "";
  // fGainConstant = -9999;
  // fGainLinear = -9999;
  // fGainQuadratic = 0;

  fIsPersistence = kFALSE;

  fPar = NULL;
  fRawEventArray = new TClonesArray("AtRawEvent");
  fRawEvent = NULL;

  fIsSeparatedData = kFALSE;
  fIsPseudoTopology = kFALSE;

  fInternalID = 1;
  fEventID = 0; // For SpecMAT data events start from 1 not 0

  fAuxChannels.clear();
  fNumCobo = 4;

  for (Int_t i = 0; i < fNumCobo; i++) {
    fIsCoboPadPlane[i] = kTRUE;
    fIsPositive[i] = kTRUE;
  }

  fMask = 0xF;
}

AtDecoderSpecMATTask::~AtDecoderSpecMATTask() {}

void AtDecoderSpecMATTask::SetPersistence(Bool_t value) {
  fIsPersistence = value;
}
void AtDecoderSpecMATTask::SetNumTbs(Int_t numTbs) {
  fNumTbs = numTbs;
  fExternalNumTbs = kTRUE;
}
void AtDecoderSpecMATTask::AddData(TString filename, Int_t coboIdx) {
  fDataList[coboIdx].push_back(filename);
}
void AtDecoderSpecMATTask::SetData(Int_t value) { fDataNum = value; }
void AtDecoderSpecMATTask::SetFPNPedestal(Double_t pedestalRMS) {
  fFPNPedestalRMS = pedestalRMS;
}
void AtDecoderSpecMATTask::SetPositivePolarity(Bool_t *value) {
  for (Int_t i = 0; i < fNumCobo; i++) {
    fIsPositive[i] = value[i];
  }
}
// void AtDecoderSpecMATTask::SetUseGainCalibration(Bool_t value) {
// fUseGainCalibration = value; } void
// AtDecoderSpecMATTask::SetGainCalibrationData(TString filename) {
// fGainCalibrationFile = filename; } void
// AtDecoderSpecMATTask::SetGainReference(Double_t constant, Double_t linear,
// Double_t quadratic)  { fGainConstant = constant; fGainLinear = linear;
// fGainQuadratic = quadratic; }
void AtDecoderSpecMATTask::SetUseSeparatedData(Bool_t value) {
  fIsSeparatedData = value;
}
void AtDecoderSpecMATTask::SetEventID(Long64_t eventid) { fEventID = eventid; }
void AtDecoderSpecMATTask::SetGeo(TString geofile) { fGeoFile = geofile; }
void AtDecoderSpecMATTask::SetProtoMap(TString mapfile) {
  fProtoMapFile = mapfile;
}
void AtDecoderSpecMATTask::SetMapOpt(Int_t value) { fOpt = value; }
Bool_t AtDecoderSpecMATTask::SetMap(Char_t const *map) { fMap = map; }
void AtDecoderSpecMATTask::SetPseudoTopologyFrame(Bool_t value) {
  fIsPseudoTopology = value;
}
void AtDecoderSpecMATTask::SetInhibitMaps(TString inimap, TString lowgmap,
                                          TString xtalkmap) {
  fIniMap = inimap;
  fLowgMap = lowgmap;
  fXtalkMap = xtalkmap;
}

void AtDecoderSpecMATTask::SetAuxChannels(std::vector<Int_t> AuxCh) {
  fAuxChannels = AuxCh;
}
void AtDecoderSpecMATTask::SetNumCobo(Int_t numCobo) { fNumCobo = numCobo; }

void AtDecoderSpecMATTask::SetIsCoboPadPlane(Bool_t *IsPadPlane) {
  for (Int_t i = 0; i < fNumCobo; i++) {
    fIsCoboPadPlane[i] = IsPadPlane[i];
  }
}
void AtDecoderSpecMATTask::SetPTFMask(Int_t mask) { fMask = mask; }

Long64_t AtDecoderSpecMATTask::GetEventID() { return fEventIDLast; }

InitStatus AtDecoderSpecMATTask::Init() {
  FairRootManager *ioMan = FairRootManager::Instance();
  if (ioMan == 0) {
    fLogger->Error(MESSAGE_ORIGIN, "Cannot find RootManager!");

    return kERROR;
  }

  fDecoder = new AtCoreSpecMAT(fOpt);
  fDecoder->SetInhibitMaps(fIniMap, fLowgMap, fXtalkMap);
  fDecoder->SetNumCobo(fNumCobo);
  fDecoder->SetIsPadPlaneCobo(fIsCoboPadPlane);
  fDecoder->SetPositivePolarity(fIsPositive);

  for (Int_t iFile = 0; iFile < fDataList[0].size(); iFile++)
    fDecoder->AddData(fDataList[0].at(iFile));

  if (fIsPseudoTopology)
    fDecoder->SetPseudoTopologyFrame(fMask, kFALSE);

  fDecoder->SetData(fDataNum);

  if (fExternalNumTbs)
    fDecoder->SetNumTbs(fNumTbs);
  else
    fDecoder->SetNumTbs(fPar->GetNumTbs());

  if (fFPNPedestalRMS == -1)
    // fFPNPedestalRMS = fPar -> GetFPNPedestalRMS();
    fFPNPedestalRMS = 5;

  fDecoder->SetFPNPedestal(fFPNPedestalRMS);

  Bool_t kMapIn = fDecoder->SetAtTpcMap(fMap);
  // std::cout<<kMapIn<<std::endl;
  if (!kMapIn) {
    fLogger->Error(MESSAGE_ORIGIN, "Cannot find SpecMAT Map!");

    return kERROR;
  }

  if (fAuxChannels.size() > 0) {
    fDecoder->SetAuxChannel(fAuxChannels);
  }

  /*  if (fGainCalibrationFile.EqualTo("") && fUseGainCalibration == kFALSE)
      fLogger -> Info(MESSAGE_ORIGIN, "Gain not calibrated!");
    else if (fGainCalibrationFile.EqualTo("") && fUseGainCalibration == kTRUE) {
      Bool_t isSetGainCalibrationData = fDecoder -> SetGainCalibrationData(fPar
    -> GetGainCalibrationDataFileName()); if (!isSetGainCalibrationData) {
        fLogger -> Error(MESSAGE_ORIGIN, "Cannot find gain calibration data
    file!");

        return kERROR;
      }*/
  //  LOG(INFO) << fPar -> GetGainCalibrationDataFileName() << " " << fPar ->
  //  GetGCConstant() << " " << fPar -> GetGCLinear() << " " << fPar ->
  //  GetGCQuadratic() << FairLogger::endl;

  /*  fDecoder -> SetGainReference(fPar -> GetGCConstant(), fPar ->
  GetGCLinear(), fPar -> GetGCQuadratic()); fLogger -> Info(MESSAGE_ORIGIN,
  "Gain calibration data is set from parameter list!"); } else { Bool_t
  isSetGainCalibrationData = fDecoder ->
  SetGainCalibrationData(fGainCalibrationFile); if (!isSetGainCalibrationData) {
      fLogger -> Error(MESSAGE_ORIGIN, "Cannot find gain calibration data
  file!");

      return kERROR;
    }

    if (fGainConstant == -9999 || fGainLinear == -9999) {
      fLogger -> Error(MESSAGE_ORIGIN, "Cannot find gain calibration data
  file!");

      return kERROR;
    }

    fDecoder -> SetGainReference(fGainConstant, fGainLinear, fGainQuadratic);
    fLogger -> Info(MESSAGE_ORIGIN, "Gain calibration data is set!");
  }*/

  ioMan->Register("AtRawEvent", "AtTPC", fRawEventArray, fIsPersistence);
  return kSUCCESS;
}

void AtDecoderSpecMATTask::SetParContainers() {
  FairRun *run = FairRun::Instance();
  if (!run)
    fLogger->Fatal(MESSAGE_ORIGIN, "No analysis run!");

  FairRuntimeDb *db = run->GetRuntimeDb();
  if (!db)
    fLogger->Fatal(MESSAGE_ORIGIN, "No runtime database!");

  fPar = (AtDigiPar *)db->getContainer("AtDigiPar");
  if (!fPar)
    fLogger->Fatal(MESSAGE_ORIGIN, "Cannot find AtDigiPar!");
}

void AtDecoderSpecMATTask::Exec(Option_t *opt) {
  //#ifdef TASKTIMER
  //  STDebugLogger::Instance() -> TimerStart("DecoderTask");
  //#endif
  fRawEventArray->Delete();
  std::cout << "Start of SpecMAT decoder task for event" << fEventID + 1
            << std::endl;
  if (fRawEvent == NULL)
    fRawEvent = fDecoder->GetRawEvent(fEventID++);
  fInternalID++;
  // if (fInternalID % 100 == 0)
  std::cout << " Event Number " << fEventID << " Internal ID : " << fInternalID
            << " Number of Pads : " << fRawEvent->GetNumPads() << std::endl;

  new ((*fRawEventArray)[0]) AtRawEvent(fRawEvent);

  fRawEvent = NULL;
  //#ifdef TASKTIMER
  //  STDebugLogger::Instance() -> TimerStop("DecoderTask");
  //#endif
}

Int_t AtDecoderSpecMATTask::ReadEvent(Int_t eventID) {
  fRawEventArray->Delete();

  fRawEvent = fDecoder->GetRawEvent(eventID);
  fEventIDLast = fDecoder->GetEventID();

  if (fRawEvent == NULL)
    return 1;

  new ((*fRawEventArray)[0]) AtRawEvent(fRawEvent);

  return 0;
}

void AtDecoderSpecMATTask::FinishEvent() {
  fRawEvent = fDecoder->GetRawEvent();

  if (fRawEvent == NULL) {
    fLogger->Info(MESSAGE_ORIGIN, "End of file. Terminating FairRun.");
    FairRootManager::Instance()->SetFinishRun();
  }
}
