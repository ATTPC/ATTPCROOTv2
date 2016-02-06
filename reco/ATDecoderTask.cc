#ifdef _OPENMP
#include <omp.h>
#endif

#include "ATDecoderTask.hh"

// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRunAna.h"
#include "FairRuntimeDb.h"

ClassImp(ATDecoderTask);

ATDecoderTask::ATDecoderTask()
{
  fLogger = FairLogger::GetLogger();

  fDecoder = NULL;
  fDataNum = 0;
  fOpt = 0;

  fUseInternalPedestal = kFALSE;
  fStartTb = 3;
 // fAverageTbs = 20;
    fPedestalFile = "";
  //fPedestalRMSFactor = 0;
  fUseFPNPedestal = kFALSE;
  fIsPositive = kFALSE;
  fDebug = kFALSE;
  fFPNPedestalRMS = 5;

  fNumTbs = 512;

  fGetRawEventMode = 0;

 // fGainCalibrationFile = "";
 // fGainConstant = -9999;
 // fGainSlope = -9999;

 // fSignalDelayFile = "";

  fIsPersistence = kFALSE;

  fPar = NULL;
  fRawEventArray = new TClonesArray("ATRawEvent");

  fEventID=0;
}

ATDecoderTask::~ATDecoderTask()
{
}

void ATDecoderTask::SetPersistence(Bool_t value)                           { fIsPersistence = value; }
void ATDecoderTask::SetNumTbs(Int_t numTbs)                                { fNumTbs = numTbs; }
void ATDecoderTask::AddData(TString filename)                              { fDataList.push_back(filename); }
void ATDecoderTask::SetData(Int_t value)                                   { fDataNum = value; }
Bool_t ATDecoderTask::SetMap(Char_t const *map)                            { fMap = map; }
void ATDecoderTask::SetInternalPedestal(Int_t startTb, Int_t averageTbs)   { fUseInternalPedestal = kTRUE; fStartTb = startTb; fAverageTbs = averageTbs; }
void ATDecoderTask::SetFPNPedestal(Double_t pedestalRMS)                   { fUseFPNPedestal = kTRUE; fUseInternalPedestal = kFALSE; fPedestalFile = ""; fFPNPedestalRMS = pedestalRMS;}
void ATDecoderTask::SetPositivePolarity(Bool_t value)                      { fIsPositive = value; }
void ATDecoderTask::SetGeo(TString geofile)				                         { fGeoFile = geofile; }
void ATDecoderTask::SetProtoMap(TString mapfile)	                         { fProtoMapFile = mapfile;}
void ATDecoderTask::SetMapOpt(Int_t value)                                 { fOpt = value; }
void ATDecoderTask::SetDebugMode(Bool_t value)                             { fDebug = value; }
void ATDecoderTask::SetGetRawEventMode(Int_t value)                        { fGetRawEventMode = value; }
//void ATDecoderTask::SetPedestalData(TString filename, Double_t rmsFactor)  { fPedestalFile = filename; fPedestalRMSFactor = rmsFactor; }
//void ATDecoderTask::SetGainCalibrationData(TString filename)               { fGainCalibrationFile = filename; }
//void ATDecoderTask::SetGainBase(Double_t constant, Double_t slope)         { fGainConstant = constant; fGainSlope = slope; }
//void ATDecoderTask::SetSignalDelayData(TString filename)                   { fSignalDelayFile = filename; }

InitStatus
ATDecoderTask::Init()
{
  FairRootManager *ioMan = FairRootManager::Instance();
  if (ioMan == 0) {
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find RootManager!");

    return kERROR;
  }

  ioMan -> Register("ATRawEvent", "ATTPC", fRawEventArray, fIsPersistence);

  fDecoder = new ATCore(fOpt);

  for (Int_t iFile = 0; iFile < fDataList.size(); iFile++)
  fDecoder -> AddData(fDataList.at(iFile));
  fDecoder -> SetData(fDataNum);
  fDecoder -> SetNumTbs(fNumTbs);
  fDecoder -> SetPositivePolarity(fIsPositive);

   if(fDebug)
       fDecoder->SetDebugMode(fDebug);

   if(!fIsPositive) fLogger -> Info(MESSAGE_ORIGIN, "Negative polarity set");
   else fLogger -> Info(MESSAGE_ORIGIN, "Positive polarity set");

  Bool_t kMapIn = fDecoder -> SetATTPCMap(fMap);
  //std::cout<<kMapIn<<std::endl;
   if (!kMapIn) {
      fLogger -> Error(MESSAGE_ORIGIN, "Cannot find ATTPC Map!");

      return kERROR;
    }


  if (fUseInternalPedestal)
     fDecoder -> SetInternalPedestal(fStartTb, fAverageTbs);



 /* if (!fPedestalFile.EqualTo("")) {
    Bool_t isSetPedestalData = fDecoder -> SetPedestalData(fPedestalFile, fPedestalRMSFactor);
    if (!isSetPedestalData) {
      fLogger -> Error(MESSAGE_ORIGIN, "Cannot find pedestal data file!");

      return kERROR;
    }

    fLogger -> Info(MESSAGE_ORIGIN, "Pedestal data is set!");
  }*/

   if (fUseFPNPedestal)
     fDecoder -> SetFPNPedestal();

  /*if (fGainCalibrationFile.EqualTo(""))
    fLogger -> Info(MESSAGE_ORIGIN, "Gain not calibrated!");
  else {
    Bool_t isSetGainCalibrationData = fDecoder -> SetGainCalibrationData(fGainCalibrationFile);
    if (!isSetGainCalibrationData) {
      fLogger -> Error(MESSAGE_ORIGIN, "Cannot find gain calibration data file!");

      return kERROR;
    }

    if (fGainConstant == -9999 || fGainSlope == -9999) {
      fLogger -> Error(MESSAGE_ORIGIN, "Cannot find gain calibration data file!");

      return kERROR;
    }

    fDecoder -> SetGainBase(fGainConstant, fGainSlope);
    fLogger -> Info(MESSAGE_ORIGIN, "Gain calibration data is set!");
  }*/

 /*if (fSignalDelayFile.EqualTo(""))
    fLogger -> Info(MESSAGE_ORIGIN, "Signal not delayed!");
  else {
    Bool_t isSetSignalDelayData = fDecoder -> SetSignalDelayData(fSignalDelayFile);
    if (!isSetSignalDelayData) {
      fLogger -> Error(MESSAGE_ORIGIN, "Cannot find signal delay data file!");

      return kERROR;
    }

    fLogger -> Info(MESSAGE_ORIGIN, "Signal delay data is set!");
  }*/

   // Here we start we different map options
    if(fOpt==1){
      	fDecoder -> SetProtoGeoFile(fGeoFile);
        fDecoder -> SetProtoMapFile(fProtoMapFile);
    }

    if(fGetRawEventMode == 0) fLogger -> Info(MESSAGE_ORIGIN, "Using Normal GetRawEventMode");
    else if (fGetRawEventMode == 1) fLogger -> Info(MESSAGE_ORIGIN, "Using Fast GetRawEventMode");
    else{
		fLogger -> Error(MESSAGE_ORIGIN, "GetRawEvent Mode NOT Set!");
                return kERROR;
	}

  return kSUCCESS;
}

void
ATDecoderTask::SetParContainers()
{
  FairRun *run = FairRun::Instance();
  if (!run)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No analysis run!");

  FairRuntimeDb *db = run -> GetRuntimeDb();
  if (!db)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No runtime database!");

  fPar = (ATDigiPar *) db -> getContainer("ATDigiPar");
  if (!fPar)
    fLogger -> Fatal(MESSAGE_ORIGIN, "Cannot find ATDigiPar!");
}

void
ATDecoderTask::Exec(Option_t *opt)
{
  fRawEventArray -> Delete();

   //ATRawEvent *rawEvent = fDecoder -> GetRawEvent(FairRootManager::Instance()->GetEntryNr());

   //ATRawEvent *rawEvent = new ATRawEvent(); ;



   if(fGetRawEventMode == 0){

    ATRawEvent *rawEvent = fDecoder -> GetRawEvent(fEventID++); //NB: Normal method
    if(rawEvent !=NULL) new ((*fRawEventArray)[0]) ATRawEvent(rawEvent);
    else {
      fLogger -> Info(MESSAGE_ORIGIN, "End of file. Terminating FairRun.");
      FairRootManager::Instance() -> SetFinishRun();
    }

  }else if(fGetRawEventMode ==1){

    ATRawEvent *rawEvent = fDecoder -> GetRawEvent(); // NB:: Method for 10Be data
    //ATRawEvent *rawEvent = fDecoder -> GetRawEvent(fEventID++);
    if(rawEvent !=NULL) new ((*fRawEventArray)[0]) ATRawEvent(rawEvent);
    else {
      fLogger -> Info(MESSAGE_ORIGIN, "End of file. Terminating FairRun.");
      FairRootManager::Instance() -> SetFinishRun();
    }
  }

  //delete rawEvent;

}
