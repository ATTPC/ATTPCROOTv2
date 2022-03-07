#include "AtDecoder2Task.h"

//#include "STGlobal.h"
//#include "STDebugLogger.h"

// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRunOnline.h"
#include "FairRun.h"
#include "FairRunAna.h"
#include "FairRuntimeDb.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(AtDecoder2Task);

AtDecoder2Task::AtDecoder2Task()
{
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
   fIsPositive = kFALSE;

   fPar = NULL;
   fRawEventArray = new TClonesArray("AtRawEvent");
   fRawEvent = NULL;

   fIsSeparatedData = kFALSE;
   fIsPseudoTopology = kFALSE;

   fInternalID = 0;

   fEventID = -1;
   fAuxChannels.clear();
   fNumCobo = 40;

   fMask = 0xF;
}

AtDecoder2Task::~AtDecoder2Task() {}

void AtDecoder2Task::SetPersistence(Bool_t value)
{
   fIsPersistence = value;
}
void AtDecoder2Task::SetNumTbs(Int_t numTbs)
{
   fNumTbs = numTbs;
   fExternalNumTbs = kTRUE;
}
void AtDecoder2Task::AddData(TString filename, Int_t coboIdx)
{
   fDataList[coboIdx].push_back(filename);
}
void AtDecoder2Task::SetData(Int_t value)
{
   fDataNum = value;
}
void AtDecoder2Task::SetFPNPedestal(Double_t pedestalRMS)
{
   fFPNPedestalRMS = pedestalRMS;
}
void AtDecoder2Task::SetPositivePolarity(Bool_t value)
{
   fIsPositive = value;
}
// void AtDecoder2Task::SetUseGainCalibration(Bool_t value)                                       { fUseGainCalibration
// = value; } void AtDecoder2Task::SetGainCalibrationData(TString filename)                                  {
// fGainCalibrationFile = filename; } void AtDecoder2Task::SetGainReference(Double_t constant, Double_t linear, Double_t
// quadratic)  { fGainConstant = constant; fGainLinear = linear; fGainQuadratic = quadratic; }
void AtDecoder2Task::SetUseSeparatedData(Bool_t value)
{
   fIsSeparatedData = value;
}
void AtDecoder2Task::SetEventID(Long64_t eventid)
{
   fEventID = eventid;
}
void AtDecoder2Task::SetGeo(TString geofile)
{
   fGeoFile = geofile;
}
void AtDecoder2Task::SetProtoMap(TString mapfile)
{
   fProtoMapFile = mapfile;
}
void AtDecoder2Task::SetMapOpt(Int_t value)
{
   fOpt = value;
}
Bool_t AtDecoder2Task::SetMap(Char_t const *map)
{
   fMap = map;
}
void AtDecoder2Task::SetPseudoTopologyFrame(Bool_t value)
{
   fIsPseudoTopology = value;
}
void AtDecoder2Task::SetInhibitMaps(TString inimap, TString lowgmap, TString xtalkmap)
{
   fIniMap = inimap;
   fLowgMap = lowgmap;
   fXtalkMap = xtalkmap;
}

void AtDecoder2Task::SetAuxChannels(std::vector<Int_t> AuxCh)
{
   fAuxChannels = AuxCh;
}
void AtDecoder2Task::SetNumCobo(Int_t numCobo)
{
   fNumCobo = numCobo;
}
void AtDecoder2Task::SetPTFMask(Int_t mask)
{
   fMask = mask;
}

Long64_t AtDecoder2Task::GetEventID()
{
   return fEventIDLast;
}

InitStatus AtDecoder2Task::Init()
{

  std::cout<<" ==== AtDecoder2Task::Init() "<<"\n";
  std::cout<<" Number of Cobo/Asad : "<<fNumCobo<<"\n";
  std::cout<<" Map option : "<<fOpt<<"\n";
  
  FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == 0) {
      fLogger->Error(MESSAGE_ORIGIN, "Cannot find RootManager!");

      return kERROR;
   }

   ioMan->Register("AtRawEvent", "AtTPC", fRawEventArray, fIsPersistence);

   fDecoder = new AtCore2(fOpt,fNumCobo);
   fDecoder->SetNumCobo(fNumCobo);
   fDecoder->SetUseSeparatedData(fIsSeparatedData);
   fDecoder->SetInhibitMaps(fIniMap, fLowgMap, fXtalkMap);
   

   for (Int_t iFile = 0; iFile < fDataList[0].size(); iFile++)
      fDecoder->AddData(fDataList[0].at(iFile));

   if (fIsSeparatedData)
      for (Int_t iCobo = 1; iCobo < fNumCobo; iCobo++)
         for (Int_t iFile = 0; iFile < fDataList[iCobo].size(); iFile++)
            fDecoder->AddData(fDataList[iCobo].at(iFile), iCobo);

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
   
   if (!kMapIn) {
      fLogger->Error(MESSAGE_ORIGIN, "Cannot find AtTPC Map!");

      return kERROR;
   }

   if (fOpt == 1) {
      fDecoder->SetProtoGeoFile(fGeoFile);
      fDecoder->SetProtoMapFile(fProtoMapFile);
      if (fAuxChannels.size() > 0)
         fDecoder->SetAuxChannel(fAuxChannels);
   }

   
   return kSUCCESS;
}

void AtDecoder2Task::SetParContainers()
{
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

void AtDecoder2Task::Exec(Option_t *opt)
{
   //#ifdef TASKTIMER
   //  STDebugLogger::Instance() -> TimerStart("DecoderTask");
   //#endif
   fRawEventArray->Delete();

   if (fRawEvent == NULL)
      fRawEvent = fDecoder->GetRawEvent(fEventID++);
   fInternalID++;
   if (fInternalID % 1 == 0)
      std::cout << " Event Number " << fEventID << " Internal ID : " << fInternalID
                << " Number of Pads : " << fRawEvent->GetNumPads() << std::endl;

   new ((*fRawEventArray)[0]) AtRawEvent(fRawEvent);

   fRawEvent = NULL;
   //#ifdef TASKTIMER
   //  STDebugLogger::Instance() -> TimerStop("DecoderTask");
   //#endif
}

Int_t AtDecoder2Task::ReadEvent(Int_t eventID)
{
   fRawEventArray->Delete();

   fRawEvent = fDecoder->GetRawEvent(eventID);
   fEventIDLast = fDecoder->GetEventID();

   if (fRawEvent == NULL)
      return 1;

   new ((*fRawEventArray)[0]) AtRawEvent(fRawEvent);

   return 0;
}

void AtDecoder2Task::FinishEvent()
{
   fRawEvent = fDecoder->GetRawEvent();

   if (fRawEvent == NULL) {
      fLogger->Info(MESSAGE_ORIGIN, "End of file. Terminating FairRun.");
      FairRootManager::Instance()->SetFinishRun();
   }
}
