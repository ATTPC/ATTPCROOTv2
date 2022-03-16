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

AtDecoderSpecMATTask::AtDecoderSpecMATTask()
{
   fLogger = FairLogger::GetLogger();

   fDecoder = nullptr;
   fDataNum = 0;
   // fOpt = 0;

   fFPNPedestalRMS = -1;

   fExternalNumTbs = kFALSE;
   fNumTbs = 512;

   // fUseGainCalibration = kFALSE;
   // fGainCalibrationFile = "";
   // fGainConstant = -9999;
   // fGainLinear = -9999;
   // fGainQuadratic = 0;

   fIsPersistence = kFALSE;

   fPar = nullptr;
   fRawEventArray = new TClonesArray("AtRawEvent");
   fRawEvent = nullptr;

   fIsSeparatedData = kFALSE;
   fIsPseudoTopology = kFALSE;

   fInternalID = 1;
   fEventID = 0; // For SpecMAT data events start from 1 not 0

   // fAuxChannels.clear();
   fNumCobo = 4;

   for (Int_t i = 0; i < fNumCobo; i++) {
      fIsCoboPadPlane[i] = kTRUE;
      fIsPositive[i] = kTRUE;
   }

   fMask = 0xF;
}

void AtDecoderSpecMATTask::SetPersistence(Bool_t value)
{
   fIsPersistence = value;
}
void AtDecoderSpecMATTask::SetNumTbs(Int_t numTbs)
{
   fNumTbs = numTbs;
   fExternalNumTbs = kTRUE;
}
void AtDecoderSpecMATTask::AddData(TString filename, Int_t coboIdx)
{
   fDataList[coboIdx].push_back(filename);
}
void AtDecoderSpecMATTask::SetData(Int_t value)
{
   fDataNum = value;
}
void AtDecoderSpecMATTask::SetFPNPedestal(Double_t pedestalRMS)
{
   fFPNPedestalRMS = pedestalRMS;
}
void AtDecoderSpecMATTask::SetPositivePolarity(Bool_t *value)
{
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
void AtDecoderSpecMATTask::SetUseSeparatedData(Bool_t value)
{
   fIsSeparatedData = value;
}
void AtDecoderSpecMATTask::SetEventID(Long64_t eventid)
{
   fEventID = eventid;
}
/* Old map style
void AtDecoderSpecMATTask::SetGeo(TString geofile)
{
   fGeoFile = geofile;
}
void AtDecoderSpecMATTask::SetProtoMap(TString mapfile)
{
   fProtoMapFile = mapfile;
}
void AtDecoderSpecMATTask::SetMapOpt(Int_t value)
{
   fOpt = value;
}
void AtDecoderSpecMATTask::SetMap(Char_t const *map)
{
   fMap = map;
}
void AtDecoderSpecMATTask::SetInhibitMaps(TString inimap, TString lowgmap, TString xtalkmap)
{
   fIniMap = inimap;
   fLowgMap = lowgmap;
   fXtalkMap = xtalkmap;
}

void AtDecoderSpecMATTask::SetAuxChannels(std::vector<Int_t> AuxCh)
{
   fAuxChannels = AuxCh;
}
*/
void AtDecoderSpecMATTask::SetPseudoTopologyFrame(Bool_t value)
{
   fIsPseudoTopology = value;
}

void AtDecoderSpecMATTask::SetNumCobo(Int_t numCobo)
{
   fNumCobo = numCobo;
}

void AtDecoderSpecMATTask::SetIsCoboPadPlane(Bool_t *IsPadPlane)
{
   for (Int_t i = 0; i < fNumCobo; i++) {
      fIsCoboPadPlane[i] = IsPadPlane[i];
   }
}
void AtDecoderSpecMATTask::SetPTFMask(Int_t mask)
{
   fMask = mask;
}

Long64_t AtDecoderSpecMATTask::GetEventID()
{
   return fEventIDLast;
}

InitStatus AtDecoderSpecMATTask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == 0) {
      LOG(error) << "Cannot find RootManager!";

      return kERROR;
   }

   /* old map style
   fDecoder = new AtCoreSpecMAT(fOpt);
   fDecoder->SetInhibitMaps(fIniMap, fLowgMap, fXtalkMap);
   */
   // new map style
   fDecoder = std::make_unique<AtCoreSpecMAT>(fMap);
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

   /* old map style
     Bool_t kMapIn = fDecoder->SetAtTpcMap(fMap);

   if (!kMapIn) {
      LOG(error) << "Cannot find SpecMAT Map!";

      return kERROR;
   }


   if (fAuxChannels.size() > 0) {
      fDecoder->SetAuxChannel(fAuxChannels);
   }
   */
   // new map style
   if (fMap == nullptr) {
      LOG(error) << "SpecMAT Map was never set!";
      return kERROR;
   } else
      fDecoder->SetMap(fMap);

   ioMan->Register("AtRawEvent", "AtTPC", fRawEventArray, fIsPersistence);
   return kSUCCESS;
}

void AtDecoderSpecMATTask::SetParContainers()
{
   FairRun *run = FairRun::Instance();
   if (!run)
      LOG(fatal) << "No analysis run!";

   FairRuntimeDb *db = run->GetRuntimeDb();
   if (!db)
      LOG(fatal) << "No runtime database!";

   fPar = (AtDigiPar *)db->getContainer("AtDigiPar");
   if (!fPar)
      LOG(fatal) << "Cannot find AtDigiPar!";
}

void AtDecoderSpecMATTask::Exec(Option_t *opt)
{
   //#ifdef TASKTIMER
   //  STDebugLogger::Instance() -> TimerStart("DecoderTask");
   //#endif
   fRawEventArray->Delete();
   std::cout << "Start of SpecMAT decoder task for event" << fEventID + 1 << std::endl;
   if (fRawEvent == nullptr)
      fRawEvent = fDecoder->GetRawEvent(fEventID++);
   fInternalID++;
   // if (fInternalID % 100 == 0)
   std::cout << " Event Number " << fEventID << " Internal ID : " << fInternalID
             << " Number of Pads : " << fRawEvent->GetNumPads() << std::endl;

   new ((*fRawEventArray)[0]) AtRawEvent(*fRawEvent);

   fRawEvent = nullptr;
   //#ifdef TASKTIMER
   //  STDebugLogger::Instance() -> TimerStop("DecoderTask");
   //#endif
}

Int_t AtDecoderSpecMATTask::ReadEvent(Int_t eventID)
{
   fRawEventArray->Delete();

   fRawEvent = fDecoder->GetRawEvent(eventID);
   fEventIDLast = fDecoder->GetEventID();

   if (fRawEvent == nullptr)
      return 1;

   new ((*fRawEventArray)[0]) AtRawEvent(*fRawEvent);

   return 0;
}

void AtDecoderSpecMATTask::FinishEvent()
{
   fRawEvent = fDecoder->GetRawEvent();

   if (fRawEvent == nullptr) {
      LOG(info) << "End of file. Terminating FairRun.";
      FairRootManager::Instance()->SetFinishRun();
   }
}
