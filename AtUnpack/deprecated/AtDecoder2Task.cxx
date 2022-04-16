#include "AtDecoder2Task.h"

// FAIRROOT classes
#include <FairRootManager.h>
#include <FairRun.h>
#include <FairRunAna.h>
#include <FairRunOnline.h>
#include <FairRuntimeDb.h>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

ClassImp(AtDecoder2Task);

AtDecoder2Task::AtDecoder2Task()
{
   fDecoder = nullptr;
   fDataNum = 0;

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

   fPar = nullptr;
   fRawEventArray = new TClonesArray("AtRawEvent");
   fRawEvent = nullptr;

   fIsSeparatedData = kFALSE;
   fIsPseudoTopology = kFALSE;

   fInternalID = 0;

   fEventID = -1;
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
void AtDecoder2Task::SetPseudoTopologyFrame(Bool_t value)
{
   fIsPseudoTopology = value;
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

   std::cout << " ==== AtDecoder2Task::Init() "
             << "\n";
   std::cout << " Number of Cobo/Asad : " << fNumCobo << "\n";

   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == 0) {
      LOG(error) << "Cannot find RootManager!";

      return kERROR;
   }

   ioMan->Register("AtRawEvent", "AtTPC", fRawEventArray, fIsPersistence);

   fDecoder = new AtCore2(fMap, fNumCobo);
   fDecoder->SetNumCobo(fNumCobo);
   fDecoder->SetUseSeparatedData(fIsSeparatedData);

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

   return kSUCCESS;
}

void AtDecoder2Task::SetParContainers()
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

void AtDecoder2Task::Exec(Option_t *opt)
{
   //#ifdef TASKTIMER
   //  STDebugLogger::Instance() -> TimerStart("DecoderTask");
   //#endif
   fRawEventArray->Delete();

   if (fRawEvent == nullptr)
      fRawEvent = fDecoder->GetRawEvent(fEventID++);
   fInternalID++;
   if (fInternalID % 1 == 0)
      std::cout << " Event Number " << fEventID << " Internal ID : " << fInternalID
                << " Number of Pads : " << fRawEvent->GetNumPads() << std::endl;

   new ((*fRawEventArray)[0]) AtRawEvent(*fRawEvent);

   fRawEvent = nullptr;
   //#ifdef TASKTIMER
   //  STDebugLogger::Instance() -> TimerStop("DecoderTask");
   //#endif
}

Int_t AtDecoder2Task::ReadEvent(Int_t eventID)
{
   fRawEventArray->Delete();

   fRawEvent = fDecoder->GetRawEvent(eventID);
   fEventIDLast = fDecoder->GetEventID();

   if (fRawEvent == nullptr)
      return 1;

   new ((*fRawEventArray)[0]) AtRawEvent(*fRawEvent);

   return 0;
}

void AtDecoder2Task::FinishEvent()
{
   fRawEvent = fDecoder->GetRawEvent();

   if (fRawEvent == nullptr) {
      LOG(info) << "End of file. Terminating FairRun.";
      FairRootManager::Instance()->SetFinishRun();
   }
}
