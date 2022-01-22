#include "AtHDFParserTask.h"

#include "FairRootManager.h"
#include "FairRunOnline.h"
#include "FairRun.h"
#include "FairRunAna.h"
#include "FairRuntimeDb.h"
#include "FairLogger.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(AtHDFParserTask);

AtHDFParserTask::AtHDFParserTask()
{
   fIsPersistence = kFALSE;
   fRawEventArray = new TClonesArray("AtRawEvent");
   fEventID = 0;
   fIniEventID = 0;
   fNumberTimestamps = 1;
   fRawEvent = new AtRawEvent();
   fIsOldFormat = kFALSE;
   fIsBaseLineSubtraction = kFALSE;
}

AtHDFParserTask::~AtHDFParserTask()
{
   delete fRawEventArray;
   delete fRawEvent;
}

InitStatus AtHDFParserTask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == 0) {
      LOG(error) << "Cannot find RootManager!";
      return kERROR;
   }

   // Need try-catch
   HDFParser = new AtHDFParser();
   fNumEvents = HDFParser->open(fFileName.c_str());
   std::cout << " Number of events : " << fNumEvents << std::endl;

   auto numUniqueEvents = HDFParser->getFirstEvent() - HDFParser->getLastEvent();

   if (fIniEventID > numUniqueEvents) {
      LOG(fatal) << "Exceeded the valid range of event numbers";
      return kERROR;
   } else
      fEventID = fIniEventID + HDFParser->getFirstEvent();

   ioMan->Register("AtRawEvent", "AtTPC", fRawEventArray, fIsPersistence);
   return kSUCCESS;
}

void AtHDFParserTask::SetParContainers()
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

void AtHDFParserTask::processHeader()
{
   TString header_name = TString::Format("evt%lu_header", fEventID);
   auto header = HDFParser->get_header(header_name.Data());

   fRawEvent->SetEventID(header.at(0));

   fRawEvent->SetNumberOfTimestamps(fNumberTimestamps);
   for (int i = 0; i < fNumberTimestamps; ++i)
      fRawEvent->SetTimestamp(header.at(i + 1), i);
}

void AtHDFParserTask::processData()
{
   TString event_name = TString::Format("evt%lu_data", fEventID);
   std::size_t npads = HDFParser->n_pads(event_name.Data());

   for (auto ipad = 0; ipad < npads; ++ipad)
      processPad(ipad);
}

void AtHDFParserTask::processPad(std::size_t ipad)
{
   std::vector<int16_t> rawadc = HDFParser->pad_raw_data(ipad);
   PadReference PadRef = {rawadc[0], rawadc[1], rawadc[2], rawadc[3]};

   auto &pad = createPadAndSetIsAux(PadRef);
   setDimensions(pad);
   setAdc(pad, rawadc);

   fRawEvent->SetIsGood(kTRUE);
}
AtPad &AtHDFParserTask::createPadAndSetIsAux(const PadReference &padRef)
{
   if (fAtMapPtr->IsAuxPad(padRef)) {
      return fRawEvent->AddAuxPad(fAtMapPtr->GetAuxName(padRef)).first->second;
   } else {
      auto padNumber = fAtMapPtr->GetPadNum(padRef);
      return fRawEvent->AddPad(padNumber);
   }
}
void AtHDFParserTask::setAdc(AtPad &pad, const std::vector<int16_t> &data)
{
   auto baseline = getBaseline(data);
   for (Int_t iTb = 0; iTb < 512; iTb++) {
      pad.SetRawADC(iTb, data.at(iTb + 5));
      pad.SetADC(iTb, data.at(iTb + 5) - baseline);
   }
   pad.SetPedestalSubtracted(fIsBaseLineSubtraction);
}

Float_t AtHDFParserTask::getBaseline(const std::vector<int16_t> &data)
{
   Float_t baseline = 0;

   if (fIsBaseLineSubtraction) {
      for (Int_t iTb = 5; iTb < 25; iTb++)
         baseline += data[iTb];
      baseline /= 20.0;
   }
   return baseline;
}
void AtHDFParserTask::setDimensions(AtPad &pad)
{
   auto PadCenterCoord = fAtMapPtr->CalcPadCenter(pad.GetPadNum());
   Int_t pSizeID = fAtMapPtr->GetPadSize(pad.GetPadNum());
   pad.SetPadXCoord(PadCenterCoord[0]);
   pad.SetPadYCoord(PadCenterCoord[1]);
   pad.SetSizeID(pSizeID);
}
void AtHDFParserTask::Exec(Option_t *opt)
{
   fRawEventArray->Delete();
   fRawEvent->Clear();

   if (fEventID > HDFParser->getLastEvent()) {
      LOG(fatal) << "Tried to unpack an event that was too large!";
      return;
   }

   processHeader();
   processData();

   // Copy the filled raw event to the tree to be written
   new ((*fRawEventArray)[0]) AtRawEvent(fRawEvent);

   ++fEventID;
}

void AtHDFParserTask::FinishEvent()
{

   // fLogger -> Info(MESSAGE_ORIGIN, "End of file. Terminating FairRun.");
   // FairRootManager::Instance() -> SetFinishRun();
}
