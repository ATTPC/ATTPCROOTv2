#include "AtHoughSpace.h"
#include "AtHoughSpaceLine.h"
#include "AtHoughSpaceCircle.h"

// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"

#include <iostream>

#include "AtHoughTask.h"

ClassImp(AtHoughTask);

AtHoughTask::AtHoughTask() : fAtPadCoord(boost::extents[10240][3][2])
{
   fLogger = FairLogger::GetLogger();
   fPar = NULL;

   fIsPersistence = kFALSE;
   fIsLinear = kFALSE;
   fIsCircular = kFALSE;
   fIsPhiReco = kFALSE;
   fRadThreshold = 0.0;
   fHoughThreshold = 0.0;
   fHoughDistance = 5.0;

   fEvent = NULL;
   fProtoevent = NULL;

   fInternalID = 0;

   fIsEnableMap = kFALSE;
}

AtHoughTask::~AtHoughTask() {}

void AtHoughTask::SetPersistence(Bool_t value)
{
   fIsPersistence = value;
}
void AtHoughTask::SetThreshold(Double_t threshold)
{
   fThreshold = threshold;
}
void AtHoughTask::SetRadiusThreshold(Float_t value)
{
   fRadThreshold = value;
}
void AtHoughTask::SetLinearHough()
{
   fIsLinear = kTRUE;
   fIsCircular = kFALSE;
}
void AtHoughTask::SetCircularHough()
{
   fIsCircular = kTRUE;
   fIsLinear = kFALSE;
}
void AtHoughTask::SetPhiReco()
{
   fIsPhiReco = kTRUE;
}
void AtHoughTask::SetHoughThreshold(Double_t value)
{
   fHoughThreshold = value;
}
void AtHoughTask::SetHoughDistance(Double_t value)
{
   fHoughDistance = value;
}
void AtHoughTask::SetEnableMap()
{
   fIsEnableMap = kTRUE;
}
void AtHoughTask::SetMap(Char_t const *map)
{
   fMap = map;
}

InitStatus AtHoughTask::Init()
{

   if (fIsLinear)
      fHoughArray = new TClonesArray("AtHoughSpaceLine");
   else if (fIsCircular)
      fHoughArray = new TClonesArray("AtHoughSpaceCircle");
   else {

      LOG(error) << "-I- AtHoughTask : Hough Space Calculation NOT Set. Please choose a Hough Space Topology";
      return kERROR;
   }

   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == 0) {
      LOG(error) << "Cannot find RootManager!";
      return kERROR;
   }

   fEventHArray = (TClonesArray *)ioMan->GetObject("AtEventH");
   if (fEventHArray == 0) {
      LOG(error) << "Cannot find AtEvent array!";
      return kERROR;
   }

   if (fIsPhiReco) { // Find the Array of ProtoEvents
      fProtoEventHArray = (TClonesArray *)ioMan->GetObject("AtProtoEvent");
      if (fProtoEventHArray == 0) {
         LOG(error) << "Cannot find AtProtoEvent array! If SetPhiReco method is enabled, Phi Reconstruction is needed";
         return kERROR;
      }
   }

   // Pointer to the Pad Plane map for digitization during the MC
   if (fIsEnableMap) {
      fAtMapPtr = new AtTpcMap();
      fAtMapPtr->GenerateAtTpc();
      fPadPlane = fAtMapPtr->GetAtTpcPlane();
      Bool_t MapIn = fAtMapPtr->ParseXMLMap(fMap);
      LOG(info) << "AtTPC Map enabled";
      if (!MapIn)
         std::cerr << " -E- AtHoughTask - : Map was enabled but not found ! " << std::endl;
      fAtPadCoord = fAtMapPtr->GetPadCoordArr();
   }

   ioMan->Register("AtHough", "AtTPC", fHoughArray, fIsPersistence);

   return kSUCCESS;
}

void AtHoughTask::SetParContainers()
{

   FairRun *run = FairRun::Instance();
   if (!run)
      LOG(fatal) << "No analysis run!";

   FairRuntimeDb *db = run->GetRuntimeDb();
   if (!db)
      LOG(fatal) << "No runtime database!";

   fPar = (AtDigiPar *)db->getContainer("AtDigiPar");
   if (!fPar)
      LOG(fatal) << "AtDigiPar not found!!";
}

void AtHoughTask::Exec(Option_t *opt)
{
   fHoughArray->Delete();

   if (fEventHArray->GetEntriesFast() == 0)
      return;

   if (fIsPhiReco) {
      if (fProtoEventHArray->GetEntriesFast() == 0)
         return;
   }

   // AtEvent *Event = (AtEvent *) fEventHArray -> At(0);
   fEvent = (AtEvent *)fEventHArray->At(0);
   if (fIsPhiReco)
      fProtoevent = (AtProtoEvent *)fProtoEventHArray->At(0);
   fInternalID++;
   std::cout << "  -I- AtHoughTask -  Event Number :  " << fEvent->GetEventID() << " Internal ID : " << fInternalID
             << std::endl;

   if (fIsLinear) { // TODO: Solve this dirty way with a dynamic cast and make global pointers

      AtHoughSpaceLine *HoughSpace = (AtHoughSpaceLine *)new ((*fHoughArray)[0]) AtHoughSpaceLine();
      HoughSpace->SetRadiusThreshold(fRadThreshold);
      HoughSpace->SetHoughDistance(fHoughDistance);
      if (fIsPhiReco)
         HoughSpace->CalcHoughSpace(fProtoevent, kTRUE, kTRUE, kTRUE, kTRUE);
      else
         HoughSpace->CalcHoughSpace(fEvent);

   } else if (fIsCircular) {
      AtHoughSpaceCircle *HoughSpace = (AtHoughSpaceCircle *)new ((*fHoughArray)[0]) AtHoughSpaceCircle();
      HoughSpace->SetThreshold(fHoughThreshold);
      // if(fIsEnableMap) HoughSpace ->CalcHoughSpace(fEvent,fPadPlane);
      if (fIsEnableMap)
         HoughSpace->CalcHoughSpace(fEvent, fPadPlane, fAtPadCoord);
      else
         HoughSpace->CalcHoughSpace(fEvent, kTRUE, kTRUE, kTRUE);
   }

   //(AtHoughSpaceLine *) new ((*fHoughArray)[0]) AtHoughSpaceLine();
   // event -> SetEventID(event -> GetEventID());
   /* event -> SetEventID(rawEvent -> GetEventID());

  if (!(rawEvent -> IsGood()))
    event -> SetIsGood(kFALSE);
  else {
    fPSA -> Analyze(rawEvent, event);
    event -> SetIsGood(kTRUE);
  }*/
}

/*void
AtHoughTask::FinishEvent()
{

  if (fEventHArray -> GetEntriesFast() == 0) return;

  fEvent  = (AtEvent *) fEventHArray -> At(0);
  if (fEvent == NULL)
  {
    fLogger -> Info(MESSAGE_ORIGIN, "End of file. Terminating FairRun.");
    FairRootManager::Instance() -> SetFinishRun();
  }
}*/
