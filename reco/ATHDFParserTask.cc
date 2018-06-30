#include "ATHDFParserTask.hh"

#include "FairRootManager.h"
#include "FairRunOnline.h"
#include "FairRun.h"
#include "FairRunAna.h"
#include "FairRuntimeDb.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(ATHDFParserTask);

ATHDFParserTask::ATHDFParserTask()
{
  fLogger = FairLogger::GetLogger();
  fIsPersistence = kFALSE;
  fRawEventArray = new TClonesArray("ATRawEvent");
  fEventID = -1;

}

ATHDFParserTask::~ATHDFParserTask()
{
}

void ATHDFParserTask::SetPersistence(Bool_t value)                                                { fIsPersistence = value; }


InitStatus ATHDFParserTask::Init()
{
  FairRootManager *ioMan = FairRootManager::Instance();
  if (ioMan == 0) {
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find RootManager!");

    return kERROR;
  }

  ioMan -> Register("ATRawEvent", "ATTPC", fRawEventArray, fIsPersistence);

  HDFParser = std::make_unique<ATHDFParser>();

    
  return kSUCCESS;
}

void ATHDFParserTask::SetParContainers()
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

void ATHDFParserTask::Exec(Option_t *opt)
{

  fRawEventArray -> Delete();

  if (fRawEvent == NULL)
	//fRawEvent = fDecoder -> GetRawEvent(fEventID++);

    //fInternalID++;
    //if(fInternalID%100==0) std::cout<<" Event Number "<<fEventID<<" Internal ID : "<<fInternalID<<" Number of Pads : "<<fRawEvent->GetNumPads()<<std::endl;

  new ((*fRawEventArray)[0]) ATRawEvent(fRawEvent);

  fRawEvent = NULL;

}

Int_t ATHDFParserTask::ReadEvent(Int_t eventID)
{
  fRawEventArray -> Delete();

  //fRawEvent = fDecoder -> GetRawEvent(eventID);
  //fEventIDLast = fDecoder -> GetEventID();

  if (fRawEvent == NULL)
    return 1;

  new ((*fRawEventArray)[0]) ATRawEvent(fRawEvent);



  return 0;
}


void ATHDFParserTask::FinishEvent()
{
  /*fRawEvent = fDecoder -> GetRawEvent();

  if (fRawEvent == NULL)
  {
    fLogger -> Info(MESSAGE_ORIGIN, "End of file. Terminating FairRun.");
    FairRootManager::Instance() -> SetFinishRun();
  }*/
}


