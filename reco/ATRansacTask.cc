#include "ATRansac.hh"
#include "ATRansacTask.hh"

// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"

#include <iostream>

ClassImp(ATRansacTask);

ATRansacTask::ATRansacTask()
{

  fLogger = FairLogger::GetLogger();
  fPar = NULL;

  kIsPersistence = kFALSE;
  kIsFullMode    = kFALSE;

  fRANSACModel = pcl::SACMODEL_LINE;
  fRANSACThreshold = 5.0;
  fMinHitsLine = 5;

}

ATRansacTask::~ATRansacTask()
{
}

void   ATRansacTask::SetPersistence(Bool_t value)             { kIsPersistence   = value; }
void   ATRansacTask::SetModelType(int model)                  { fRANSACModel     = model; }
void   ATRansacTask::SetDistanceThreshold(Float_t threshold)  { fRANSACThreshold = threshold;}
void   ATRansacTask::SetFullMode()                            { kIsFullMode      = kTRUE;}
void   ATRansacTask::SetMinHitsLine(Int_t nhits)              { fMinHitsLine     = nhits;}

InitStatus
ATRansacTask::Init()
{

  fRansacArray = new TClonesArray("ATRANSACN::ATRansac");

  FairRootManager *ioMan = FairRootManager::Instance();
  if (ioMan == 0) {
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find RootManager!");
    return kERROR;
  }

  fEventHArray = (TClonesArray *) ioMan -> GetObject("ATEventH");
  if (fEventHArray == 0) {
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find ATEvent array!");
    return kERROR;
  }


  ioMan -> Register("ATRansac", "ATTPC", fRansacArray, kIsPersistence);




  return kSUCCESS;
}


void
ATRansacTask::SetParContainers()
{


  FairRun *run = FairRun::Instance();
  if (!run)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No analysis run!");

  FairRuntimeDb *db = run -> GetRuntimeDb();
  if (!db)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No runtime database!");

  fPar = (ATDigiPar *) db -> getContainer("ATDigiPar");
  if (!fPar)
    fLogger -> Fatal(MESSAGE_ORIGIN, "ATDigiPar not found!!");
}

void
ATRansacTask::Exec(Option_t *opt)
{

      fRansacArray -> Delete();

      if (fEventHArray -> GetEntriesFast() == 0)
       return;


      fEvent  = (ATEvent *) fEventHArray -> At(0);
      ATRANSACN::ATRansac *Ransac = (ATRANSACN::ATRansac *) new ((*fRansacArray)[0]) ATRANSACN::ATRansac();
      Ransac->SetModelType(fRANSACModel);
      Ransac->SetDistanceThreshold(fRANSACThreshold);
      if(kIsFullMode) Ransac->CalcRANSACFull(fEvent);
      else Ransac->CalcRANSAC(fEvent);

}
