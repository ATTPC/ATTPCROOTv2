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

  fIsPersistence = kFALSE;

  fRANSACModel = pcl::SACMODEL_LINE;
  fRANSACThreshold = 5.0;

}

ATRansacTask::~ATRansacTask()
{
}

void   ATRansacTask::SetPersistence(Bool_t value)             { fIsPersistence   = value; }
void   ATRansacTask::SetModelType(int model)                  { fRANSACModel     = model; }
void   ATRansacTask::SetDistanceThreshold(Float_t threshold)  { fRANSACThreshold = threshold;}

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


  ioMan -> Register("ATRansac", "ATTPC", fRansacArray, fIsPersistence);




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
      Ransac->CalcRANSAC(fEvent);

}
