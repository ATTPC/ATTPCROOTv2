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
  fNumItera = 500;
  fRANSACAlg = 0;

}

ATRansacTask::~ATRansacTask()
{
}

void   ATRansacTask::SetPersistence(Bool_t value)             { kIsPersistence   = value; }
void   ATRansacTask::SetModelType(int model)                  { fRANSACModel     = model; }
void   ATRansacTask::SetDistanceThreshold(Float_t threshold)  { fRANSACThreshold = threshold;}
void   ATRansacTask::SetFullMode()                            { kIsFullMode      = kTRUE;}
void   ATRansacTask::SetMinHitsLine(Int_t nhits)              { fMinHitsLine     = nhits;}
void   ATRansacTask::SetTiltAngle(Double_t val)               { fTiltAngle       = val;}
void   ATRansacTask::SetNumItera(Int_t niterations)  { fNumItera = niterations;}
void   ATRansacTask::SetAlgorithm(Int_t val)  { fRANSACAlg = val;}

InitStatus
ATRansacTask::Init()
{

  if(fRANSACAlg==0)fRansacArray = new TClonesArray("ATRANSACN::ATRansac");
  else if(fRANSACAlg==1) fRansacArray = new TClonesArray("ATRansacMod");
  else{
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find Ransac algorithm!");
    return kERROR;
  }

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

      if(fRANSACAlg==0){
        ATRANSACN::ATRansac *Ransac = (ATRANSACN::ATRansac *) new ((*fRansacArray)[0]) ATRANSACN::ATRansac();
        Ransac -> SetTiltAngle(fTiltAngle);
        Ransac->SetModelType(fRANSACModel);
        Ransac->SetDistanceThreshold(fRANSACThreshold);
        Ransac->SetMinHitsLine(fMinHitsLine);
        if(kIsFullMode) Ransac->CalcRANSACFull(fEvent);
        else Ransac->CalcRANSAC(fEvent);
      }

      //std::cout << "/* Number of hits in the task    */"<<fEvent->GetNumHits() << '\n';
      if(fRANSACAlg==1){
        ATRansacMod * Rantest = (ATRansacMod *) new ((*fRansacArray)[0]) ATRansacMod();
        Rantest->SetDistanceThreshold(fRANSACThreshold);
        Rantest->SetMinHitsLine(fMinHitsLine);
        Rantest->SetNumItera(fNumItera);
        Rantest->CalcRANSACMod(fEvent);
      }

}
