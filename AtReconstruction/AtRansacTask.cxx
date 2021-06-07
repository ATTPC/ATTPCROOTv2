#include "AtRansac.h"
#include "AtRansacTask.h"

// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"

#include <iostream>

ClassImp(AtRansacTask);

AtRansacTask::AtRansacTask()
{

   fLogger = FairLogger::GetLogger();
   fPar = NULL;

   kIsPersistence = kFALSE;
   kIsFullMode = kFALSE;
   kIsReprocess = kFALSE;

   fRANSACModel = pcl::SACMODEL_LINE;
   fRANSACThreshold = 5.0;
   fMinHitsLine = 5;
   fNumItera = 500;
   fRANSACAlg = 0;
   fRandSamplMode = 0;
   fCharThres = 0;
   fVertexMode = 0;
}

AtRansacTask::~AtRansacTask() {}

void AtRansacTask::SetPersistence(Bool_t value)
{
   kIsPersistence = value;
}
void AtRansacTask::SetModelType(int model)
{
   fRANSACModel = model;
}
void AtRansacTask::SetDistanceThreshold(Float_t threshold)
{
   fRANSACThreshold = threshold;
}
void AtRansacTask::SetFullMode()
{
   kIsFullMode = kTRUE;
}
void AtRansacTask::SetMinHitsLine(Int_t nhits)
{
   fMinHitsLine = nhits;
}
void AtRansacTask::SetTiltAngle(Double_t val)
{
   fTiltAngle = val;
}
void AtRansacTask::SetNumItera(Int_t niterations)
{
   fNumItera = niterations;
}
void AtRansacTask::SetAlgorithm(Int_t val)
{
   fRANSACAlg = val;
}
void AtRansacTask::SetRanSamMode(Int_t mode)
{
   fRandSamplMode = mode;
};
void AtRansacTask::SetIsReprocess(Bool_t value)
{
   kIsReprocess = value;
}
void AtRansacTask::SetChargeThreshold(Double_t value)
{
   fCharThres = value;
}
void AtRansacTask::SetVertexMode(Int_t value)
{
   fVertexMode = value;
}

InitStatus AtRansacTask::Init()
{

   if (fRANSACAlg == 0)
      fRansacArray = new TClonesArray("AtRANSACN::AtRansac");
   else if (fRANSACAlg == 1)
      fRansacArray = new TClonesArray("AtRansacMod");
   else if (fRANSACAlg == 2)
      fRansacArray = new TClonesArray("AtMlesacMod");
   else if (fRANSACAlg == 3)
      fRansacArray = new TClonesArray("AtLmedsMod");
   else {
      LOG(error) << "Cannot find Ransac algorithm!";
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

   ioMan->Register("AtRansac", "AtTPC", fRansacArray, kIsPersistence);

   if (kIsReprocess) {
      ioMan->Register("AtEventH", "AtTPC", fEventHArray, kIsPersistence);
      /*
      fS800CalcBr = (TClonesArray *) ioMan -> GetObject("s800cal");
      if (fS800CalcBr == 0) {
        fLogger -> Error(MESSAGE_ORIGIN, "Cannot find AtEvent array!");
        return kERROR;
      }
      ioMan -> Register("s800cal", "S800", fS800CalcBr, fIsPersistence);
      */
   }

   return kSUCCESS;
}

void AtRansacTask::SetParContainers()
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

void AtRansacTask::Exec(Option_t *opt)
{

   fRansacArray->Delete();

   if (fEventHArray->GetEntriesFast() == 0)
      return;

   fEvent = (AtEvent *)fEventHArray->At(0);

   if (fRANSACAlg == 0) {
      AtRANSACN::AtRansac *Ransac = (AtRANSACN::AtRansac *)new ((*fRansacArray)[0]) AtRANSACN::AtRansac();
      Ransac->SetTiltAngle(fTiltAngle);
      Ransac->SetModelType(fRANSACModel);
      Ransac->SetDistanceThreshold(fRANSACThreshold);
      Ransac->SetMinHitsLine(fMinHitsLine);
      if (kIsFullMode)
         Ransac->CalcRANSACFull(fEvent);
      else
         Ransac->CalcRANSAC(fEvent);
   }

   // std::cout << "/* Number of hits in the task    */"<<fEvent->GetNumHits() << '\n';
   if (fRANSACAlg == 1) {
      AtRansacMod *Rantest = (AtRansacMod *)new ((*fRansacArray)[0]) AtRansacMod();
      Rantest->SetDistanceThreshold(fRANSACThreshold);
      Rantest->SetMinHitsLine(fMinHitsLine);
      Rantest->SetNumItera(fNumItera);
      Rantest->SetRanSamMode(fRandSamplMode);
      Rantest->CalcRANSACMod(fEvent);
      Rantest->SetChargeThres(fCharThres);
      Rantest->SetVertexMod(fVertexMode);
   }

   if (fRANSACAlg == 2) {
      AtMlesacMod *Rantest = (AtMlesacMod *)new ((*fRansacArray)[0]) AtMlesacMod();
      Rantest->SetDistanceThreshold(fRANSACThreshold);
      Rantest->SetMinHitsLine(fMinHitsLine);
      Rantest->SetNumItera(fNumItera);
      Rantest->SetRanSamMode(fRandSamplMode);
      Rantest->CalcMlesacMod(fEvent);
      Rantest->SetChargeThres(fCharThres);
      Rantest->SetVertexMod(fVertexMode);
   }

   if (fRANSACAlg == 3) {
      AtLmedsMod *Rantest = (AtLmedsMod *)new ((*fRansacArray)[0]) AtLmedsMod();
      Rantest->SetDistanceThreshold(fRANSACThreshold);
      Rantest->SetMinHitsLine(fMinHitsLine);
      Rantest->SetNumItera(fNumItera);
      Rantest->SetRanSamMode(fRandSamplMode);
      Rantest->CalcLmedsMod(fEvent);
      Rantest->SetChargeThres(fCharThres);
      Rantest->SetVertexMod(fVertexMode);
   }
}
