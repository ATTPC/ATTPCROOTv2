// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"

#include <iostream>
#include <memory>

#include "AtAnalysisTask.h"
#include "AtProtoAnalysis.h"

ClassImp(AtAnalysisTask);

AtAnalysisTask::AtAnalysisTask() : fAtPadCoord(boost::extents[10240][3][2])
{
   fLogger = FairLogger::GetLogger();
   fIsPersistence = kFALSE;
   fIsPhiReco = kFALSE;
   fIsFullScale = kFALSE;
   fHoughDist = 2.0;
   fRunNum = 0;
   fInternalID = 0;
   fProtoEventAnaArray = new TClonesArray("AtProtoEventAna");
   fTrackingEventAnaArray = new TClonesArray("AtTrackingEventAna");
   fProtoAnalysis = NULL;
   fTrackingAnalysis = NULL;
   fIsEnableMap = kFALSE;
   fIsSimpleMode = kFALSE;
}

AtAnalysisTask::~AtAnalysisTask()
{

   for (Int_t i = 0; i < 4; i++) {
      delete fHoughFit[i];
      delete fHitPatternFilter[i];
      delete fFitResult[i];
   }
}

void AtAnalysisTask::SetPersistence(Bool_t value)
{
   fIsPersistence = value;
}
void AtAnalysisTask::SetPhiReco()
{
   fIsPhiReco = kTRUE;
}
void AtAnalysisTask::SetFullScale()
{
   fIsFullScale = kTRUE;
}
void AtAnalysisTask::SetHoughDist(Double_t value)
{
   fHoughDist = value;
}
void AtAnalysisTask::SetUpperLimit(Double_t value)
{
   fUpperLimit = value;
}
void AtAnalysisTask::SetLowerLimit(Double_t value)
{
   fLowerLimit = value;
}
void AtAnalysisTask::SetELossPar(std::vector<Double_t> par[10])
{
   for (Int_t i = 0; i < 10; i++)
      fELossPar[i] = par[i];
}
void AtAnalysisTask::SetEtoRParameters(std::vector<Double_t> (&parRtoE)[10])
{
   for (Int_t i = 0; i < 10; i++)
      fEtoRPar[i] = parRtoE[i];
}
void AtAnalysisTask::AddParticle(std::vector<std::pair<Int_t, Int_t>> ptcl)
{
   fParticleAZ = ptcl;
}
void AtAnalysisTask::SetEnableMap()
{
   fIsEnableMap = kTRUE;
}
void AtAnalysisTask::SetMap(Char_t const *map)
{
   fMap = map;
}
void AtAnalysisTask::SetSimpleMode()
{
   fIsSimpleMode = kTRUE;
}

InitStatus AtAnalysisTask::Init()
{

   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == 0) {
      fLogger->Error(MESSAGE_ORIGIN, "Cannot find RootManager!");
      return kERROR;
   }

   /*fEventHArray = (TClonesArray *) ioMan -> GetObject("AtEventH");
   if (fEventHArray == 0) {
     fLogger -> Error(MESSAGE_ORIGIN, "Cannot find AtEvent array!");
     return kERROR;
   }*/

   // AtANALYSIS IS BASED ON PAtTERN RECOGNITION ALGORITHMS

   fHoughArray = (TClonesArray *)ioMan->GetObject("AtHough");
   if (fHoughArray == 0) {
      fLogger->Error(MESSAGE_ORIGIN, "Cannot find AtHough array!");
      // return kERROR;
   } else
      fLogger->Info(MESSAGE_ORIGIN, "AtHough array found!");

   fRansacArray = (TClonesArray *)ioMan->GetObject("AtRansac");
   if (fRansacArray == 0) {
      fLogger->Error(MESSAGE_ORIGIN, "Cannot find AtRansac array!");
      // return kERROR;
   } else
      fLogger->Info(MESSAGE_ORIGIN, "AtRansac array found!");

   if (fIsPhiReco && fHoughArray) { // Find the Array of ProtoEvents

      fProtoAnalysis = new AtProtoAnalysis();
      fProtoAnalysis->SetHoughDist(fHoughDist);
      fProtoAnalysis->SetUpperLimit(fUpperLimit);
      fProtoAnalysis->SetLowerLimit(fLowerLimit);

      fProtoEventHArray = (TClonesArray *)ioMan->GetObject("AtProtoEvent");
      if (fProtoEventHArray == 0) {
         fLogger->Error(
            MESSAGE_ORIGIN,
            "Cannot find AtProtoEvent array! If SetPhiReco method is enabled, Phi Reconstruction is needed");
         return kERROR;
      }

      // For fitting the Prototype data
      for (Int_t i = 0; i < 4; i++) {
         fHoughFit[i] =
            new TF1(Form("HoughFit%i", i), " (  (-TMath::Cos([0])/TMath::Sin([0]))*x ) + [1]/TMath::Sin([0])", 0, 120);
         fHitPatternFilter[i] = new TGraph();
      }

      ioMan->Register("AtProtoEventAna", "AtTPC", fProtoEventAnaArray, fIsPersistence);

   } else if (fIsFullScale && fRansacArray) {

      fTrackingAnalysis = new AtTrackingAnalysis();

      ioMan->Register("AtTrackingEventAna", "AtTPC", fTrackingEventAnaArray, fIsPersistence);
   }

   if (fIsEnableMap) {
      fAtMapPtr = new AtTpcMap();
      fAtMapPtr->GenerateATTPC();
      fPadPlane = fAtMapPtr->GetATTPCPlane();
      Bool_t MapIn = fAtMapPtr->ParseXMLMap(fMap);
      fLogger->Info(MESSAGE_ORIGIN, "AtTPC Map enabled");
      if (!MapIn)
         std::cerr << " -E- AtHoughTask - : Map was enabled but not found ! " << std::endl;
      fAtPadCoord = fAtMapPtr->GetPadCoordArr();
   }

   return kSUCCESS;
}

void AtAnalysisTask::SetParContainers()
{

   FairRun *run = FairRun::Instance();
   if (!run)
      fLogger->Fatal(MESSAGE_ORIGIN, "No analysis run!");

   FairRuntimeDb *db = run->GetRuntimeDb();
   if (!db)
      fLogger->Fatal(MESSAGE_ORIGIN, "No runtime database!");

   fPar = (AtDigiPar *)db->getContainer("AtDigiPar");
   if (!fPar)
      fLogger->Fatal(MESSAGE_ORIGIN, "AtDigiPar not found!!");
}

void AtAnalysisTask::Exec(Option_t *opt)
{

   fProtoEventAnaArray->Delete();
   fTrackingEventAnaArray->Delete();

   if (fIsPhiReco && fHoughArray) {

      if (fProtoEventHArray->GetEntriesFast() == 0)
         return;

      if (fHoughArray->GetEntriesFast() == 0)
         return;

      // TODO:Use dynamic casting for each detector. Do the same in the Hough Task
      fHoughSpace = (AtHoughSpaceLine *)fHoughArray->At(0);
      if (fIsPhiReco)
         fProtoevent = (AtProtoEvent *)fProtoEventHArray->At(0);
      fInternalID++;
      // std::cout << "  -I- AtAnalysisTask -  Event Number by Internal ID : "<<fInternalID<< std::endl;

      AtProtoEventAna *protoeventAna = (AtProtoEventAna *)new ((*fProtoEventAnaArray)[0]) AtProtoEventAna();

      // new ((*fAnalysisArray)[0]) AtProtoAnalysis();

      // AtProtoAnalysis * ProtoAnalysis = (AtProtoAnalysis *) new ((*fAnalysisArray)[0]) AtProtoAnalysis();
      // fProtoAnalysis = (AtProtoAnalysis *) fAnalysisArray->ConstructedAt(0);
      // std::auto_ptr<AtProtoAnalysis> ProtoAnalysis(new AtProtoAnalysis());
      fProtoAnalysis->Analyze(fProtoevent, protoeventAna, fHoughSpace, fHoughFit, fHitPatternFilter, fFitResult);

      for (Int_t i = 0; i < 4; i++) {
         // fHoughFit[i]->Set(0);
         fHitPatternFilter[i]->Set(0);
         // fFitResult[i]->Clear(0);
      }

   } else if (fIsFullScale && fRansacArray && fIsEnableMap) {

      if (fRansacArray->GetEntriesFast() == 0)
         return;

      fRansac = (AtRANSACN::AtRansac *)fRansacArray->At(0);
      fInternalID++;
      AtTrackingEventAna *trackingeventAna =
         (AtTrackingEventAna *)new ((*fTrackingEventAnaArray)[0]) AtTrackingEventAna();
      std::cout << "  -I- AtAnalysisTask -  Event Number by Internal ID : " << fInternalID << std::endl;
      if (fELossPar[0].size() > 0 && fParticleAZ.size() > 0 && fEtoRPar[0].size() > 0) {
         fTrackingAnalysis->SetElossParameters(fELossPar);
         fTrackingAnalysis->SetEtoRParameters(fEtoRPar);
         fTrackingAnalysis->AddParticle(fParticleAZ);
      } else
         std::cout << cYELLOW << " AtAnalysisTask::Exec - Warning! No Energy Loss parameters found! " << cNORMAL
                   << std::endl;

      if (fIsSimpleMode)
         fTrackingAnalysis->AnalyzeSimple(fRansac, trackingeventAna, fPadPlane, fAtPadCoord);
      else
         fTrackingAnalysis->Analyze(fRansac, trackingeventAna, fPadPlane, fAtPadCoord);
   }
}
