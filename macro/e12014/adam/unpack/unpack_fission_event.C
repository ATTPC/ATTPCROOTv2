#include <FairFileSource.h>
#include <FairParAsciiFileIo.h>
#include <FairRootFileSink.h>
#include <FairRuntimeDb.h>

#include <TChain.h>
#include <TFile.h>
#include <TString.h>
#ifndef __CLING__
#include "../build/include/AtCopyTreeTask.h"
#include "../build/include/AtCutHEIST.h"
#include "../build/include/AtDataReductionTask.h"
#include "../build/include/AtLinkDAQTask.h"
#include "../build/include/AtPSADeconvFit.h"
#include "../build/include/AtRunAna.h"
#include "../build/include/AtTpcMap.h"
#endif

void unpack_fission_event(TString species, TString pressure)
{
   TString input = TString::Format("/mnt/analysis/e12014/TPC/%s/%s.root", pressure.Data(), species.Data());
   TString output = TString::Format("/mnt/analysis/e12014/TPC/%s/%sFission.root", pressure.Data(), species.Data());
   TString outputEvt = TString::Format("/mnt/analysis/e12014/TPC/%s/%sEvt.root", pressure.Data(), species.Data());

   TString geoFile = "ATTPC_v1.1_geomanager.root";
   TString mapFile = "e12014_pad_mapping.xml";
   TString parFile = "ATTPC.e12014.par";

   TString dir = getenv("VMCWORKDIR");
   AtRunAna *fRun = new AtRunAna();
   fRun->SetSink(new FairRootFileSink(output));
   fRun->SetGeomFile(dir + "/geometry/" + geoFile);
   fRun->SetSource(new FairFileSource(input));

   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(dir + "/parameters/" + parFile, "in");
   fRun->GetRuntimeDb()->setFirstInput(parIo1);
   fRun->GetRuntimeDb()->getContainer("AtDigiPar");

   auto fMap = std::make_shared<AtTpcMap>();
   fMap->ParseXMLMap(dir + "/scripts/" + mapFile);

   /**
    * These files have the following branches (name: description)
    * AtRawEventSub: AtRawEvent with ch0 subtraction
    * AtEvent : AtEvent with max finding PSA
    *
    * To add:
    * AtEvent: AtEvent with PSADeconvFit
    * AtEventCorr: AtEvent with correction (from branch AtEvent)
    */

   AtRawEvent *respAvgEvent;
   TFile *f2 = new TFile("respAvg.root");
   f2->GetObject("avgResp", respAvgEvent);
   f2->Close();

   // Create PSA task this is the uncorrected data
   auto psa = std::make_unique<AtPSADeconvFit>();
   psa->SetResponse(*respAvgEvent);
   psa->SetThreshold(15); // Threshold in charge units
   psa->SetFilterOrder(6);
   psa->SetCutoffFreq(75);
   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetInputBranch("AtRawEventSub");
   eveMan->AddTask(psaTask);
}
