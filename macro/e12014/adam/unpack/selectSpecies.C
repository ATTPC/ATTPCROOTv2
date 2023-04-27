#include <FairFileSource.h>
#include <FairParAsciiFileIo.h>
#include <FairRootFileSink.h>
#include <FairRuntimeDb.h>

#include <TChain.h>
#include <TFile.h>
#include <TStopwatch.h>
#include <TString.h>

#ifndef __CLING__
#include "../build/include/AtCopyTreeTask.h"
#include "../build/include/AtCutHEIST.h"
#include "../build/include/AtDataReductionTask.h"
#include "../build/include/AtFilterCalibrate.h"
#include "../build/include/AtFilterSubtraction.h"
#include "../build/include/AtFilterTask.h"
#include "../build/include/AtHDFUnpacker.h"
#include "../build/include/AtLinkDAQTask.h"
#include "../build/include/AtPSA.h"
#include "../build/include/AtPSAComposite.h"
#include "../build/include/AtPSADeconvFit.h"
#include "../build/include/AtPSATBAvg.h"
#include "../build/include/AtPSAtask.h"
#include "../build/include/AtRadialChargeModel.h"
#include "../build/include/AtRunAna.h"
#include "../build/include/AtSampleConsensus.h"
#include "../build/include/AtSampleConsensusTask.h"
#include "../build/include/AtSpaceChargeCorrectionTask.h"
#include "../build/include/AtTpcMap.h"
#include "../build/include/AtUnpackTask.h"
#endif

// Macro "unpack"
std::vector<int> torr150 = {130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146,
                            147, 148, 149, 150, 151, 152, 153, 154, 155, 157, 159, 160, 161, 162, 163, 164, 165,
                            166, 167, 200, 201, 202, 203, 204, 206, 207, 208, 210, 211, 212, 213, 214, 215, 216};

std::vector<int> torr150Short = {130, 133, 134, 140, 141, 142};

std::vector<int> torr200 = {173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188,
                            189, 190, 191, 192, 198, 217, 218, 219, 221, 222, 223, 224, 225, 226, 227, 228,
                            229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242};
void selectSpecies(TString species, TString pressure, std::vector<int> runs);

void selectSpecies(TString species, int pressure)
{
   if (pressure == 150)
      selectSpecies(species, "150Torr", torr150);
   if (pressure == 200)
      selectSpecies(species, "200Torr", torr200);
}

void selectSpecies(TString species, TString pressure, std::vector<int> runs)
{
   TString input = "/mnt/analysis/e12014/TPC/fission_linked_yFit2/run_%04d.root";
   TString inputEvt = "/mnt/analysis/e12014/TPC/fission_linked_yFit2/evtRun_%04d.root";
   TString inputCuts = TString::Format("/mnt/projects/hira/e12014/tpcSharedInfo/PIDcuts/%sYield.root", pressure.Data());
   TString output = TString::Format("/mnt/analysis/e12014/TPC/%s_yFit2/%s.root", pressure.Data(), species.Data());
   TString outputEvt = TString::Format("/mnt/analysis/e12014/TPC/%s_yFit2/%sEvt.root", pressure.Data(), species.Data());

   TString geoFile = "ATTPC_v1.1_geomanager.root";
   TString mapFile = "e12014_pad_mapping.xml";
   TString parFile = "ATTPC.e12014.par";

   TString dir = getenv("VMCWORKDIR");

   AtRunAna *fRun = new AtRunAna();
   fRun->SetSink(new FairRootFileSink(output));
   fRun->SetGeomFile(dir + "/geometry/" + geoFile);

   // Get the initial
   FairFileSource *source = nullptr;
   TChain evtChain("E12014");
   AtLinkDAQTask *linkTask = new AtLinkDAQTask();
   for (auto run : runs) {
      TString fileName = TString::Format(input, run);
      TString evtFileName = TString::Format(inputEvt, run);

      // Check if the file exists
      auto file = TFile::Open(fileName);
      if (file == nullptr)
         continue;
      delete file;

      // If the file exists add it to the run and to the linker
      evtChain.Add(evtFileName);
      if (source == nullptr) {
         source = new FairFileSource(fileName);
         linkTask->SetInputTree(evtFileName, "E12014");
      } else {
         source->AddFile(fileName);
         linkTask->AddInputTree(evtFileName);
      }
   }

   fRun->SetSource(source);

   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(dir + "/parameters/" + parFile, "in");
   fRun->GetRuntimeDb()->setFirstInput(parIo1);
   fRun->GetRuntimeDb()->getContainer("AtDigiPar");

   auto fMap = std::make_shared<AtTpcMap>();
   fMap->ParseXMLMap(dir + "/scripts/" + mapFile);

   /**** Cut on species ****/
   AtCutHEIST cut(&evtChain, inputCuts);
   cut.AddSpecies(species.Data());
   AtDataReductionTask *reduceTask = new AtDataReductionTask();
   reduceTask->SetInputBranch("AtRawEventRaw");
   reduceTask->SetReductionFunction([&cut]() { return cut(); });

   /***** Copy tree to new file *****/
   AtCopyTreeTask *task = new AtCopyTreeTask();

   /***** Link HEIST and TPC *****/
   linkTask->SetEvtOutputFile(outputEvt);
   linkTask->SetInputBranch("AtRawEventRaw");
   linkTask->SetEvtTimestamp("tstamp");
   linkTask->SetTpcTimestampIndex(1);
   linkTask->SetSearchMean(1);
   linkTask->SetSearchRadius(2);
   linkTask->SetCorruptedSearchRadius(1000);

   fRun->AddTask(reduceTask);
   fRun->AddTask(task);
   fRun->AddTask(linkTask);

   fRun->Init();

   if (FairRootManager::Instance()->GetInChain()->GetEntries() != linkTask->GetInChain()->GetEntries())
      LOG(fatal) << "Number of events in EVT and FairRoot tree do not match!!!!!!!";

   // fRun->Run(0, 100);
   fRun->Run();
}
