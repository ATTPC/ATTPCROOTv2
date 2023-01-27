/*#include "TString.h"
#include "AtEventDrawTask.h"
#include "AtEventManager.h"

#include "FairParRootFileIo.h"
#include "FairRunAna.h"
*/
#include "FairLogger.h"

#include "HEISTpid1.h"

void run_eve_test_multiPSA(int runNum = 214, TString OutputDataFile = "./data/output.reco_display.root")
{
   auto verbSpec =
      fair::VerbositySpec::Make(fair::VerbositySpec::Info::severity, fair::VerbositySpec::Info::file_line_function);
   fair::Logger::DefineVerbosity("user1", verbSpec);
   // fair::Logger::SetVerbosity("user1");
   // fair::Logger::SetConsoleSeverity("debug");

   TString inputDirectory = "/mnt/analysis/e12014/TPC/unpackedLinked/";
   TString InputDataFile = TString::Format(inputDirectory + "/run_%04d.root", runNum);
   TString evtInputDataFile = TString::Format(inputDirectory + "/evtRun_%04d.root", runNum);
   std::cout << "Opening: " << InputDataFile << std::endl;
   std::cout << "Opening: " << evtInputDataFile << std::endl;

   TString dir = getenv("VMCWORKDIR");
   TString geoFile = "ATTPC_v1.1_geomanager.root";
   TString mapFile = "e12014_pad_mapping.xml";

   TString InputDataPath = InputDataFile;
   TString OutputDataPath = OutputDataFile;
   TString GeoDataPath = dir + "/geometry/" + geoFile;
   TString mapDir = dir + "/scripts/" + mapFile;

   FairRunAna *fRun = new FairRunAna();
   FairRootFileSink *sink = new FairRootFileSink(OutputDataFile);
   FairFileSource *source = new FairFileSource(InputDataFile);
   fRun->SetSource(source);
   fRun->SetSink(sink);
   fRun->SetGeomFile(GeoDataPath);

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParRootFileIo *parIo1 = new FairParRootFileIo();
   // parIo1->open("param.dummy.root");
   rtdb->setFirstInput(parIo1);

   auto fMap = std::make_shared<AtTpcMap>();
   fMap->ParseXMLMap(mapDir.Data());

   AtViewerManager *eveMan = new AtViewerManager(fMap);

   AtRawEvent *respAvgEvent;
   TFile *f2 = new TFile("respAvg.root");
   f2->GetObject("avgResp", respAvgEvent);
   f2->Close();

   auto threshold = 45;
   auto filterOrder = 6;
   auto cutoff = 75;
   auto cycles = 4;

   auto psa = std::make_unique<AtPSAIterDeconv>();
   psa->SetResponse(*respAvgEvent);
   psa->SetFilterOrder(filterOrder);
   psa->SetCutoffFreq(cutoff);
   psa->SetIterations(cycles);
   // psa->SetThreshold(threshold);
   auto sidePSA = new AtSidebarPSAIterDeconv(eveMan->GetSidebar());
   sidePSA->SetPSA(psa.get());
   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetInputBranch("AtRawEventFiltered");
   psaTask->SetOutputBranch("AtEventIter");
   psaTask->SetPersistence(kTRUE);
   eveMan->GetSidebar()->AddSidebarFrame(sidePSA);

   auto tabMain = std::make_unique<AtTabMain>();
   tabMain->SetMultiHit(100); // Set the maximum number of multihits in the visualization

   auto tabPad = std::make_unique<AtTabPad>(2, 2);
   tabPad->DrawRawADC(0, 0);
   tabPad->DrawADC(0, 1);
   tabPad->DrawAuxADC("IC", 1, 0);
   tabPad->DrawArrayAug("Qreco", 1, 1);

   auto heistTree =
      std::make_shared<AtTabInfoTree>("E12014", evtInputDataFile, AtViewerManager::Instance()->GetCurrentEntry());
   auto heistInfo = std::make_shared<AtTabInfoBranch<HTMusicIC>>(heistTree, "MUSIC");
   auto tabMac = std::make_unique<AtTabMacro>();
   tabMac->GetTabInfo()->AddAugment(heistInfo, "MusicIC");
   tabMac->GetTabInfo()->AddAugment(heistTree);
   tabMac->SetDrawTreeFunction(PlotPID);

   eveMan->AddTask(psaTask);
   eveMan->AddTab(std::move(tabMain));
   eveMan->AddTab(std::move(tabPad));
   eveMan->AddTab(std::move(tabMac));

   eveMan->Init();

   std::cout << "Finished init" << std::endl;
   // eveMan->RunEvent(27);
}
