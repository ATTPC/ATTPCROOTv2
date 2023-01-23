/*#include "TString.h"
#include "AtEventDrawTask.h"
#include "AtEventManager.h"

#include "FairParRootFileIo.h"
#include "FairRunAna.h"
*/
#include "FairLogger.h"
#include "HEISTpid1.h"

void run_eve_test_heist(int runNum = 214, TString OutputDataFile = "./data/output.reco_display.root")
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
   AtEventManagerNew *eveMan = new AtEventManagerNew(fMap);

   auto tabMain = std::make_unique<AtTabMain>();
   tabMain->SetMap(fMap);
   tabMain->SetMultiHit(100); // Set the maximum number of multihits in the visualization
   tabMain->SetRawEventBranch("AtRawEventFiltered");
   tabMain->SetEventBranch("AtEventFiltered");

   auto tabPad = std::make_unique<AtTabPad>();
   tabPad->SetMap(fMap);
   tabPad->SetColumns(2);
   tabPad->SetRows(2);
   tabPad->SetDrawRawADC(0);
   tabPad->SetDrawADC(1);
   // tabPad->SetDrawArrayAug(2, "Qreco");
   tabPad->SetRawEventBranch("AtRawEventFiltered");
   tabPad->SetEventBranch("AtEventFiltered");

   auto heistInfo = std::make_unique<AtTabInfoHiRAEVT<HTMusicIC>>("MUSIC");
   auto tabMac = std::make_unique<AtTabMacro>();
   tabMac->AddInfoAugment("MusicIC", std::move(heistInfo));
   // tabMac->SetInputTree(evtInputDataFile, "E12014");
   //  tabMac->SetDrawTreeFunction(0, PlotPID);

   AtTabTask *tab = new AtTabTask();
   tab->AddTab(std::move(tabMain));
   tab->AddTab(std::move(tabPad));
   tab->AddTab(std::move(tabMac));

   eveMan->AddTask(tab);
   eveMan->Init();

   std::cout << "Finished init" << std::endl;
   // eveMan->RunEvent(27);
}
