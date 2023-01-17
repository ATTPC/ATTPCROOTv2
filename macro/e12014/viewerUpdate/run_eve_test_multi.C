/*#include "TString.h"
#include "AtEventDrawTask.h"
#include "AtEventManager.h"

#include "FairParRootFileIo.h"
#include "FairRunAna.h"
*/
#include "FairLogger.h"

void run_eve_test_multi(int runNum = 214, TString OutputDataFile = "./data/output.reco_display.root")
{
   auto verbSpec =
      fair::VerbositySpec::Make(fair::VerbositySpec::Info::severity, fair::VerbositySpec::Info::file_line_function);
   fair::Logger::DefineVerbosity("user1", verbSpec);
   // fair::Logger::SetVerbosity("user1");
   // fair::Logger::SetConsoleSeverity("debug");

   TString inputDirectory = "/mnt/analysis/e12014/TPC/unpackedCalibrated/";
   TString InputDataFile = TString::Format(inputDirectory + "/run_%04dReduced.root", runNum);
   std::cout << "Opening: " << InputDataFile << std::endl;

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

   auto tabPad = std::make_unique<AtTabPad>();
   tabPad->SetMap(fMap);
   tabPad->SetColumns(2);
   tabPad->SetRows(2);
   tabPad->SetDrawRawADC(0);
   tabPad->SetDrawADC(1);
   // tabPad->SetDrawArrayAug(2, "Qreco");

   AtTabTask *tab = new AtTabTask();
   tab->AddTab(std::move(tabMain));
   tab->AddTab(std::move(tabPad));

   eveMan->AddTask(tab);
   eveMan->Init();

   std::cout << "Finished init" << std::endl;
   // eveMan->RunEvent(27);
}
