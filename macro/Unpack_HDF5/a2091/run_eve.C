#include "FairLogger.h"

void run_eve(int runNum = 138, TString OutputDataFile = "./output.reco_display.root")
{

   auto verbSpec =
      fair::VerbositySpec::Make(fair::VerbositySpec::Info::severity, fair::VerbositySpec::Info::file_line_function);
   fair::Logger::DefineVerbosity("user1", verbSpec);
   // fair::Logger::SetVerbosity("user1");
   // fair::Logger::SetConsoleSeverity("debug");

   TString inputDirectory = "./";
   TString InputDataFile = TString::Format(inputDirectory + "/run_%04d.root", runNum);
   std::cout << "Opening: " << InputDataFile << std::endl;

   TString dir = getenv("VMCWORKDIR");
   TString geoFile = "ATTPC_D300torr_v2_geomanager.root";
   TString mapFile = "ANL2023.xml";

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
   auto eveMan = new AtViewerManager(fMap);

   auto tabMain = std::make_unique<AtTabMain>();
   tabMain->SetMultiHit(100); // Set the maximum number of multihits in the visualization

   auto psa = new AtPSAMax();
   psa->SetThreshold(50);
   // Create PSA task
   AtPSAtask *psaTask = new AtPSAtask(psa);
   psaTask->SetPersistence(false);
   // psaTask->SetInputBranch("AtRawEventFiltered");
   psaTask->SetOutputBranch("AtEventChange");
   auto sidePSA = new AtSidebarPSA(eveMan->GetSidebar());
   sidePSA->SetPSA(psa);
   eveMan->GetSidebar()->AddSidebarFrame(sidePSA);
   eveMan->AddTask(psaTask);
   auto tabPad = std::make_unique<AtTabPad>(2, 1);
   tabPad->DrawRawADC(0, 0);
   tabPad->DrawADC(0, 1);

   eveMan->AddTab(std::move(tabMain));
   eveMan->AddTab(std::move(tabPad));

   eveMan->Init();

   std::cout << "Finished init" << std::endl;
}
