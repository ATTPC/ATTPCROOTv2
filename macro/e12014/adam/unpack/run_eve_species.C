/*#include "TString.h"
#include "AtEventDrawTask.h"
#include "AtEventManager.h"

#include "FairParRootFileIo.h"
#include "FairRunAna.h"
*/
#include "FairLogger.h"

#include "/mnt/simulations/attpcroot/adam/ATTPCROOTv2/macro/e12014/viewerUpdate/HEISTpid1.h"
void run_eve_species(TString species = "Bi200", int pressure = 150,
                     TString OutputDataFile = "./data/output.reco_display.root")
{

   auto verbSpec =
      fair::VerbositySpec::Make(fair::VerbositySpec::Info::severity, fair::VerbositySpec::Info::file_line_function);
   fair::Logger::DefineVerbosity("user1", verbSpec);
   // fair::Logger::SetVerbosity("user1");
   // fair::Logger::SetConsoleSeverity("debug");

   // TString inputDirectory = TString::Format("/mnt/analysis/e12014/TPC/%dTorr/%s.root", pressure, species.Data());
   TString InputDataFile = TString::Format("/mnt/analysis/e12014/TPC/%dTorr_nomod/%s.root", pressure, species.Data());
   TString evtInputDataFile =
      TString::Format("/mnt/analysis/e12014/TPC/%dTorr_nomod/%sEvt.root", pressure, species.Data());
   //   TString InputDataFile = "./data/output.root";

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
   auto eveMan = new AtViewerManager(fMap);

   auto tabMain = std::make_unique<AtTabFission>();
   tabMain->SetMultiHit(100); // Set the maximum number of multihits in the visualization

   auto tabPad = std::make_unique<AtTabPad>(2, 2);
   tabPad->DrawRawADC(0, 0);
   tabPad->DrawADC(0, 1);
   tabPad->DrawAuxADC("IC", 1, 0);
   // tabPad->SetDrawArrayAug(2, "Qreco");

   auto heistTree =
      std::make_shared<AtTabInfoTree>("E12014", evtInputDataFile, AtViewerManager::Instance()->GetCurrentEntry());
   auto heistInfo = std::make_shared<AtTabInfoBranch<HTMusicIC>>(heistTree, "MUSIC");
   auto tabMac = std::make_unique<AtTabMacro>();
   tabMac->GetTabInfo()->AddAugment(std::move(heistInfo), "MusicIC");
   tabMac->GetTabInfo()->AddAugment(std::move(heistTree));
   tabMac->SetDrawTreeFunction(PlotPID);

   eveMan->AddTab(std::move(tabMain));
   eveMan->AddTab(std::move(tabPad));
   eveMan->AddTab(std::move(tabMac));

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(0);
   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetInputBranch("AtRawEventSub");
   eveMan->AddTask(psaTask);

   eveMan->Init();

   std::cout << "Finished init" << std::endl;
}
