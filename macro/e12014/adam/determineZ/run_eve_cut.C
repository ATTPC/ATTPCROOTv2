/*#include "TString.h"
#include "AtEventDrawTask.h"
#include "AtEventManager.h"

#include "FairParRootFileIo.h"
#include "FairRunAna.h"
*/
#include "FairLogger.h"

void run_eve_cut(TString cut = "cut1", TString species = "Bi200", int pressure = 150, bool lise = false,
                 TString OutputDataFile = "./data/output.reco_display.root")
{

   auto verbSpec =
      fair::VerbositySpec::Make(fair::VerbositySpec::Info::severity, fair::VerbositySpec::Info::file_line_function);
   fair::Logger::DefineVerbosity("user1", verbSpec);
   // fair::Logger::SetVerbosity("user1");
   // fair::Logger::SetConsoleSeverity("debug");

   TString InputDataFile = TString::Format("/mnt/analysis/e12014/TPC/%dTorr_yFit/%s/%s/%sChi2.root", pressure,
                                           cut.Data(), lise ? "LISE" : "SRIM", species.Data());

   // InputDataFile = "/mnt/analysis/e12014/TPC/150Torr_nomod/pConserve/SRIM/Bi200Chi2.root";
   // InputDataFile = "./Bi200NewObj.root";
   // InputDataFile = "./Bi200NewFit.root";

   TString dir = getenv("VMCWORKDIR");
   TString geoFile = "ATTPC_v1.1_geomanager.root";
   TString mapFile = "e12014_pad_mapping.xml";
   TString parFile = "ATTPC.e12014.par";

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

   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(dir + "/parameters/" + parFile, "in");
   fRun->GetRuntimeDb()->setFirstInput(parIo1);
   fRun->GetRuntimeDb()->getContainer("AtDigiPar");

   E12014::CreateMap();
   auto fMap = E12014::fMap;
   fMap->ParseXMLMap(mapDir.Data());

   auto eveMan = new AtViewerManager(fMap);

   auto tabMain = std::make_unique<AtTabFission>();
   tabMain->SetMultiHit(100); // Set the maximum number of multihits in the visualization

   auto tabPad = std::make_unique<AtTabPad>(2, 2);
   tabPad->DrawRawADC(0, 0);
   tabPad->DrawADC(0, 1);
   tabPad->DrawArrayAug("Q", 1, 0);
   tabPad->DrawArrayAug("Qreco", 1, 1);
   tabPad->DrawHits(1, 1);
   tabPad->DrawHits(1, 0);

   auto &fissionBranch = tabMain->GetFissionBranch();
   auto tabFF = std::make_unique<AtTabFF>(fissionBranch, false);

   eveMan->AddTab(std::move(tabMain));
   eveMan->AddTab(std::move(tabPad));
   eveMan->AddTab(std::move(tabFF));
   eveMan->AddTab(std::make_unique<AtTabEnergyLoss>(fissionBranch));

   eveMan->Init();

   std::cout << "Finished init" << std::endl;
}
