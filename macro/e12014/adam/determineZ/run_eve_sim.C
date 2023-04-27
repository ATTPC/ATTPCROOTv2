/*#include "TString.h"
#include "AtEventDrawTask.h"
#include "AtEventManager.h"

#include "FairParRootFileIo.h"
#include "FairRunAna.h"
*/
#include "FairLogger.h"

#include "/mnt/simulations/attpcroot/adam/ATTPCROOTv2/macro/e12014/viewerUpdate/HEISTpid1.h"
std::shared_ptr<MCFitter::AtMCFission> fitter = nullptr;
std::shared_ptr<AtPulseLine> pulse = nullptr;
std::shared_ptr<AtClusterizeLine> cluster = nullptr;
std::shared_ptr<AtSimpleSimulation> sim = nullptr;
std::shared_ptr<AtLineChargeModel> scModel = nullptr;
std::shared_ptr<AtPSADeconvFit> simPSA = nullptr;

void run_eve_sim(TString species = "Bi200", int pressure = 150,
                 TString OutputDataFile = "./data/output.reco_display.root")
{

   auto verbSpec =
      fair::VerbositySpec::Make(fair::VerbositySpec::Info::severity, fair::VerbositySpec::Info::file_line_function);
   fair::Logger::DefineVerbosity("user1", verbSpec);
   // fair::Logger::SetVerbosity("user1");
   // fair::Logger::SetConsoleSeverity("debug");

   TString InputDataFile = TString::Format("/mnt/analysis/e12014/TPC/%dTorr/%s.root", pressure, species.Data());
   // TString InputDataFile =
   // "/mnt/simulations/attpcroot/adam/ATTPCROOTv2/macro/e12014/simulation/eventGenerator/sym90/output_digi.root";
   TString evtInputDataFile = TString::Format("/mnt/analysis/e12014/TPC/%dTorr/%sEvt.root", pressure, species.Data());
   //   TString InputDataFile = "./data/output.root";

   std::cout << "Opening: " << InputDataFile << std::endl;

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
   auto tabFF = std::make_unique<AtTabFF>(fissionBranch);

   eveMan->AddTab(std::move(tabMain));
   eveMan->AddTab(std::move(tabPad));
   eveMan->AddTab(std::move(tabFF));
   eveMan->AddTab(std::make_unique<AtTabEnergyLoss>(fissionBranch));

   // Create underlying simulation class
   sim = std::make_shared<AtSimpleSimulation>(GeoDataPath.Data());
   sim->SetDistanceStep(5);

   scModel = std::make_shared<AtLineChargeModel>();
   scModel->SetBeamLocation({0, -6, 0}, {10, 0, 1000});
   sim->SetSpaceChargeModel(scModel);

   // Create and load energy loss models
   std::vector<std::pair<int, int>> ions;
   for (int i = 30; i <= 55; i++)
      ions.push_back({i, std::round((double)i / 85 * 204)});
   for (auto [Z, A] : ions) {
      auto eloss = std::make_shared<AtTools::AtELossTable>();
      eloss->LoadLiseTable(TString::Format("./eLoss/%d_%d.txt", Z, A).Data(), A, 0);
      sim->AddModel(Z, A, eloss);
   }

   cluster = std::make_shared<AtClusterizeLine>();
   pulse = std::make_shared<AtPulseLine>(fMap);
   pulse->SetSaveCharge(true);
   pulse->SetDoConvolution(false);
   pulse->SetNumIntegrationPoints(250);

   fitter = std::make_shared<MCFitter::AtMCFission>(sim, cluster, pulse);
   fitter->SetTimeEvent(true);

   simPSA = std::make_shared<AtPSADeconvFit>();
   simPSA->SetUseSimCharge(true);
   simPSA->SetThreshold(25);

   fitter->SetPSA(simPSA);
   fitter->SetNumIter(10);
   fitter->SetNumThreads(1);

   AtMCFitterTask *fitTask = new AtMCFitterTask(fitter);
   fitTask->SetPatternBranchName("AtFissionEvent");

   eveMan->AddTask(fitTask);

   eveMan->Init();

   std::cout << "Finished init" << std::endl;
}
