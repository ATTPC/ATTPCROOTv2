/*#include "TString.h"
#include "AtEventDrawTask.h"
#include "AtEventManager.h"

#include "FairParRootFileIo.h"
#include "FairRunAna.h"
*/
#include "FairLogger.h"

#include "../unpack/TxtEvent.h"
#include "/mnt/simulations/attpcroot/adam/ATTPCROOTv2/macro/e12014/viewerUpdate/HEISTpid1.h"

void run_test(TString species = "Bi200", int pressure = 150, TString OutputDataFile = "./data/fitterTest.root")
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

   auto fMap = std::make_shared<AtTpcMap>();
   fMap->ParseXMLMap(mapDir.Data());

   /**** Data reduction task (keep fission only) ****/
   AtDataReductionTask *reduceTask = new AtDataReductionTask();
   reduceTask->SetInputBranch("AtRawEvent");
   reduceTask->SetOutputBranch("AtFissionEvent");
   TxtEvents events;
   events.AddTxtFile("goodEvent.txt");
   reduceTask->SetReductionFunction(events);

   // Create underlying simulation class
   auto sim = std::make_shared<AtSimpleSimulation>(GeoDataPath.Data());

   auto scModel = std::make_shared<AtRadialChargeModel>(nullptr);
   scModel->SetStepSize(0.1);
   scModel->SetBeamLocation({0, -6, 0}, {10, 0, 1000});
   sim->SetSpaceChargeModel(scModel);

   // Create and load energy loss models
   std::vector<std::pair<int, int>> ions = {{42, 101}, {43, 103}};
   for (auto [Z, A] : ions) {
      auto eloss = std::make_shared<AtTools::AtELossTable>();
      eloss->LoadLiseTable(TString::Format("./eLoss/%d_%d.txt", Z, A).Data(), A, 0);
      sim->AddModel(Z, A, eloss);
   }

   auto cluster = std::make_shared<AtClusterizeLine>();
   auto pulse = std::make_shared<AtPulseLine>(fMap);
   pulse->SetSaveCharge(true);
   auto psa2 = std::make_shared<AtPSADeconvFit>();
   psa2->SetUseSimCharge(true);
   psa2->SetThreshold(25);

   auto fitter = std::make_shared<MCFitter::AtMCFission>(sim, cluster, pulse);
   fitter->SetPSA(psa2);
   fitter->SetNumIter(30);

   AtMCFitterTask *fitTask = new AtMCFitterTask(fitter);
   fitTask->SetPatternBranchName("AtFissionEvent");

   fRun->AddTask(reduceTask);
   fRun->AddTask(fitTask);

   fRun->Init();
   fRun->Run(0, 100);
   std::cout << "Finished init" << std::endl;
}
