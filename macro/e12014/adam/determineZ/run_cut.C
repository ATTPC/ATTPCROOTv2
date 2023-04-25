#include "FairLogger.h"

bool cut1(AtFissionEvent *evt)
{
   // Get the upper angle cut
   if (evt == nullptr)
      return false;
   auto vZ = evt->GetVertex().Z();
   if (vZ < 200 || vZ > 955)
      return false;

   double upper = 0.58 + (955 - vZ) * .2 / 755;
   double lower = 0.46 + (955 - vZ) * .2 / 755;
   double ang = evt->GetFoldingAngle();

   return ang < upper && ang > lower;
}

bool cut2(AtFissionEvent *evt)
{
   // Get the upper angle cut
   if (evt == nullptr)
      return false;
   auto vZ = evt->GetVertex().Z();
   if (vZ < 200 || vZ > 955)
      return false;

   double upper = 0.58 + (955 - vZ) * .2 / 755;
   double lower = 0.40 + (955 - vZ) * .2 / 755;
   double ang = evt->GetFoldingAngle();

   return ang < upper && ang > lower;
}

void run_cut(TString cutName = "cut1", TString species = "Bi200", int pressure = 150)
{
   ROOT::EnableThreadSafety();

   auto verbSpec =
      fair::VerbositySpec::Make(fair::VerbositySpec::Info::severity, fair::VerbositySpec::Info::file_line_function);
   fair::Logger::DefineVerbosity("user1", verbSpec);
   // fair::Logger::SetVerbosity("user1");
   // fair::Logger::SetConsoleSeverity("debug");

   TString InputDataFile = TString::Format("/mnt/analysis/e12014/TPC/%dTorr/%s.root", pressure, species.Data());
   TString evtInputDataFile = TString::Format("/mnt/analysis/e12014/TPC/%dTorr/%sEvt.root", pressure, species.Data());
   TString OutputDataFile =
      TString::Format("/mnt/analysis/e12014/TPC/%dTorr/%s/%s.root", pressure, cutName.Data(), species.Data());

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

   /** Add the cuts and copy the data to the output tree **/
   AtDataReductionTask *reduceTask = new AtDataReductionTask();
   reduceTask->SetInputBranch("AtFissionEvent");
   if (cutName == "cut1")
      reduceTask->SetReductionFunction<AtFissionEvent>(&cut1);
   else if (cutName == "cut2")
      reduceTask->SetReductionFunction<AtFissionEvent>(&cut2);
   else
      throw std::invalid_argument(cutName + " is not a defined cut!");
   AtCopyTreeTask *copyTask = new AtCopyTreeTask();

   /** Create the simulation to do the fittin on **/
   /*
   auto sim = std::make_shared<AtSimpleSimulation>(GeoDataPath.Data());
   sim->SetDistanceStep(5);

   auto scModel = std::make_shared<AtLineChargeModel>();
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

   auto cluster = std::make_shared<AtClusterizeLine>();
   auto pulse = std::make_shared<AtPulseLine>(fMap);
   pulse->SetSaveCharge(true);
   pulse->SetDoConvolution(false);
   pulse->SetNumIntegrationPoints(250);

   auto fitter = std::make_shared<MCFitter::AtMCFission>(sim, cluster, pulse);
   fitter->SetTimeEvent(true);

   auto simPSA = std::make_shared<AtPSADeconvFit>();
   simPSA->SetUseSimCharge(true);
   simPSA->SetThreshold(25);

   fitter->SetPSA(simPSA);
   fitter->SetNumIter(10);
   // fitter->SetNumThreads(1);


   AtMCFitterTask *fitTask = new AtMCFitterTask(fitter);
   fitTask->SetPatternBranchName("AtFissionEvent");
   fitTask->SetSaveRawEvent(true);
   fitTask->SetSaveEvent(true);
   */
   fRun->AddTask(reduceTask);
   fRun->AddTask(copyTask);
   // fRun->AddTask(fitTask);

   fRun->Init();

   fRun->Run(0, 65);
   // fRun->Run();
}
