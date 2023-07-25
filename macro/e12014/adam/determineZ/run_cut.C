#include "FairLogger.h"

bool cut1(AtFissionEvent *evt)
{
   // Get the upper angle cut
   if (evt == nullptr)
      return false;
   auto vZ = evt->GetVertex().Z();
   if (vZ < 200 || vZ > 955)
      return false;

   // Bi200
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

   // Bi200
   double upper = 0.58 + (955 - vZ) * .2 / 755;
   double lower = 0.40 + (955 - vZ) * .2 / 755;

   double ang = evt->GetFoldingAngle();

   return ang < upper && ang > lower;
}

enum ChargeObj { kChi2, kDiff2 };

string to_string(ChargeObj obj)
{
   switch (obj) {
   case ChargeObj::kChi2: return "Chi2";

   case ChargeObj::kDiff2: return "Diff2";
   }
   return "";
}

/**
 * Macro for running the MCFit code applying some cut given the following compount nucleus
 * This WILL overwrite data if you are not carful.
 */
int Zcn = 83 + 2;
int Acn = 200 + 4;
int Zmin = 26;
int Zmax = 59;

void run_cut(TString cutName = "cut1", TString species = "Bi200", int pressure = 150, bool lise = true,
             ChargeObj obj = kChi2)
{

   auto verbSpec =
      fair::VerbositySpec::Make(fair::VerbositySpec::Info::severity, fair::VerbositySpec::Info::file_line_function);
   fair::Logger::DefineVerbosity("user1", verbSpec);
   // fair::Logger::SetVerbosity("user1");
   // fair::Logger::SetConsoleSeverity("debug");
   TString path = "/mnt/analysis/e12014/TPC/%dTorr_yFit";
   TString InputDataFile = TString::Format(path + "/%s.root", pressure, species.Data());
   TString evtInputDataFile = TString::Format(path + "/%sEvt.root", pressure, species.Data());

   TString OutputDataFile = TString::Format(path + "/%s/%s/%s%s.root", pressure, cutName.Data(), lise ? "LISE" : "SRIM",
                                            species.Data(), to_string(obj).c_str());
   // OutputDataFile = "./Bi200NewObj.root";
   // OutputDataFile = "./Chi2Norm.root";
   // OutputDataFile = "./Bi200NewFit.root";
   /*
         TString OutputDataFile = TString::Format("/mnt/analysis/e12014/TPC/%dTorr/%s/%s/%s.root", pressure,
         cutName.Data(), lise ? "LISE" : "SRIM", species.Data());
      */
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

   auto sim = std::make_shared<AtSimpleSimulation>(GeoDataPath.Data());
   sim->SetDistanceStep(5);

   auto scModel = std::make_shared<AtLineChargeModel>();
   scModel->SetBeamLocation({0, -6, 0}, {10, 0, 1000});
   sim->SetSpaceChargeModel(scModel);

   // Create and load energy loss models
   std::vector<std::pair<int, int>> ions;
   for (int i = Zmin; i <= Zmax; i++)
      ions.push_back({i, std::round((double)i / Zcn * Acn)});
   for (auto [Z, A] : ions) {
      auto eloss = std::make_shared<AtTools::AtELossTable>();
      if (lise)
         eloss->LoadLiseTable(TString::Format("./eLoss/LISE/%d_%d.txt", Z, A).Data(), A, 0);
      else
         eloss->LoadSrimTable(TString::Format("./eLoss/SRIM/%d_%d.txt", Z, A).Data());
      sim->AddModel(Z, A, eloss);
   }

   auto cluster = std::make_shared<AtClusterizeLine>();
   auto pulse = std::make_shared<AtPulseLine>(fMap);
   pulse->SetSaveCharge(true);
   pulse->SetDoConvolution(false);
   pulse->SetNumIntegrationPoints(250);

   auto fitter = std::make_shared<MCFitter::AtMCFission>(sim, cluster, pulse);
   fitter->SetTimeEvent(true);
   fitter->SetCN({Zcn, Acn});
   fitter->SetZRange(Zmin, Zmax);
   switch (obj) {
   case kDiff2: fitter->SetChargeObjective(MCFitter::AtMCFission::ObjectiveChargeDiff2); break;
   case kChi2: fitter->SetChargeObjective(MCFitter::AtMCFission::ObjectiveChargeChi2); break;
   }
   // fitter->SetAmp(0.454);

   auto simPSA = std::make_shared<AtPSADeconvFit>();
   simPSA->SetUseSimCharge(true);
   simPSA->SetThreshold(25);

   fitter->SetPSA(simPSA);
   fitter->SetNumIter(100);
   fitter->SetNumRounds(2);
   fitter->SetNumThreads(4);

   AtMCFitterTask *fitTask = new AtMCFitterTask(fitter);
   fitTask->SetPatternBranchName("AtFissionEvent");
   fitTask->SetSaveRawEvent(true);
   fitTask->SetSaveEvent(true);

   fRun->AddTask(reduceTask);
   fRun->AddTask(copyTask);
   fRun->AddTask(fitTask);

   auto initStart = std::chrono::high_resolution_clock::now();
   fRun->Init();
   auto runStart = std::chrono::high_resolution_clock::now();
   // fRun->Run(0, 4);
   // fRun->Run(0, 100);
   // fRun->Run(0, 2000);
   fRun->Run();
   auto runStop = std::chrono::high_resolution_clock::now();

   LOG(info) << "Run processed in "
             << std::chrono::duration_cast<std::chrono::milliseconds>(runStop - initStart).count() / (double)1000
             << " seconds.";
}
