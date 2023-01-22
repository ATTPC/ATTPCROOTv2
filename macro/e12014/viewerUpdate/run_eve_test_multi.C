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
   AtViewerManager *eveMan = new AtViewerManager(fMap);
   // eveMan->GetSidebar()->UsePictureButtons(false);

   auto tabMain = std::make_unique<AtTabMain>();
   tabMain->SetMultiHit(100); // Set the maximum number of multihits in the visualization

   auto tabPad = std::make_unique<AtTabPad>(2, 2);
   tabPad->DrawRawADC(0, 0);
   tabPad->DrawADC(0, 1);
   tabPad->DrawAuxADC("IC", 1, 0);
   // tabPad->SetDrawArrayAug(2, "Qreco");

   eveMan->AddTab(std::move(tabMain));
   eveMan->AddTab(std::move(tabPad));

   auto method = std::make_unique<SampleConsensus::AtSampleConsensus>(
      SampleConsensus::Estimators::kRANSAC, AtPatterns::PatternType::kY, RandomSample::SampleMethod::kWeightedY);
   method->SetDistanceThreshold(20);
   method->SetNumIterations(500);
   method->SetMinHitsPattern(150);
   method->SetChargeThreshold(20); //-1 implies no charge-weighted fitting
   method->SetFitPattern(true);

   auto sacTask = new AtSampleConsensusTask(std::move(method));
   sacTask->SetPersistence(false);
   sacTask->SetInputBranch("AtEventH");

   eveMan->AddTask(sacTask);

   eveMan->Init();

   std::cout << "Finished init" << std::endl;
}
