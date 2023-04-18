/*#include "TString.h"
#include "AtEventDrawTask.h"
#include "AtEventManager.h"

#include "FairParRootFileIo.h"
#include "FairRunAna.h"
*/
#include "FairLogger.h"

#include "/mnt/simulations/attpcroot/adam/ATTPCROOTv2/macro/e12014/viewerUpdate/HEISTpid1.h"

void run_eve(TString species = "Bi200", int pressure = 150, TString OutputDataFile = "./data/output.reco_display.root")
{

   auto verbSpec =
      fair::VerbositySpec::Make(fair::VerbositySpec::Info::severity, fair::VerbositySpec::Info::file_line_function);
   fair::Logger::DefineVerbosity("user1", verbSpec);
   // fair::Logger::SetVerbosity("user1");
   //  fair::Logger::SetConsoleSeverity("debug");

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
   auto eveMan = new AtViewerManager(E12014::fMap);

   auto tabMain = std::make_unique<AtTabFission>();
   tabMain->SetMultiHit(100); // Set the maximum number of multihits in the visualization
   auto &fissionBranch = tabMain->GetFissionBranch();

   auto tabPad = std::make_unique<AtTabPad>(2, 2);
   tabPad->DrawRawADC(0, 0);
   tabPad->DrawADC(0, 1);
   tabPad->DrawAuxADC("IC", 1, 0);
   tabPad->DrawArrayAug("Qreco", 1, 1);
   tabPad->DrawHits(1, 1);

   eveMan->AddTab(std::move(tabMain));
   eveMan->AddTab(std::move(tabPad));
   eveMan->AddTab(std::make_unique<AtTabEnergyLoss>(fissionBranch));

   /*
   AtRawEvent *respAvgEvent;
   TFile *f2 = new TFile("respAvg.root");
   f2->GetObject("avgResp", respAvgEvent);
   f2->Close();


   // Create PSA and control for it
   auto psa = std::make_unique<AtPSADeconvFit>();
   psa->SetResponse(*respAvgEvent);
   psa->SetThreshold(15); // Threshold in charge units
   psa->SetFilterOrder(6);
   psa->SetCutoffFreq(75);
   auto sidePSA = new AtSidebarPSADeconv(eveMan->GetSidebar());
   sidePSA->SetPSA(psa.get());
   eveMan->GetSidebar()->AddSidebarFrame(sidePSA);

   // Add PSA task to run
   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetInputBranch("AtRawEventSub");
   eveMan->AddTask(psaTask);

   auto method = std::make_unique<SampleConsensus::AtSampleConsensus>(
      SampleConsensus::Estimators::kRANSAC, AtPatterns::PatternType::kLine, RandomSample::SampleMethod::kUniform);
   method->SetDistanceThreshold(20);
   method->SetNumIterations(200);
   method->SetMinHitsPattern(20);
   method->SetChargeThreshold(-1); //-1 implies no charge-weighted fitting
   method->SetFitPattern(true);

   auto sacTask = new AtSampleConsensusTask(std::move(method));
   sacTask->SetPersistence(false);
   sacTask->SetInputBranch("AtEvent");
   eveMan->AddTask(sacTask);

   auto method2 = std::make_unique<SampleConsensus::AtSampleConsensus>(
      SampleConsensus::Estimators::kRANSAC, AtPatterns::PatternType::kLine, RandomSample::SampleMethod::kUniform);
   method2->SetDistanceThreshold(20);
   method2->SetNumIterations(200);
   method2->SetMinHitsPattern(20);
   method2->SetChargeThreshold(15); //-1 implies no charge-weighted fitting
   method2->SetFitPattern(true);

   auto sacTask2 = new AtSampleConsensusTask(std::move(method2));
   sacTask2->SetPersistence(false);
   sacTask2->SetInputBranch("AtEventH");
   sacTask2->SetOutputBranch("AtPatternDeconv");
   eveMan->AddTask(sacTask2);
   */
   eveMan->Init();

   std::cout << "Finished init" << std::endl;
}
