// Code to take MC tracks and digitize
//#include "eventCombine.h"
#include "response.h"
#include "DigiSimInfo.h"

bool reduceFunc(AtRawEvent *evt);

void run_digi_fiss(int runNum = 0)
{
    auto verbSpec =
      fair::VerbositySpec::Make(fair::VerbositySpec::Info::severity, fair::VerbositySpec::Info::file_line_function);
   fair::Logger::DefineVerbosity("user1", verbSpec);
   fair::Logger::SetVerbosity("user1");

   TString inOutDir = "./data/";
   //TString outputFile = inOutDir + "output_digiLg.root";
   TString outputFile = inOutDir + TString::Format("output_digi%02d.root", runNum);
   outputFile = inOutDir + TString::Format("output_digi%02d.root", 1);
   TString scriptfile = "e12014_pad_mapping.xml";
   TString paramFile = "ATTPC.e12014.par";

   TString dir = getenv("VMCWORKDIR");

   // TString mcFile = "./data/sim_attpc.root";
   //TString mcFile = inOutDir + "symFissionLg.root";
   TString mcFile = inOutDir + TString::Format("simFission%02d.root", runNum);
   TString sharedInfoDir = "~/fission/data/e12014/tpcSharedInfo/";

   // Create the full parameter file paths
   //TString digiParFile = dir + "/parameters/" + paramFile;
   TString digiParFile = paramFile;
   TString mapParFile = dir + "/scripts/" + scriptfile;

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;

   // ------------------------------------------------------------------------

   // __ Run ____________________________________________
   FairRunAna *fRun = new FairRunAna();
   FairFileSource *source = new FairFileSource(mcFile);
   fRun->SetSource(source);
   fRun->SetOutputFile(outputFile);

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);

   // Create the detector map to pass to the simulation
   auto mapping = std::make_shared<AtTpcMap>();
   mapping->ParseXMLMap(mapParFile.Data());
   mapping->GeneratePadPlane();

   std::ifstream file(sharedInfoDir + "/e12014_zap.csv");
   if (!file.is_open())
      LOG(error) << "Failed to open smart zap file";

   // Clear out the header
   std::string temp;
   std::getline(file, temp);
   std::getline(file, temp);

   for (auto &row : CSVRange<int>(file)) {
      LOG(debug) << "Inhibiting " << row[4];
      mapping->InhibitPad(row[4], AtMap::InhibitType::kLowGain);
   }

   // __ AT digi tasks___________________________________
   // AtClusterizeTask *clusterizer = new AtClusterizeTask(std::make_shared<AtClusterize>());
   AtClusterizeTask *clusterizer = new AtClusterizeTask(std::make_shared<AtClusterizeLine>());
   clusterizer->SetPersistence(kFALSE);

   // AtPulseLineTask *pulse = new AtPulseLineTask();

   // AtPulseTask *pulse = new AtPulseTask(std::make_shared<AtPulse>(mapping));
   auto pulse = std::make_shared<AtPulseLine>(mapping, Response::GetResponse);
   pulse->SetSaveCharge(true);
   pulse->SetLowGain(0.19);
   //pulse->SetLowGain(1);
   Response::scaling = 0.01 * 0.75;
   AtPulseTask *pulseTask = new AtPulseTask(pulse);
   pulseTask->SetPersistence(kTRUE);

   /*AtMacroTask *combTask = new AtMacroTask();
   combTask->AddInitFunction(eventComb::init);
   combTask->AddFunction(eventComb::combine);*/

   AtDataReductionTask *reduceTask = new AtDataReductionTask();
   reduceTask->SetInputBranch("AtRawEvent");
   reduceTask->SetReductionFunction<AtRawEvent>(&reduceFunc);

   /**** PSA Task ****/
   AtRawEvent *respAvgEvent;
   TFile *f2 = new TFile(sharedInfoDir + "respAvg.root");
   f2->GetObject("avgResp", respAvgEvent);
   f2->Close();

   auto threshold = 50;
   auto maxThreshold = 1500;

   auto psa = std::make_unique<AtPSADeconvFit>();
   psa->SetResponse(*respAvgEvent);
   psa->SetThreshold(15); // Threshold in charge units
   psa->SetFilterOrder(6);
   psa->SetCutoffFreq(75);
   psa->SetThreshold(10);
   auto psaBeam = std::make_unique<AtPSATBAvg>();
   psaBeam->SetThreshold(45);
   auto psaComp = std::make_unique<AtPSAComposite>(std::move(psaBeam), std::move(psa), 20);
   AtPSAtask *psaTask = new AtPSAtask(std::move(psaComp));
   psaTask->SetInputBranch("AtRawEvent");
   psaTask->SetOutputBranch("AtEvent");
   psaTask->SetPersistence(true);

   /**** Space charge correction ****/
   // auto SCModel = std::make_unique<AtRadialChargeModel>(E12014SC(nsclRunNum));
   auto SCModel = std::make_unique<AtLineChargeModel>();
   //SCModel->SetLambda(E12014SC(nsclRunNum).GetLambda());
   // SCModel->SetStepSize(0.1);
   //SCModel->SetBeamLocation({-7, 10, 0}, {0, 7, 1000});
   SCModel->SetBeamLocation({0, -6, 0}, {10, 0, 1000});
   auto scTask = new AtSpaceChargeCorrectionTask(std::move(SCModel));
   scTask->SetInputBranch("AtEvent");
   scTask->SetOutputBranch("AtEventCorr");

   /**** 2 lines pattern fit ****/
   auto method = std::make_unique<SampleConsensus::AtSampleConsensus>(
      SampleConsensus::Estimators::kYRANSAC, AtPatterns::PatternType::kFission, RandomSample::SampleMethod::kY);
   method->SetDistanceThreshold(20);
   method->SetNumIterations(500);
   method->SetMinHitsPattern(150);
   method->SetChargeThreshold(10); //-1 implies no charge-weighted fitting
   method->SetFitPattern(true);
   auto sacTask = new AtSampleConsensusTask(std::move(method));
   sacTask->SetPersistence(false);
   sacTask->SetInputBranch("AtEvent");
   sacTask->SetOutputBranch("AtPatternEvent");

   /******** Create fission task ********/
   AtFissionTask *fissionTask = new AtFissionTask(0);
   fissionTask->SetUncorrectedEventBranch("AtEvent");
   fissionTask->SetPatternBranch("AtPatternEvent");
   fissionTask->SetOutBranch("AtFissionEvent");
   fissionTask->SetPersistance(true);


   AtMacroTask *infoTask = new AtMacroTask();
   infoTask->AddInitFunction(digiSimInfo::Init);

   fRun->AddTask(clusterizer);
   fRun->AddTask(pulseTask);
   fRun->AddTask(reduceTask);
   fRun->AddTask(psaTask);
   fRun->AddTask(sacTask);
   fRun->AddTask(fissionTask);
   fRun->AddTask(infoTask);

   //  __ Init and run ___________________________________

   Response::Init();
   fRun->Init();


   timer.Start();
   // fRun->Run(0, 20001);
   fRun->Run();
   timer.Stop();

   std::cout << std::endl << std::endl;
   std::cout << "Macro finished succesfully." << std::endl << std::endl;
   // -----   Finish   -------------------------------------------------------

   Double_t rtime = timer.RealTime();
   Double_t ctime = timer.CpuTime();
   cout << endl;
   cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
   cout << endl;
   // ------------------------------------------------------------------------
}

bool reduceFunc(AtRawEvent *evt)
{
   /*if(evt->GetEventID() % 2 == 0)
      return false;
   return (evt->GetNumPads() > 0) && evt->IsGood();*/
   return true;
}

