#include <FairFileSource.h>
#include <FairParAsciiFileIo.h>
#include <FairRootFileSink.h>
#include <FairRuntimeDb.h>

#include <TChain.h>
#include <TFile.h>
#include <TStopwatch.h>
#include <TString.h>

#include "SpaceChargeModel.h"
#include "TxtEvent.h"
#ifndef __CLING__
#include "../build/include/AtCopyTreeTask.h"
#include "../build/include/AtCutHEIST.h"
#include "../build/include/AtDataReductionTask.h"
#include "../build/include/AtFilterCalibrate.h"
#include "../build/include/AtFilterSubtraction.h"
#include "../build/include/AtFilterTask.h"
#include "../build/include/AtFissionTask.h"
#include "../build/include/AtHDFUnpacker.h"
#include "../build/include/AtLineChargeModel.h"
#include "../build/include/AtLinkDAQTask.h"
#include "../build/include/AtPSA.h"
#include "../build/include/AtPSAComposite.h"
#include "../build/include/AtPSADeconvFit.h"
#include "../build/include/AtPSATBAvg.h"
#include "../build/include/AtPSAtask.h"
#include "../build/include/AtRadialChargeModel.h"
#include "../build/include/AtRunAna.h"
#include "../build/include/AtSampleConsensus.h"
#include "../build/include/AtSampleConsensusTask.h"
#include "../build/include/AtSpaceChargeCorrectionTask.h"
#include "../build/include/AtTpcMap.h"
#include "../build/include/AtUnpackTask.h"
#endif

/**
 * This will unpack a run file with every fission event in it that can be matched to HEIST.
 * The output tree structure has these branches (branch name: description)
 *
 * AtRawEvent: AtRawEvent with ch0 subtraction and calibration applied.
 * AtEvent: AtEvent with no space charge correction.
 * AtEventCorr: AtEvent with space charge correction.
 * AtPatternEvent: AtPatternEvent with space charge correction.
 * AtFissionEvent: AtFissionEvent constructed from AtPatternEvent and AtEvent.
 *
 */

void unpack_linked(int tpcRunNum = 130)
{
   auto verbSpec =
      fair::VerbositySpec::Make(fair::VerbositySpec::Info::severity, fair::VerbositySpec::Info::file_line_function);
   fair::Logger::DefineVerbosity("user1", verbSpec);
   // fair::Logger::SetVerbosity("user1");
   // fair::Logger::SetConsoleSeverity("debug");

   // Load the library for unpacking and reconstruction
   gSystem->Load("libAtReconstruction.so");

   TStopwatch timer;
   timer.Start();

   // Set the input/output directories
   TString inputDir = "/mnt/rawdata/e12014_attpc/h5";
   TString evtInputDir = "/mnt/analysis/e12014/HiRAEVT/mapped";
   // TString outDir = "/mnt/analysis/e12014/TPC/fission_linked";
   TString outDir = "/mnt/analysis/e12014/TPC/fission_linked_baseline/";
   // TString outDir = "./";
   TString evtOutDir = outDir;
   TString sharedInfoDir = "/mnt/projects/hira/e12014/tpcSharedInfo/";

   /**** Should not have to change code between this line and the next star comment ****/

   EventMap eventMap(sharedInfoDir + "RunMap.csv");
   int nsclRunNum = eventMap.GetNsclRunNum(tpcRunNum);
   if (nsclRunNum == -1)
      throw std::invalid_argument("No matching NSCL run number");

   // Set the in/out files
   TString inputFile = inputDir + TString::Format("/run_%04d.h5", tpcRunNum);
   TString outputFile = outDir + TString::Format("/run_%04d.root", tpcRunNum);
   TString evtOutputFile = evtOutDir + TString::Format("/evtRun_%04d.root", tpcRunNum);
   TString evtInputFile = evtInputDir + TString::Format("/mappedRun-%d.root", nsclRunNum);

   std::cout << "Unpacking run " << tpcRunNum << " from: " << inputFile << std::endl;
   std::cout << "Saving in: " << outputFile << std::endl;
   std::cout << "Linking to run " << nsclRunNum << " from: " << evtInputFile << std::endl;
   std::cout << "Saving in: " << evtOutputFile << std::endl;

   // Set the mapping for the TPC
   TString mapFile = "e12014_pad_mapping.xml"; //"Lookup20150611.xml";
   TString parameterFile = "ATTPC.e12014.par";

   // Set directories
   TString dir = gSystem->Getenv("VMCWORKDIR");
   TString mapDir = dir + "/scripts/" + mapFile;
   TString geomDir = dir + "/geometry/";
   gSystem->Setenv("GEOMPATH", geomDir.Data());
   TString digiParFile = dir + "/parameters/" + parameterFile;
   TString geoManFile = dir + "/geometry/ATTPC_v1.1.root";

   // Create a run
   AtRunAna *run = new AtRunAna();
   run->SetSink(new FairRootFileSink(outputFile));
   run->SetGeomFile(geoManFile);
   run->SetRunId(tpcRunNum);

   // Set the parameter file
   FairRuntimeDb *rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();

   std::cout << "Setting par file: " << digiParFile << std::endl;
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);
   rtdb->getContainer("AtDigiPar");

   // Create the detector map
   auto mapping = std::make_shared<AtTpcMap>();
   mapping->ParseXMLMap(mapDir.Data());
   mapping->GeneratePadPlane();

   /**** Should not have to change code between this line and the above star comment ****/
   mapping->AddAuxPad({10, 0, 0, 0}, "MCP_US");
   mapping->AddAuxPad({10, 0, 0, 34}, "TPC_Mesh");
   mapping->AddAuxPad({10, 0, 1, 0}, "MCP_DS");
   mapping->AddAuxPad({10, 0, 2, 34}, "IC");

   /**** HDF5 unpacker ****/
   auto unpacker = std::make_unique<AtHDFUnpacker>(mapping);
   unpacker->SetInputFileName(inputFile.Data());
   unpacker->SetNumberTimestamps(2);
   unpacker->SetBaseLineSubtraction(true);
   auto unpackTask = new AtUnpackTask(std::move(unpacker));
   unpackTask->SetOuputBranchName("AtRawEventRaw");
   unpackTask->SetPersistence(true);

   /**** Data reduction task (keep fission only) ****/
   AtDataReductionTask *reduceTask = new AtDataReductionTask();
   reduceTask->SetInputBranch("AtRawEventRaw");
   TxtEvents events;
   events.AddTxtFile(TString::Format(sharedInfoDir + "/EventLabels/fissionEventsRun_%04d", tpcRunNum).Data());
   reduceTask->SetReductionFunction(events);

   /**** DAQ linking task  ****/
   AtLinkDAQTask *linker = new AtLinkDAQTask(); //< Must run after the data reduction task!!!
   auto success = linker->SetInputTree(evtInputFile, "E12014");
   linker->SetEvtOutputFile(evtOutputFile);
   linker->SetEvtTimestamp("tstamp");
   linker->SetTpcTimestampIndex(1);
   linker->SetSearchMean(1);
   linker->SetSearchRadius(2);
   linker->SetCorruptedSearchRadius(1000);
   linker->SetInputBranch("AtRawEventRaw");

   /**** Ch0 subtraction ****/
   auto filterSub = new AtFilterSubtraction(mapping);
   filterSub->SetThreshold(25);
   filterSub->SetIsGood(false); // Save events event if
   AtFilterTask *subTask = new AtFilterTask(filterSub);
   subTask->SetPersistence(false);
   subTask->SetFilterAux(true);
   subTask->SetInputBranch("AtRawEventRaw");
   subTask->SetOutputBranch("AtRawEventSub");

   /**** Calibration Task ****/
   auto filterCal = new AtFilterCalibrate();
   filterCal->SetCalibrationFile(sharedInfoDir + "/calibrationFormated.txt");
   AtFilterTask *calTask = new AtFilterTask(filterCal);
   calTask->SetPersistence(true);
   calTask->SetFilterAux(false);
   calTask->SetInputBranch("AtRawEventRaw");
   calTask->SetOutputBranch("AtRawEvent");

   /**** PSA Task ****/
   AtRawEvent *respAvgEvent;
   TFile *f2 = new TFile(sharedInfoDir + "respAvg.root");
   f2->GetObject("avgResp", respAvgEvent);
   f2->Close();

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
   psaTask->SetInputBranch("AtRawEventRaw");
   psaTask->SetOutputBranch("AtEvent");
   psaTask->SetPersistence(true);

   /**** Space charge correction ****/
   // auto SCModel = std::make_unique<AtRadialChargeModel>(E12014SC(nsclRunNum));
   auto SCModel = std::make_unique<AtLineChargeModel>();
   SCModel->SetLambda(E12014SC(nsclRunNum).GetLambda());
   // SCModel->SetStepSize(0.1);
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
   sacTask->SetInputBranch("AtEventCorr");
   sacTask->SetOutputBranch("AtPatternEvent");

   /******** Create fission task ********/
   AtFissionTask *fissionTask = new AtFissionTask(E12014SC(nsclRunNum).GetLambda());
   fissionTask->SetUncorrectedEventBranch("AtEvent");
   fissionTask->SetPatternBranch("AtPatternEvent");
   fissionTask->SetOutBranch("AtFissionEvent");
   fissionTask->SetPersistance(true);

   run->AddTask(unpackTask);
   run->AddTask(reduceTask);
   run->AddTask(linker);
   // run->AddTask(subTask);
   // run->AddTask(calTask);
   run->AddTask(psaTask);
   run->AddTask(scTask);
   run->AddTask(sacTask);
   run->AddTask(fissionTask);

   run->Init();

   // Get the number of events and unpack the whole run
   auto numEvents = unpackTask->GetNumEvents();

   // numEvents = 1700;//217;
   numEvents = 100;

   std::cout << "Unpacking " << numEvents << " events. " << std::endl;
   // numEvents = 3800;
   //      return;
   run->Run(0, numEvents);

   std::cout << std::endl << std::endl;
   std::cout << "Done unpacking events" << std::endl << std::endl;
   std::cout << "- Output file : " << outputFile << std::endl << std::endl;
   // -----   Finish   -------------------------------------------------------
   timer.Stop();
   Double_t rtime = timer.RealTime();
   Double_t ctime = timer.CpuTime();
   using std::cout;
   using std::endl;
   cout << endl << endl;
   cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
   cout << endl;
   // ------------------------------------------------------------------------
}
