bool reduceFunc(AtRawEvent *evt)
{
   return (evt->GetNumPads() > 300) && evt->IsGood();
}

// Requires the TPC run number
void run_unpack_attpc(int runNumber = 210)
{
   // Load the library for unpacking and reconstruction
   gSystem->Load("libAtReconstruction.so");

   TStopwatch timer;
   timer.Start();

   // Set the input/output directories
   TString inputDir = "/mnt/rawdata/e12014_attpc/h5";
   // TString inputDir = "/mnt/analysis/attpc";
   TString outDir = "/mnt/analysis/e12014/TPC/filterTesting";

   // Set the in/out files
   TString inputFile = inputDir + TString::Format("/run_%04d.h5", runNumber);
   TString outputFile = outDir + TString::Format("/run_%04d.root", runNumber);

   std::cout << "Unpacking run " << runNumber << " from: " << inputFile << std::endl;
   std::cout << "Saving in: " << outputFile << std::endl;

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
   FairRunAna *run = new FairRunAna();
   run->SetSink(new FairRootFileSink(outputFile));
   run->SetGeomFile(geoManFile);

   // Set the parameter file
   FairRuntimeDb *rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();

   std::cout << "Setting par file: " << digiParFile << std::endl;
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);
   std::cout << "Getting containers..." << std::endl;
   // We must get the container before initializing a run
   rtdb->getContainer("AtDigiPar");

   // Create the detector map
   auto fAtMapPtr = std::make_shared<AtTpcMap>();
   fAtMapPtr->ParseXMLMap(mapDir.Data());
   fAtMapPtr->GeneratePadPlane();

   fAtMapPtr->AddAuxPad({10, 0, 0, 0}, "MCP_US");
   fAtMapPtr->AddAuxPad({10, 0, 0, 34}, "TPC_Mesh");
   fAtMapPtr->AddAuxPad({10, 0, 1, 0}, "MCP_DS");
   fAtMapPtr->AddAuxPad({10, 0, 2, 34}, "IC");

   // Create the unpacker task
   auto unpacker = std::make_unique<AtHDFUnpacker>(fAtMapPtr);
   unpacker->SetInputFileName(inputFile.Data());
   unpacker->SetNumberTimestamps(2);
   unpacker->SetBaseLineSubtraction(true);

   auto unpackTask = new AtUnpackTask(std::move(unpacker));
   unpackTask->SetPersistence(true);
   run->AddTask(unpackTask);

   // Create data reduction task
   AtDataReductionTask *reduceTask = new AtDataReductionTask();
   reduceTask->SetInputBranch("AtRawEvent");
   reduceTask->SetReductionFunction(&reduceFunc);
   run->AddTask(reduceTask);

   auto threshold = 45;

   auto *filterCh0 = new AtFilterSubtraction(fAtMapPtr);
   filterCh0->SetThreshold(threshold);
   filterCh0->SetIsGood(false);

   AtFilterTask *filterTask = new AtFilterTask(filterCh0, "filterCh0");
   filterTask->SetPersistence(true);
   filterTask->SetFilterAux(false);
   filterTask->SetInputBranch("AtRawEvent");
   filterTask->SetOutputBranch("AtRawEventSubtracted");
   // run->AddTask(filterTask);

   auto *filterFFTRaw = new AtFilterFFT();
   filterFFTRaw->SetSaveTransform(true);
   filterFFTRaw->AddFreqRange({0, 0.4, 1, 0.95});
   filterFFTRaw->AddFreqRange({1, 0.95, 4, 1});
   filterFFTRaw->AddFreqRange({4, 1, 25, 1});
   filterFFTRaw->AddFreqRange({25, 1, 90, 0.7});
   filterFFTRaw->AddFreqRange({90, 0.7, 257, 0.7});
   filterFFTRaw->DumpFactors();

   AtFilterTask *fftTaskRaw = new AtFilterTask(filterFFTRaw);
   fftTaskRaw->SetPersistence(true);
   fftTaskRaw->SetFilterAux(false);
   fftTaskRaw->SetInputBranch("AtRawEvent");
   fftTaskRaw->SetOutputBranch("AtRawEventFFT");
   run->AddTask(fftTaskRaw);

   /*
   auto *filterFFTSub = new AtFilterFFT();
   filterFFTSub->SetSaveTransform(true);
   filterFFTSub->DumpFactors();

   AtFilterTask *fftTaskSub = new AtFilterTask(filterFFTSub);
   fftTaskSub->SetPersistence(false);
   fftTaskSub->SetFilterAux(false);
   fftTaskSub->SetInputBranch("AtRawEventSubtracted");
   fftTaskSub->SetOutputBranch("AtRawEventFFTSub");
   // run->AddTask(fftTaskSub);
   */

   AtPSASimple2 *psa = new AtPSASimple2();
   psa->SetThreshold(0);
   psa->SetMaxFinder();

   AtPSAtask *psaTask = new AtPSAtask(psa);
   psaTask->SetPersistence(true);
   run->AddTask(psaTask);

   AtPSAtask *psaTaskSub = new AtPSAtask(psa);
   psaTaskSub->SetPersistence(true);
   psaTaskSub->SetInputBranch("AtRawEventSubtracted");
   psaTaskSub->SetOutputBranch("AtEventSubstracted");
   run->AddTask(psaTaskSub);

   AtPSAtask *psaTaskFFT = new AtPSAtask(psa);
   psaTaskFFT->SetPersistence(true);
   psaTaskFFT->SetInputBranch("AtRawEventFFT");
   psaTaskFFT->SetOutputBranch("AtEventFFT");
   run->AddTask(psaTaskFFT);

   std::cout << "***** Starting Init ******" << std::endl;
   run->Init();
   std::cout << "***** Ending Init ******" << std::endl;

   // Get the number of events and unpack the whole run
   auto numEvents = unpackTask->GetNumEvents();

   // numEvents = 1700;//217;
   // numEvents = 100;

   std::cout << "Unpacking " << numEvents << " events. " << std::endl;

   // return;
   run->Run(0, numEvents);

   std::cout << std::endl << std::endl;
   std::cout << "Done unpacking events" << std::endl << std::endl;
   std::cout << "- Output file : " << outputFile << std::endl << std::endl;
   // -----   Finish   -------------------------------------------------------
   timer.Stop();
   Double_t rtime = timer.RealTime();
   Double_t ctime = timer.CpuTime();
   cout << endl << endl;
   cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
   cout << endl;
   // ------------------------------------------------------------------------

   return 0;
}
