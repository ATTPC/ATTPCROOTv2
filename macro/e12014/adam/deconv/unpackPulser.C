// Unpacks tpc files from /mnt/rawdata/ to /mnt/analysis/e12014/TPC/unpacked

// Requires the TPC run number
void unpackPulser(int runNumber)
{
   // Load the library for unpacking and reconstruction
   gSystem->Load("libAtReconstruction.so");

   TStopwatch timer;
   timer.Start();

   // Set the input/output directories
   TString inputDir = "/mnt/analysis/hira_collaboration/e12014/Pulser/";
   TString outDir = "./";

   // Set the in/out files
   TString inputFile = inputDir + TString::Format("/run_%04d.h5", runNumber);
   TString outputFile = outDir + TString::Format("/run_%04dSub.root", runNumber);

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

   // Create the unpacker task
   auto unpacker = std::make_unique<AtHDFUnpacker>(fAtMapPtr);
   unpacker->SetInputFileName(inputFile.Data());
   unpacker->SetBaseLineSubtraction(true);
   unpacker->SetSaveFPN();

   auto unpackTask = new AtUnpackTask(std::move(unpacker));
   unpackTask->SetPersistence(true);

   // Attempt to restore the data
   auto filterTask = new AtFilterTask(new AtFilterZero());
   filterTask->SetFilterFPN(true);
   filterTask->SetPersistence(true);
   filterTask->SetOutputBranch("FilledData");

   auto filterTask2 = new AtFilterTask(new AtRemovePulser(1000, 15));
   filterTask2->SetFilterFPN(true);
   filterTask2->SetFilterPads(false);
   filterTask2->SetPersistence(true);
   filterTask2->SetInputBranch("FilledData");
   filterTask2->SetOutputBranch("NoPulserData");

   auto fpnFilter = new AtFilterFPN(fAtMapPtr, true);
   fpnFilter->SetThreshold(20);
   auto subtractionTask = new AtFilterTask(fpnFilter);
   subtractionTask->SetFilterFPN(true);
   subtractionTask->SetFilterPads(true);
   subtractionTask->SetPersistence(true);
   subtractionTask->SetInputBranch("NoPulserData");

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(-1);

   // Create PSA task
   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetPersistence(true);

   // Add unpacker to the run
   run->AddTask(unpackTask);
   run->AddTask(filterTask);
   run->AddTask(filterTask2);
   run->AddTask(subtractionTask);
   run->AddTask(psaTask);

   run->Init();

   // Get the number of events and unpack the whole run
   auto numEvents = unpackTask->GetNumEvents();

   // numEvents = 1700;//217;
   // numEvents = 1;

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
