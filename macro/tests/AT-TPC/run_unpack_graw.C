// Unpacks tpc files from /mnt/rawdata/ to /mnt/analysis/e12014/TPC/unpacked

// Requires the TPC run number
void run_unpack_graw(TString dataFile = "./pulser-files.txt", int runNumber = 36)
{
   // Load the library for unpacking and reconstruction
   gSystem->Load("libAtReconstruction.so");

   TStopwatch timer;
   timer.Start();

   // Set the input/output directories
   TString outDir = "./data/pulser";

   // Set the in/out files
   TString outputFile = outDir + TString::Format("/run_%04dSub.root", runNumber);

   // std::cout << "Unpacking run " << runNumber << " from: " << inputFile << std::endl;
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
   auto unpacker = std::make_unique<AtGRAWUnpacker>(fAtMapPtr, 36);
   unpacker->SetInputFileName(dataFile.Data(), "file%i");
   unpacker->SetInitialEventID(0);
   unpacker->SetSaveFPN();
   unpacker->SetSaveLastCell(true);
   unpacker->SetSubtractFPN(false);

   auto unpackTask = new AtUnpackTask(std::move(unpacker));
   unpackTask->SetPersistence(true);

   auto filterTask = new AtFilterTask(new AtSCACorrect(
      fAtMapPtr, "/mnt/analysis/e12014/huntc/newFork/ATTPCROOTv2/code/unpacking/background.root", "background",
      "/mnt/analysis/e12014/huntc/newFork/ATTPCROOTv2/code/unpacking/phase.root", "phase"));
   filterTask->SetPersistence(true);
   filterTask->SetOutputBranch("CorrectedData");

   // Add unpacker to the run
   run->AddTask(unpackTask);
   run->AddTask(filterTask);

   run->Init();

   // Get the number of events and unpack the whole run
   auto numEvents = unpackTask->GetNumEvents();

   // numEvents = 1700;//217;
   // numEvents = 1;

   std::cout << "Unpacking " << numEvents << " events. " << std::endl;

   // return;
   // run->Run(0, numEvents);
   run->Run(0, 2);

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
