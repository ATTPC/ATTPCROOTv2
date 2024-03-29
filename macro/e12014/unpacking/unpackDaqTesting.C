// Unpacks tpc files from /mnt/daqtesting/ to /mnt/analysis/e12014/TPC/unpacked

// Requires the TPC run number
void unpackDaqTesting(int runNumber)
{
   // Load the library for unpacking and reconstruction
   gSystem->Load("libAtReconstruction.so");

   TStopwatch timer;
   timer.Start();

   // Set the input file
   TString inputFile = TString::Format("/mnt/daqtesting/e12014_attpc_transfer/h5/run_%04d.h5", runNumber);

   // Set the output file
   TString outputFile = TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", runNumber);

   std::cout << "Unpacking run " << runNumber << " from: " << inputFile << std::endl;
   std::cout << "Saving in: " << outputFile << std::endl;

   // Set the mapping for the TPC
   TString scriptfile = "e12014_pad_mapping.xml"; //"Lookup20150611.xml";
   TString parameterFile = "ATTPC.e12014.par";

   // Set directories
   TString dir = gSystem->Getenv("VMCWORKDIR");
   TString scriptdir = dir + "/scripts/" + scriptfile;
   TString geomDir = dir + "/geometry/";
   gSystem->Setenv("GEOMPATH", geomDir.Data());

   TString digiParFile = dir + "/parameters/" + parameterFile;
   TString geoManFile = dir + "/geometry/ATTPC_v1.1.root";
   std::cout << "Setting par file: " << digiParFile << std::endl;

   // Create a run
   FairRunAna *run = new FairRunAna();
   run->SetOutputFile(outputFile);
   run->SetGeomFile(geoManFile);

   // Set the parameter file
   FairRuntimeDb *rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setSecondInput(parIo1);

   // Create the unpacker task
   AtHDFParserTask *HDFParserTask = new AtHDFParserTask();
   HDFParserTask->SetPersistence(kTRUE);
   HDFParserTask->SetAtTPCMap(scriptdir.Data());
   HDFParserTask->SetFileName(inputFile.Data());
   HDFParserTask->SetOldFormat(false);
   HDFParserTask->SetNumberTimestamps(2);
   HDFParserTask->SetBaseLineSubtraction(kTRUE);

   // Add the aux channels from the experiment
   auto hash = HDFParserTask->CalculateHash(10, 0, 0, 0);
   HDFParserTask->SetAuxChannel(hash, "MCP_US");
   hash = HDFParserTask->CalculateHash(10, 0, 0, 34);
   HDFParserTask->SetAuxChannel(hash, "TPC_Mesh");
   hash = HDFParserTask->CalculateHash(10, 0, 1, 0);
   HDFParserTask->SetAuxChannel(hash, "MCP_DS");
   hash = HDFParserTask->CalculateHash(10, 0, 2, 34);
   HDFParserTask->SetAuxChannel(hash, "IC");

   // Create PSA task
   AtPSASimple2 *psa = new AtPSASimple2();
   psa->SetThreshold(0);
   psa->SetThresholdLow(0);
   psa->SetMaxFinder();

   AtPSAtask *psaTask = new AtPSAtask(psa);
   psaTask->SetPersistence(kTRUE);

   // ATRansacTask *RansacTask = new ATRansacTask();
   // RansacTask -> SetPersistence(kTRUE);
   // RansacTask -> SetVerbose(kFALSE);
   // RansacTask -> SetDistanceThreshold(20.0);
   // RansacTask -> SetTiltAngle(0);
   // RansacTask -> SetMinHitsLine(1000);
   // RansacTask -> SetFullMode();

   // Add unpacker to the run
   run->AddTask(HDFParserTask);
   run->AddTask(psaTask);
   // run -> AddTask(RansacTask);

   run->Init();

   // Get the number of events and unpack the whole run
   auto numEvents = HDFParserTask->GetNumEvents() / 2;

   // numEvents = 1700;//217;
   numEvents = 10;
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
