// Unpacks tpc files from /mnt/rawdata/ to /mnt/analysis/e12014/TPC/unpacked
bool reduceFunc(AtRawEvent *evt)
{
   return (evt->GetNumPads() > 300) && evt->IsGood();
}

// Requires the TPC run number
void unpackReducedCalibrated(int runNumber)
{
   // Load the library for unpacking and reconstruction
   gSystem->Load("libAtReconstruction.so");

   TStopwatch timer;
   timer.Start();

   // Set the input/output directories
   TString inputDir = "/mnt/rawdata/e12014_attpc/h5";
   TString outDir = "/mnt/analysis/e12014/TPC/unpackedCalibrated";

   /**** Should not have to change code between this line and the next star comment ****/

   // Set the in/out files
   TString inputFile = inputDir + TString::Format("/run_%04d.h5", runNumber);
   TString outputFile = outDir + TString::Format("/run_%04dReduced.root", runNumber);

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
   run->SetOutputFile(outputFile);
   run->SetGeomFile(geoManFile);

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

   // Create the unpacker task
   auto unpacker = std::make_unique<AtHDFUnpacker>(mapping);
   unpacker->SetInputFileName(inputFile.Data());
   unpacker->SetNumberTimestamps(2);
   unpacker->SetBaseLineSubtraction(true);

   auto unpackTask = new AtUnpackTask(std::move(unpacker));
   unpackTask->SetPersistence(false);

   AtDataReductionTask *reduceTask = new AtDataReductionTask();
   reduceTask->SetInputBranch("AtRawEvent");
   reduceTask->SetReductionFunction(&reduceFunc);

   auto threshold = 45;

   auto filterSub = new AtFilterSubtraction(mapping);
   filterSub->SetThreshold(threshold);
   AtFilterTask *subTask = new AtFilterTask(filterSub);
   subTask->SetPersistence(false);
   subTask->SetFilterAux(true);
   subTask->SetOutputBranch("AtRawEventSub");

   AtFilterCalibrate *filterCal = new AtFilterCalibrate();
   filterCal->SetCalibrationFile("calibrationFormated.txt");
   AtFilterTask *calTask = new AtFilterTask(filterCal);
   calTask->SetPersistence(true);
   calTask->SetFilterAux(false);
   calTask->SetInputBranch("AtRawEventSub");
   calTask->SetOutputBranch("AtRawEventCal");

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(threshold);

   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetInputBranch("AtRawEventCal");
   psaTask->SetOutputBranch("AtEventH");
   psaTask->SetPersistence(true);

   // Add unpacker to the run
   run->AddTask(unpackTask);
   run->AddTask(reduceTask);
   run->AddTask(subTask);
   run->AddTask(calTask);
   run->AddTask(psaTask);

   run->Init();

   // Get the number of events and unpack the whole run
   auto numEvents = unpackTask->GetNumEvents();

   // numEvents = 1700;//217;
   // numEvents = 20;

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
