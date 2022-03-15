// Unpacks tpc files from /mnt/rawdata/ to /mnt/analysis/e12014/TPC/unpacked

bool reduceFunc(AtRawEvent *evt);

// Requires the TPC run number
void unpack(int runNumber)
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
   TString outDir = "/mnt/analysis/e12014/TPC/unpacked";

   /**** Should not have to change code between this line and the next star comment ****/

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
   run->SetOutputFile(outputFile);
   run->SetGeomFile(geoManFile);

   // Set the parameter file
   FairRuntimeDb *rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();

   std::cout << "Setting par file: " << digiParFile << std::endl;
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setSecondInput(parIo1);

   // Create the detector map
   auto mapping = std::make_shared<AtTpcMap>();
   mapping->ParseXMLMap(mapDir.Data());
   mapping->GenerateAtTpc();

   /**** Should not have to change code between this line and the above star comment ****/
   mapping->AddAuxPad({10, 0, 0, 0}, "MCP_US");
   mapping->AddAuxPad({10, 0, 0, 34}, "TPC_Mesh");
   mapping->AddAuxPad({10, 0, 1, 0}, "MCP_DS");
   mapping->AddAuxPad({10, 0, 2, 34}, "IC");

   // Create the unpacker task
   AtHDFParserTask *HDFParserTask = new AtHDFParserTask();
   HDFParserTask->SetPersistence(kTRUE);
   HDFParserTask->SetMap(mapping);
   HDFParserTask->SetFileName(inputFile.Data());
   HDFParserTask->SetOldFormat(false);
   HDFParserTask->SetNumberTimestamps(2);
   HDFParserTask->SetBaseLineSubtraction(kTRUE);

   // Create data reduction task
   AtDataReductionTask *reduceTask = new AtDataReductionTask();
   reduceTask->SetInputBranch("AtRawEvent");
   reduceTask->SetReductionFunction(&reduceFunc);

   auto threshold = 45;

   AtFilterSubtraction *filter = new AtFilterSubtraction(mapping);
   filter->SetThreshold(threshold);
   AtFilterTask *filterTask = new AtFilterTask(filter);
   filterTask->SetPersistence(kTRUE);
   filterTask->SetFilterAux(true);

   AtTrapezoidFilter *auxFilter = new AtTrapezoidFilter();
   auxFilter->SetM(17.5);
   auxFilter->SetRiseTime(4);
   auxFilter->SetTopTime(10);
   AtAuxFilterTask *auxFilterTask = new AtAuxFilterTask(auxFilter);
   auxFilterTask->SetInputBranchName("AtRawEventFiltered");
   auxFilterTask->AddAuxPad("IC");

   AtPSASimple2 *psa = new AtPSASimple2();
   psa->SetThreshold(threshold);
   psa->SetMaxFinder();

   // Create PSA task
   AtPSAtask *psaTask = new AtPSAtask(psa);
   psaTask->SetInputBranch("AtRawEventFiltered");
   psaTask->SetOutputBranch("AtEventFiltered");
   psaTask->SetPersistence(kTRUE);

   AtRansacTask *ransacTask = new AtRansacTask();
   ransacTask->SetPersistence(kTRUE);
   ransacTask->SetInputBranch("AtEventFiltered");
   ransacTask->SetVerbose(kFALSE);
   ransacTask->SetDistanceThreshold(20.0);
   ransacTask->SetTiltAngle(0);
   ransacTask->SetMinHitsLine(10);
   ransacTask->SetFullMode();

   // Add unpacker to the run
   run->AddTask(HDFParserTask);
   run->AddTask(reduceTask);
   run->AddTask(filterTask);
   // run->AddTask(auxFilterTask);
   run->AddTask(psaTask);
   run->AddTask(ransacTask);

   run->Init();

   // Get the number of events and unpack the whole run
   auto numEvents = HDFParserTask->GetNumEvents() / 2;

   // numEvents = 1700;//217;
   numEvents = 200;

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

bool reduceFunc(AtRawEvent *evt)
{
   // return (evt->GetNumPads() > 0);
   return (evt->GetNumPads() > 250) && evt->IsGood();
}
