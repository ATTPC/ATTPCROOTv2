bool reduceFunc(AtRawEvent *evt);

void linkRunsTask(int tpcRunNum = 118, int nsclRunNum = 310)
{
   gSystem->Load("libAtReconstruction.so");

   TStopwatch timer;
   timer.Start();

   // Set the input/output directories
   TString inputDir = "/mnt/rawdata/e12014_attpc/h5";
   TString evtInputDir = "/mnt/analysis/e12014/HiRAEVT/mapped";
   TString outDir = "/mnt/analysis/e12014/TPC/unpackedLinked";
   TString evtOutDir = "/mnt/analysis/e12014/TPC/unpackedLinked";

   /**** Should not have to change code between this line and the next star comment ****/

   // Set the in/out files
   TString inputFile = inputDir + TString::Format("/run_%04d.h5", tpcRunNum);
   TString outputFile = outDir + TString::Format("/run_%04d.root", tpcRunNum);
   TString evtOutputFile = evtOutDir + TString::Format("/evtRun_%04d.root", tpcRunNum);
   TString nsclTreeFile = evtInputDir + TString::Format("/mappedRun-%d.root", nsclRunNum);

   std::cout << "Unpacking run " << tpcRunNum << " from: " << inputFile << std::endl;
   std::cout << "Saving in: " << outputFile << std::endl;
   std::cout << "Linking NSCL run: " << nsclRunNum << " from: " << nsclTreeFile << std::endl
             << "Saving in: " << evtOutputFile << std::endl;

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
   auto threshold = 45;

   // Add aux pads to map
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
   // AtDataReductionTask *reduceTask = new AtDataReductionTask();
   // reduceTask->SetInputBranch("AtRawEvent");
   // reduceTask->SetReductionFunction(&reduceFunc);

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

   AtLinkDAQTask *linker = new AtLinkDAQTask();
   auto success = linker->SetInputTree(nsclTreeFile, "E12014");
   linker->SetEvtOutputFile(evtOutputFile);
   linker->SetEvtTimestamp("tstamp");
   linker->SetTpcTimestampIndex(1);
   linker->SetSearchMean(1);
   linker->SetSearchRadius(2);
   linker->SetCorruptedSearchRadius(1000);

   // Add unpacker to the run
   run->AddTask(HDFParserTask);
   run->AddTask(filterTask);
   run->AddTask(auxFilterTask);
   run->AddTask(psaTask);
   run->AddTask(linker);

   run->Init();

   // Get the number of events and unpack the whole run
   auto numEvents = HDFParserTask->GetNumEvents() / 2;

   // numEvents = 5000;//217;
   // numEvents = 200;

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

bool checkEvent(AtRawEvent *evt)
{
   return (evt->GetNumPads() > 300);
}
