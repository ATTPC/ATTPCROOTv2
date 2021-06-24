
void linkRunsTask(int tpcRunNum = 118, int nsclRunNum = 310)
{


   gSystem->Load("libAtReconstruction.so");

   TStopwatch timer;
   timer.Start();

   // Set the input/output directories
   TString inputDir = "/mnt/rawdata/e12014_attpc/h5";
   TString outDir = "./";
   
   // Set the in/out files
   TString inputFile = inputDir + TString::Format("/run_%04d.h5", tpcRunNum);
   TString outputFile = outDir + TString::Format("/run_%04d.root", tpcRunNum);
   TString evtOutputFile = outDir + TString::Format("/evtRun_%04d.root", tpcRunNum);
   TString nsclTreeFile = TString::Format("/mnt/analysis/e12014/HiRAEVT/mapped/mappedRun-%d.root", nsclRunNum);

   std::cout << "Unpacking run " << tpcRunNum << " from: " << inputFile << std::endl;
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

   // Create the unpacker task
   AtHDFParserTask *HDFParserTask = new AtHDFParserTask();
   HDFParserTask->SetPersistence(kFALSE);
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

   AtLinkDAQTask *linker = new AtLinkDAQTask();
   linker->SetPersistance(true);
   auto success = linker->SetInputTree(nsclTreeFile, "E12014");
   linker->SetEvtOutputFile(evtOutputFile);
   linker->SetEvtTimestamp("tstamp");
   linker->SetTpcTimestampIndex(1);
   linker->SetSearchMean(1);
   linker->SetSearchRadius(2);

   
   // Add unpacker to the run
   run->AddTask(HDFParserTask);
   run->AddTask(linker);

   run->Init();

   // Get the number of events and unpack the whole run
   auto numEvents = HDFParserTask->GetNumEvents() / 2;

   // numEvents = 1700;//217;
   //numEvents = 10;
   
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


