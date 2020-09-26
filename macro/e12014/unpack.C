//Unpacks tpc files from /mnt/rawdata/ to /mnt/analysis/e12014/TPC/unpacked

// Requires the TPC run number
void unpack(int runNumber)
{
  //Load the library for unpacking and reconstruction
  gSystem->Load("libATTPCReco.so");
  
  TStopwatch timer;
  timer.Start();
  
   
  //Set the input file
  TString inputFile = TString::Format("/mnt/rawdata/e12014_attpc/h5/run_%04d.h5", runNumber);

  //Set the output file
  TString outputFile = TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", runNumber);
  
  std::cout << "Unpacking run " << runNumber << " from: " << inputFile << std::endl;
  std::cout << "Saving in: " << outputFile << std::endl;

   
  //Set the mapping for the TPC
  TString scriptfile = "e12014_pad_mapping.xml";//"Lookup20150611.xml";
  TString parameterFile = "ATTPC.e12014.par";
   
  //Set directories
  TString dir = gSystem->Getenv("VMCWORKDIR");
  TString scriptdir = dir + "/scripts/"+ scriptfile;
  TString geomDir   = dir + "/geometry/";
  gSystem -> Setenv("GEOMPATH", geomDir.Data());
  TString digiParFile = dir + "/parameters/" + parameterFile;
  TString geoManFile  = dir + "/geometry/ATTPC_v1.1.root";


  //Create a run
  FairRunAna* run = new FairRunAna();
  run -> SetOutputFile(outputFile);
  run -> SetGeomFile(geoManFile);
   
  //Set the parameter file
  FairRuntimeDb* rtdb = run->GetRuntimeDb();
  FairParAsciiFileIo* parIo1 = new FairParAsciiFileIo();
  parIo1 -> open(digiParFile.Data(), "in");
  rtdb -> setSecondInput(parIo1);

  //Create the unpacker task
  ATHDFParserTask* HDFParserTask = new ATHDFParserTask();
  HDFParserTask->SetPersistence(kTRUE);
  HDFParserTask->SetATTPCMap(scriptdir.Data());
  HDFParserTask->SetFileName(inputFile.Data());
  HDFParserTask->SetOldFormat(false);
  HDFParserTask->SetTimestampIndex(2);
  HDFParserTask->SetBaseLineSubtraction(kTRUE);

  auto hash = HDFParserTask->CalculateHash(10,0,2,32);
  HDFParserTask->SetAuxChannel(hash, "IonCb_32");
  hash = HDFParserTask->CalculateHash(10,0,2,34);
  HDFParserTask->SetAuxChannel(hash, "IonCb_34");

  //Create PSA task
  ATPSATask *psaTask = new ATPSATask();
  psaTask -> SetPersistence(kTRUE);
  psaTask -> SetThreshold(0);
  psaTask -> SetPSAMode(1); //NB: 1 is ATTPC - 2 is pATTPC - 3 Filter for ATTPC - 4: Full Time Buckets
  psaTask -> SetMaxFinder();

  //ATRansacTask *RansacTask = new ATRansacTask();
  //RansacTask -> SetPersistence(kTRUE);
  //RansacTask -> SetVerbose(kFALSE);
  //RansacTask -> SetDistanceThreshold(20.0);
  //RansacTask -> SetTiltAngle(0);
  //RansacTask -> SetMinHitsLine(1000);
  //RansacTask -> SetFullMode();
  
  //Add unpacker to the run
  run -> AddTask(HDFParserTask);
  run -> AddTask(psaTask);
  //run -> AddTask(RansacTask);

  run -> Init();

  //Get the number of events and unpack the whole run
  auto numEvents = HDFParserTask->GetNumEvents()/2;

  //numEvents = 1700;//217;
  std::cout << "Unpacking " << numEvents << " events. " << std::endl;

  //return;
  run->Run(0,numEvents);


  std::cout << std::endl << std::endl;
  std::cout << "Done unpacking events"  << std::endl << std::endl;
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
