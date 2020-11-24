//Unpacks tpc files from /mnt/rawdata/ to /mnt/analysis/e12014/TPC/unpacked

// Requires the TPC run number
void unpack_new(int runNumber)
{
  //Load the library for unpacking and reconstruction
  gSystem->Load("libATTPCReco.so");

  TStopwatch timer;
  timer.Start();

  //Set the input file
  TString inputFile = TString::Format("hdf5Files/run_%04d.h5", runNumber);

  //Set the output file
  TString outputFile = TString::Format("rootFiles/run_unpacked_%04d_new.root", runNumber);

  std::cout << "Unpacking run " << runNumber << " from: " << inputFile << std::endl;
  std::cout << "Saving in: " << outputFile << std::endl;

  //Set the mapping for the TPC
  TString scriptfile = "e12014_pad_mapping.xml";//"Lookup20150611.xml";
 // TString scriptfile = "Lookup20150611.xml";//"Lookup20150611.xml";
  TString parameterFile = "ATTPC.testnew.par";

  //Set directories
  TString dir = gSystem->Getenv("VMCWORKDIR");
  TString scriptdir = dir + "/scripts/"+ scriptfile;
  TString geomDir   = dir + "/geometry/";
  gSystem -> Setenv("GEOMPATH", geomDir.Data());
  TString digiParFile = dir + "/parameters/" + parameterFile;
  //TString geoManFile  = dir + "/geometry/ATTPC_v1.1.root";
  //TString geoManFile  = dir + "/geometry/ATTPC_He1bar.root";
  TString geoManFile  = dir + "/geometry/ATTPC_d2He_07atm.root";


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
  HDFParserTask->SetPersistence(kFALSE);//kTRUE
  HDFParserTask->SetATTPCMap(scriptdir.Data());
  HDFParserTask->SetFileName(inputFile.Data());
  HDFParserTask->SetOldFormat(false);
  HDFParserTask->SetTimestampIndex(1);
  HDFParserTask->SetBaseLineSubtraction(kTRUE);

 // auto hash = HDFParserTask->CalculateHash(10,0,2,32);
 // HDFParserTask->SetAuxChannel(hash, "IonCb_32");
 // hash = HDFParserTask->CalculateHash(10,0,2,34);
 // HDFParserTask->SetAuxChannel(hash, "IonCb_34");


  //TString S800File = TString::Format("../Simulation/d2He/Analyis_d2He/rootS800/raw/test-runs800-48Ca-RAW-%04d.root", runNumber);
  TString S800File = "/mnt/simulations/ceclub/giraud/attpc/ATTPCROOTv2/macro/Simulation/d2He/Analyis_d2He/rootS800/cal/test-runs800-48Ca-CAL-0001.root";
  ATMergeTask *MergeEvt = new ATMergeTask();
  MergeEvt->SetS800File(S800File);
  MergeEvt->SetPersistence(kTRUE);
  MergeEvt->SetOptiEvtDelta(100);
  MergeEvt->SetPIDcut("/mnt/simulations/ceclub/giraud/attpc/ATTPCROOTv2/macro/Unpack_HDF5/rootFiles/cutest");

  //Create PSA task
  ATPSATask *psaTask = new ATPSATask();
  psaTask -> SetPersistence(kTRUE);//kFALSE
  psaTask -> SetThreshold(50);
  psaTask -> SetThresholdLow(50);
  psaTask -> SetPSAMode(1); //NB: 1 is ATTPC - 2 is pATTPC - 3 Filter for ATTPC - 4: Full Time Buckets
  psaTask -> SetMaxFinder();

/*
  ATRansacTask *RandTask = new ATRansacTask();
  RandTask -> SetPersistence(kTRUE);
  RandTask -> SetVerbose(kFALSE);
  RandTask -> SetDistanceThreshold(10.0);
  RandTask -> SetTiltAngle(0);
  RandTask -> SetMinHitsLine(30);
  //RandTask -> SetFullMode();
*/
  ATRansacTask *RandTask = new ATRansacTask();
  RandTask ->SetPersistence(kTRUE);
  //RandTask ->SetModelType(1);
  //RandTask ->SetFullMode();
  RandTask->SetTiltAngle(0.0);
  RandTask->SetDistanceThreshold(15.0);
  RandTask->SetMinHitsLine(7);
  RandTask->SetAlgorithm(3); // 0=PCL ransac; 1=Homemade Ransac; 2=Homemade Mlesac; 3=Homemade Lmeds;
  RandTask->SetRanSamMode(3);// 0=Uniform; 1=Gaussian; 2=Weighted; 3=Gaussian+Weighted



  //Add unpacker to the run
  run -> AddTask(HDFParserTask);
  run -> AddTask(MergeEvt);
  run -> AddTask(psaTask);
  run -> AddTask(RandTask);

  run -> Init();

  //Get the number of events and unpack the whole run
  auto numEvents = HDFParserTask->GetNumEvents()/2;

  //numEvents = 1700;//217;
  std::cout << "Unpacking " << numEvents << " events. " << std::endl;

  //return;
  run->Run(0,numEvents);
  //run->Run(0,7);


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
