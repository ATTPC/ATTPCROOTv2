#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"


void run_unpack_Mg22elastic(std::string dataFile = "/media/dani/ubuntu/f5files/run_0290.h5",TString parameterFile = "ATTPC.e15250.par",TString mappath="")
{
 
  // -----   Timer   --------------------------------------------------------
 TStopwatch timer;
 timer.Start();
 // ------------------------------------------------------------------------

  gSystem->Load("libXMLParser.so");
  // -----------------------------------------------------------------
  // Set file names
  TString scriptfile = "Lookup20150611.xml";
  TString dir = getenv("VMCWORKDIR");
  TString scriptdir = dir + "/scripts/"+ scriptfile;
  TString dataDir = dir + "/macro/data/";
  TString geomDir = dir + "/geometry/";
  gSystem -> Setenv("GEOMPATH", geomDir.Data());


  TString outputFile  = "/media/dani/ubuntuwindows/GitHub_Yassidfork/ATTPCROOTv2/macro/Unpack_HDF5/e15250/Hierar_Dani/realdata_and_unpack/run_0290.root";
  TString loggerFile  = dataDir + "ATTPCLog.log";
  TString digiParFile = dir + "/parameters/" + parameterFile;
  TString geoManFile  = dir + "/geometry/ATTPC_Mg22elastic.root";

  TString inimap   = mappath + "inhib.txt";
  TString lowgmap  = mappath + "lowgain.txt";
  TString xtalkmap = mappath + "beampads_e15503b.txt";

  // -----------------------------------------------------------------
  FairRunAna* run = new FairRunAna();
  run -> SetOutputFile(outputFile);
  run -> SetGeomFile(geoManFile);

  FairRuntimeDb* rtdb = run->GetRuntimeDb();
  FairParAsciiFileIo* parIo1 = new FairParAsciiFileIo();
  parIo1 -> open(digiParFile.Data(), "in");
  //FairParRootFileIo* parIo2 = new FairParRootFileIo();
  //parIo2 -> open("param.dummy_proto.root");
 // rtdb -> setFirstInput(parIo2);
  rtdb -> setSecondInput(parIo1);

  ATHDFParserTask* HDFParserTask = new ATHDFParserTask();
  HDFParserTask->SetPersistence(kTRUE);
  HDFParserTask->SetATTPCMap(scriptdir.Data());
  HDFParserTask->SetFileName(dataFile);

  ATPSATask *psaTask = new ATPSATask();
  psaTask -> SetPersistence(kTRUE);
  psaTask -> SetThreshold(0);//Threshold in the charge
  psaTask -> SetPSAMode(6); //NB: 1 is ATTPC - 2 is pATTPC - 3 Filter for ATTPC - 4: Full Time Buckets - 6 ATTPC with FFT
  //psaTask -> SetPeakFinder(); //NB: Use either peak finder of maximum finder but not both at the same time
  psaTask -> SetMaxFinder();
  psaTask -> SetBaseCorrection(kTRUE); //Directly apply the base line correction to the pulse amplitude to correct for the mesh induction. If false the correction is just saved
  //psaTask -> SetTimeCorrection(kFALSE); //Interpolation around the maximum of the signal peak

  ATPRATask *praTask = new ATPRATask();
  praTask -> SetPersistence(kTRUE);


  //ATAnalysisTask *AnalysisTask = new ATAnalysisTask();
  //AnalysisTask -> SetPersistence(kTRUE);
/*  ATHierarchicalClusteringTask *hierarchicalClusteringTask = new ATHierarchicalClusteringTask();
 
	// optional: set different parameters
	hierarchicalClusteringTask->SetBestClusterDistanceDelta(2.0f);
	hierarchicalClusteringTask->SetCleanupMinTriplets(40);
	hierarchicalClusteringTask->SetCloudScaleModifier(4.0f);
	hierarchicalClusteringTask->SetGenTripletsMaxError(0.01f);
	hierarchicalClusteringTask->SetGenTripletsNnKandidates(10);
	hierarchicalClusteringTask->SetGenTripletsNBest(2);
	hierarchicalClusteringTask->SetSmoothRadius(5.0f);
	hierarchicalClusteringTask->SetSplineTangentScale(0.5f);
	hierarchicalClusteringTask->SetSplineMinControlPointDistance(30.0f);
	hierarchicalClusteringTask->SetSplineJump(1);

  run -> AddTask(hierarchicalClusteringTask); 

   //Moved to analysis macro!
   ATPhiRecoTask *phirecoTask = new ATPhiRecoTask();
   phirecoTask -> SetPersistence();
   run -> AddTask(phirecoTask);

   ATHoughTask *HoughTask = new ATHoughTask();
   HoughTask->SetPhiReco();
   HoughTask->SetPersistence();
   //HoughTask->SetLinearHough();
   HoughTask->SetRadiusThreshold(3.0); // Truncate Hough Space Calculation
   HoughTask ->SetHoughThreshold(150.0); // Charge threshold for Hough
   HoughTask ->SetHoughDistance(0.25);
   //HoughTask ->SetEnableMap(); //Enables an instance of the ATTPC map:  This enables the MC with Q instead of position
   HoughTask ->SetCircularHough();
   run ->AddTask(HoughTask);



  ATRansacTask *RansacTask = new ATRansacTask();
  RansacTask->SetPersistence(kTRUE);
  RansacTask->SetModelType(pcl::SACMODEL_CIRCLE2D);
  RansacTask->SetDistanceThreshold(0.5);
  RansacTask->SetFullMode();
  RansacTask->SetMinHitsLine(15);
*/  
  run -> AddTask(HDFParserTask);
  run -> AddTask(psaTask);
  run -> AddTask(praTask);
  //run -> AddTask(AnalysisTask);
  //run -> AddTask(RansacTask);
  run -> Init();

  run->Run(0,20);


  std::cout << std::endl << std::endl;
  std::cout << "Macro finished succesfully."  << std::endl << std::endl;
  std::cout << "- Output file : " << outputFile << std::endl << std::endl;
  // -----   Finish   -------------------------------------------------------
  timer.Stop();
  Double_t rtime = timer.RealTime();
  Double_t ctime = timer.CpuTime();
  cout << endl << endl;
  cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
  cout << endl;
  // ------------------------------------------------------------------------

  gApplication->Terminate();

}

