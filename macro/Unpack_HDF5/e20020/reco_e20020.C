#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

void reco_e20020()
{

  // -----   Timer   --------------------------------------------------------
 TStopwatch timer;
 timer.Start();
 // ------------------------------------------------------------------------

  gSystem->Load("libXMLParser.so");
  // -----------------------------------------------------------------
  // Set file names
  TString fileName="run_0094";
  TString parameterFile = "ATTPC.e20020.par";
  TString mappath = "";
  TString filepath = "";
  TString fileExt = ".root";
  TString dataFile = fileName + fileExt;
  TString scriptfile = "e12014_pad_mapping.xml";
  TString dir = getenv("VMCWORKDIR");
  TString scriptdir = dir + "/scripts/"+ scriptfile;
  TString dataDir = dir + "/macro/data/";
  TString geomDir = dir + "/geometry/";
  gSystem -> Setenv("GEOMPATH", geomDir.Data());

  //TString inputFile   = dataDir + name + ".digi.root";
  //TString outputFile  = dataDir + "output.root";
  TString outputFile  = fileName+"_reco.root";
  //TString mcParFile   = dataDir + name + ".params.root";
  TString loggerFile  = dataDir + "ATTPCLog.log";
  TString digiParFile = dir + "/parameters/" + parameterFile;
  TString geoManFile  = dir + "/geometry/ATTPC_He1bar_v2_geomanager.root";

  //TString inimap   = mappath + "inhib.txt";
  //TString lowgmap  = mappath + "lowgain.txt";
  //TString xtalkmap = mappath + "beampads_e15503b.txt";

  // -----------------------------------------------------------------
  // Logger
  FairLogger *fLogger = FairLogger::GetLogger();
  /*fLogger -> SetLogFileName(loggerFile);
  fLogger -> SetLogToScreen(kTRUE);
  fLogger -> SetLogToFile(kTRUE);
  fLogger -> SetLogVerbosityLevel("LOW");*/

  FairRunAna* run = new FairRunAna();
  run -> SetInputFile(dataFile);
  run -> SetOutputFile(outputFile);
  run -> SetGeomFile(geoManFile);

  FairRuntimeDb* rtdb = run->GetRuntimeDb();
  FairParAsciiFileIo* parIo1 = new FairParAsciiFileIo();
  parIo1 -> open(digiParFile.Data(), "in");
  //FairParRootFileIo* parIo2 = new FairParRootFileIo();
  //parIo2 -> open("param.dummy_proto.root");
 // rtdb -> setFirstInput(parIo2);
  rtdb -> setSecondInput(parIo1);

  //AtHDFParserTask* HDFParserTask = new AtHDFParserTask();
  //HDFParserTask->SetPersistence(kFALSE);
  //HDFParserTask->SetAtTPCMap(scriptdir.Data());
  //HDFParserTask->SetFileName(dataFile.Data());
  //HDFParserTask->SetBaseLineSubtraction(kTRUE);
  
  //AtPSASimple2 *psa = new AtPSASimple2();
   // psa -> SetPeakFinder(); //NB: Use either peak finder of maximum finder but not both at the same time
   // psa -> SetBaseCorrection(kFALSE);
   // psa -> SetTimeCorrection(kFALSE);

   //AtPSAtask *psaTask = new AtPSAtask(psa);
   //psaTask->SetPersistence(kTRUE);
   //psa->SetThreshold(250);
   //psa->SetMaxFinder();
  
   //AtPRAtask *praTask = new AtPRAtask();
  //praTask->SetPersistence(kTRUE);
   
   //run -> AddTask(HDFParserTask);
  //run -> AddTask(psaTask);
  //run -> AddTask(praTask);

  AtFitterTask *fitterTask = new AtFitterTask();
  fitterTask->SetPersistence(kTRUE);

  run->AddTask(fitterTask);
  
  run -> Init();

  run->Run(61,62);
   //run->RunOnTBData();

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

