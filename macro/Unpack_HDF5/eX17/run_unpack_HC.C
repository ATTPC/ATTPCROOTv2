#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

struct auxchannel
{
  std::string name;
  uint8_t cobo;
  uint8_t asad;
  uint8_t aget;
  uint8_t channel;
};


void run_unpack_HC(std::string dataFile = "/mnt/daqtesting/e20009_attpc_transfer/x17/h5/run_0169.h5",TString parameterFile = "pATTPC.X17.par",TString mappath="")
{

  // -----   Timer   --------------------------------------------------------
 TStopwatch timer;
 timer.Start();
 // ------------------------------------------------------------------------

  gSystem->Load("libXMLParser.so");
  // -----------------------------------------------------------------
  // Set file names
  TString scriptfile = "LookupProtoX17.xml";
  TString protomapfile = "proto20181201.map";
  TString dir = getenv("VMCWORKDIR");
  TString scriptdir = dir + "/scripts/"+ scriptfile;
  TString dataDir = dir + "/macro/data/";
  TString geomDir = dir + "/geometry/";
  TString protomapdir = dir + "/scripts/"+ protomapfile;
  TString geo = "proto20181201_geo_hires.root";
  gSystem -> Setenv("GEOMPATH", geomDir.Data());

  //TString inputFile   = dataDir + name + ".digi.root";
  //TString outputFile  = dataDir + "output.root";
  TString outputFile  = "output_proto.root";
  //TString mcParFile   = dataDir + name + ".params.root";
  TString loggerFile  = dataDir + "ATTPCLog.log";
  TString digiParFile = dir + "/parameters/" + parameterFile;
  TString geoManFile  = dir + "/geometry/ATTPC_Proto_v1.0.root";


  // -----------------------------------------------------------------
  // Logger
  FairLogger *fLogger = FairLogger::GetLogger();
  /*fLogger -> SetLogFileName(loggerFile);
  fLogger -> SetLogToScreen(kTRUE);
  fLogger -> SetLogToFile(kTRUE);
  fLogger -> SetLogVerbosityLevel("LOW");*/

  FairRunAna* run = new FairRunAna();
  run -> SetOutputFile(outputFile);
  //run -> SetGeomFile("../geometry/ATTPC_Proto_v1.0.root");
  run -> SetGeomFile(geoManFile);

  FairRuntimeDb* rtdb = run->GetRuntimeDb();
  FairParAsciiFileIo* parIo1 = new FairParAsciiFileIo();
  parIo1 -> open(digiParFile.Data(), "in");
  //FairParRootFileIo* parIo2 = new FairParRootFileIo();
  //parIo2 -> open("param.dummy_proto.root");
 // rtdb -> setFirstInput(parIo2);
  rtdb -> setSecondInput(parIo1);


  AtHDFParserTask* HDFParserTask = new AtHDFParserTask(1);
  HDFParserTask->SetPersistence(kTRUE);
  HDFParserTask->SetAtTPCMap(scriptdir.Data());
  HDFParserTask->SetProtoGeoFile(geo.Data());
  HDFParserTask->SetProtoMapFile(protomapdir.Data());
  HDFParserTask->SetBaseLineSubtraction(kTRUE);
  HDFParserTask->SetFileName(dataFile);

 
  AtPSASimple2 *psa = new AtPSASimple2();
   // psa -> SetPeakFinder(); //NB: Use either peak finder of maximum finder but not both at the same time
   // psa -> SetBaseCorrection(kFALSE);
   // psa -> SetTimeCorrection(kFALSE);

    

   AtPSAtask *psaTask = new AtPSAtask(psa);
   psaTask->SetPersistence(kTRUE);
   psa->SetThreshold(30);
   psa->SetMaxFinder();
   // psa->SetMeanK(4);
   // psa->SetStddevMulThresh(0.1);

   AtPRAtask *praTask = new AtPRAtask();
   praTask->SetPersistence(kTRUE);
   praTask->SetTcluster(5.0);
   //praTask->SetMaxNumHits(3000);
   //praTask->SetMinNumHits(300);

   run->AddTask(HDFParserTask);
   run->AddTask(psaTask);
   run->AddTask(praTask);  
  
  
  run -> Init();

  run->Run(0,300);
  

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

