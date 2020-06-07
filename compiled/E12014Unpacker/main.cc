#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

#include <iostream>
using namespace std;

void run_unpack_adam(std::string dataFile = "/mnt/user/e12014/attpc/run_0260.h5",
		     TString parameterFile = "ATTPC.e15250.par", TString mappath="")
{

  //dataFile="/mnt/analysis/e18505_attpc/ND2019/run_0171.h5";
  //dataFile = "/mnt/user/e12014/attpc/run_0157.h5";
  
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
  TString dataDir   = dir + "/macro/data/";
  TString geomDir   = dir + "/geometry/";
  gSystem -> Setenv("GEOMPATH", geomDir.Data());

  TString outputFile  = "output.root";

  //TString mcParFile   = dataDir + name + ".params.root";

  TString loggerFile  = dataDir + "ATTPCLog.log";
  TString digiParFile = dir + "/parameters/" + parameterFile;
  TString geoManFile  = dir + "/geometry/ATTPC_v1.1.root";

  // -----------------------------------------------------------------
  // Logger
  FairLogger *fLogger = FairLogger::GetLogger();
  fLogger -> SetLogFileName(TString(dir + "/macro/Unpack_HDF5/ATTPCLog.log").Data());
  fLogger -> SetLogToScreen(kTRUE);
  fLogger -> SetLogToFile(kTRUE);
  fLogger -> SetLogVerbosityLevel("LOW");

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
  HDFParserTask->SetOldFormat(false);
  HDFParserTask->SetTimestampIndex(2);
  
  ATPSATask *psaTask = new ATPSATask();
  psaTask -> SetPersistence(kTRUE);
  psaTask -> SetThreshold(10);
  psaTask -> SetPSAMode(1); //NB: 1 is ATTPC - 2 is pATTPC - 3 Filter for ATTPC - 4: Full Time Buckets
    
  
  run -> AddTask(HDFParserTask);
  //run -> AddTask(psaTask);

  run -> Init();
  auto numEvents = HDFParserTask->GetNumEvents();
  
  std::cout << "There are " << numEvents << "Events." << std::endl;
  
  //Only look at first 10 events
  //run->Run(0,10);

  // Unpack all events
  run->Run(0,numEvents/2);



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

