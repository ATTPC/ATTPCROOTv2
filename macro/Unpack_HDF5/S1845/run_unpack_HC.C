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


void run_unpack_HC(TString inputFile = "/home/yassid/fair_install/data/S1845/run_0103.h5",TString parameterFile = "ATTPC.e20009.par",TString mappath="")
{

  // -----   Timer   --------------------------------------------------------
 TStopwatch timer;
 timer.Start();
 // ------------------------------------------------------------------------

  gSystem->Load("libXMLParser.so");
  // -----------------------------------------------------------------
  // Set file names
  TString mapfile = "LookupProto20181201v2.xml";
  TString protomapfile = "proto20181201.map";
  TString dir = getenv("VMCWORKDIR");
  TString mapdir = dir + "/scripts/"+ mapfile;
  TString dataDir = dir + "/macro/data/";
  TString geomDir = dir + "/geometry/";
  TString protomapdir = dir + "/scripts/"+ protomapfile;
  TString geo = "proto20181201_geo_hires.root";
  gSystem -> Setenv("GEOMPATH", geomDir.Data());

   TString outputFile  = "output_proto.root";
  TString loggerFile  = dataDir + "ATTPCLog.log";
  TString digiParFile = dir + "/parameters/" + parameterFile;
  TString geoManFile  = dir + "/geometry/ATTPC_Proto_v1.0.root";


   FairRunAna *run = new FairRunAna();
   run->SetOutputFile(outputFile);
   run->SetGeomFile(geoManFile);

   // Set the parameter file
   FairRuntimeDb *rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();

   std::cout << "Setting par file: " << digiParFile << std::endl;
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);
   std::cout << "Getting containers..." << std::endl;
   // We must get the container before initializing a run
   rtdb->getContainer("AtDigiPar");

   auto fAtMapPtr = std::make_shared<AtTpcMap>();
   fAtMapPtr->ParseXMLMap(mapdir.Data());
   
  // -----------------------------------------------------------------
  
  fAtMapPtr->AddAuxPad({5,0,0,65},"mutant");
   fAtMapPtr->AddAuxPad({5,0,0,66},"mesh");
   fAtMapPtr->AddAuxPad({5,0,0,67},"protons");
   fAtMapPtr->AddAuxPad({5,0,0,61},"begin_DAQ");
   fAtMapPtr->AddAuxPad({5,0,0,64},"unknown");
   fAtMapPtr->AddAuxPad({5,0,0,59},"downscaled_alpha");
    

   auto unpacker = std::make_unique<AtHDFUnpacker>(fAtMapPtr);
   unpacker->SetInputFileName(inputFile.Data());
   //unpacker->SetNumberTimestamps(2);
   unpacker->SetBaseLineSubtraction(true);

   auto unpackTask = new AtUnpackTask(std::move(unpacker));
   unpackTask->SetPersistence(false);

   auto threshold = 20;

   
   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(threshold);
   
   // Create PSA task
   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetPersistence(kTRUE);
  
   run->AddTask(unpackTask);
   run->AddTask(psaTask);

   std::cout << "***** Starting Init ******" << std::endl;
   run->Init();
   std::cout << "***** Ending Init ******" << std::endl;

   // Get the number of events and unpack the whole run
   auto numEvents = unpackTask->GetNumEvents();
   std::cout << "Unpacking " << numEvents << " events. " << std::endl;

   run->Run(0,numEvents);
   
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

