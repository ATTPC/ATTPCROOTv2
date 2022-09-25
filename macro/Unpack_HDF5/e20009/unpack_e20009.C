#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

bool reduceFunc(AtRawEvent *evt)
{
   return (evt->GetNumPads() > 0) && evt->IsGood();
}


void unpack_e20009(TString fileName = "run_0260")
{

   // Load the library for unpacking and reconstruction
   gSystem->Load("libAtReconstruction.so");

   TStopwatch timer;
   timer.Start();
  
  
   TString parameterFile = "ATTPC.e20009.par";
   TString mappath = "";
   TString filepath = "/mnt/daqtesting/e20009_attpc_transfer/h5/";
   TString fileExt = ".h5";
   TString inputFile = filepath + fileName + fileExt;
   TString scriptfile = "e12014_pad_mapping.xml";
   TString dir = getenv("VMCWORKDIR");
   TString mapDir = dir + "/scripts/" + scriptfile;
   TString scriptdir = dir + "/scripts/" + scriptfile;
   TString dataDir = dir + "/macro/data/";
   TString geomDir = dir + "/geometry/";
   gSystem->Setenv("GEOMPATH", geomDir.Data());
   TString outputFile =  fileName + ".root"; 
   TString loggerFile = dataDir + "ATTPCLog.log";
   TString digiParFile = dir + "/parameters/" + parameterFile;
   TString geoManFile = dir + "/geometry/ATTPC_He1bar_v2.root";
   
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
   fAtMapPtr->ParseXMLMap(mapDir.Data());
   fAtMapPtr->GeneratePadPlane();

   fAtMapPtr->AddAuxPad({10,0,0,34},"trigger_live");
   fAtMapPtr->AddAuxPad({10,0,0,0},"mesh");
   fAtMapPtr->AddAuxPad({10,0,1,34},"mesh_MCA");
   fAtMapPtr->AddAuxPad({10,0,1,0},"IC");
   fAtMapPtr->AddAuxPad({10,0,2,34},"IC_sca");
   fAtMapPtr->AddAuxPad({10,0,2,0},"trigger_free");
   fAtMapPtr->AddAuxPad({10,0,3,34},"DB_beam");
   fAtMapPtr->AddAuxPad({10,0,3,0},"unassigned");
   
   auto unpacker = std::make_unique<AtHDFUnpacker>(fAtMapPtr);
   unpacker->SetInputFileName(inputFile.Data());
   unpacker->SetNumberTimestamps(2);
   unpacker->SetBaseLineSubtraction(true);

   auto unpackTask = new AtUnpackTask(std::move(unpacker));
   unpackTask->SetPersistence(false);
   
   AtFilterSubtraction *filter = new AtFilterSubtraction(fAtMapPtr);
   filter->SetThreshold(50);
   filter->SetIsGood(false);

   AtFilterTask *filterTask = new AtFilterTask(filter);
   filterTask->SetPersistence(false);
   filterTask->SetFilterAux(false);

   auto threshold = 20;

   // auto psa = new AtPSASimple2();
   auto psa = new AtPSAMax();
   psa->SetThreshold(threshold);
   // psa->SetMaxFinder();

   // Create PSA task
   AtPSAtask *psaTask = new AtPSAtask(psa);
   psaTask->SetPersistence(kTRUE);
   psaTask->SetInputBranch("AtRawEventFiltered");

   AtPRAtask *praTask = new AtPRAtask();
   praTask->SetPersistence(kTRUE);
   praTask->SetMaxNumHits(3000);
   praTask->SetMinNumHits(100);

   run->AddTask(unpackTask);
   run->AddTask(filterTask);
   run->AddTask(psaTask);
   run->AddTask(praTask);
   
   std::cout << "***** Starting Init ******" << std::endl;
   run->Init();
   std::cout << "***** Ending Init ******" << std::endl;

   // Get the number of events and unpack the whole run
   auto numEvents = unpackTask->GetNumEvents();
   std::cout << "Unpacking " << numEvents << " events. " << std::endl;

   run->Run(0,numEvents);

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
}
