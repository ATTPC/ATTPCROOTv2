#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"
void run_unpack_HC(std::string dataFile = "/mnt/daqtesting/e20009_attpc_transfer/x17/h5/run_0169.h5",
                   TString parameterFile = "pATTPC.X17.par", TString mappath = "")
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
   TString scriptdir = dir + "/scripts/" + scriptfile;
   TString dataDir = dir + "/macro/data/";
   TString geomDir = dir + "/geometry/";
   TString protomapdir = dir + "/scripts/" + protomapfile;
   TString geo = "proto20181201_geo_hires.root";
   gSystem->Setenv("GEOMPATH", geomDir.Data());

   // TString inputFile   = dataDir + name + ".digi.root";
   // TString outputFile  = dataDir + "output.root";
   TString outputFile = "output_proto.root";
   // TString mcParFile   = dataDir + name + ".params.root";
   TString loggerFile = dataDir + "ATTPCLog.log";
   TString digiParFile = dir + "/parameters/" + parameterFile;
   TString geoManFile = dir + "/geometry/ATTPC_Proto_v1.0.root";

   // -----------------------------------------------------------------
   // Logger
   FairLogger *fLogger = FairLogger::GetLogger();
   /*fLogger -> SetLogFileName(loggerFile);
   fLogger -> SetLogToScreen(kTRUE);
   fLogger -> SetLogToFile(kTRUE);
   fLogger -> SetLogVerbosityLevel("LOW");*/

   FairRunAna *run = new FairRunAna();
   run->SetSink(new FairRootFileSink(outputFile));
   run->SetGeomFile(geoManFile);

   FairRuntimeDb *rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();

   std::cout << "Setting par file: " << digiParFile << std::endl;
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);
   rtdb->getContainer("AtDigiPar");
   // FairParRootFileIo* parIo2 = new FairParRootFileIo();
   // parIo2 -> open("param.dummy_proto.root");
   // rtdb -> setFirstInput(parIo2);

   auto fMapPtr = std::make_shared<AtTpcProtoMap>();
   fMapPtr->ParseXMLMap(scriptdir.Data());
   fMapPtr->SetGeoFile(geo.Data());
   fMapPtr->SetProtoMap(protomapdir.Data());
   fMapPtr->GeneratePadPlane();

   auto unpacker = std::make_unique<AtHDFUnpacker>(fMapPtr);
   unpacker->SetInputFileName(dataFile);
   unpacker->SetBaseLineSubtraction(true);
   auto unpackTask = new AtUnpackTask(std::move(unpacker));
   unpackTask->SetPersistence(true);
   run->AddTask(unpackTask);

   auto filter = new AtFilterFFT();
   filter->SetSaveTransform(true);
   filter->AddFreqRange({0, 1, 512 / 2 + 1, 1});
   filter->DumpFactors();

   AtFilterTask *filterTask = new AtFilterTask(filter);
   filterTask->SetPersistence(true);
   filterTask->SetFilterAux(true);
   run->AddTask(filterTask);

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(20);
   auto psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetInputBranch("AtRawEventFiltered");
   psaTask->SetPersistence(true);
   run->AddTask(psaTask);

   auto praTask = new AtPRAtask();
   praTask->SetPersistence(true);
   // praTask->SetTcluster(5.0);
   // praTask->SetMaxNumHits(3000);
   // praTask->SetMinNumHits(300);
   run->AddTask(praTask);

   run->Init();
   run->Run(0, 10);

   std::cout << std::endl << std::endl;
   std::cout << "Macro finished succesfully." << std::endl << std::endl;
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
