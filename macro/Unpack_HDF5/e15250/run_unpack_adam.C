#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

struct auxchannel {
   std::string name;
   uint8_t cobo;
   uint8_t asad;
   uint8_t aget;
   uint8_t channel;
};

void run_unpack_adam(
   std::string dataFile = "/media/yassid/bdcb3c81-adb9-4a9d-9172-0bd5935c1dd5/data/e15250_attpc/h5/run_0255.h5",
   TString parameterFile = "ATTPC.e15250_sim.par", TString mappath = "")
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
   TString scriptdir = dir + "/scripts/" + scriptfile;
   TString dataDir = dir + "/macro/data/";
   TString geomDir = dir + "/geometry/";
   gSystem->Setenv("GEOMPATH", geomDir.Data());

   // TString inputFile   = dataDir + name + ".digi.root";
   // TString outputFile  = dataDir + "output.root";
   TString outputFile = "/media/yassid/bdcb3c81-adb9-4a9d-9172-0bd5935c1dd5/data/e15250_attpc/root/output.root";
   // TString mcParFile   = dataDir + name + ".params.root";
   TString loggerFile = dataDir + "ATTPCLog.log";
   TString digiParFile = dir + "/parameters/" + parameterFile;
   TString geoManFile = dir + "/geometry/ATTPC_v1.1.root";

   TString inimap = mappath + "inhib.txt";
   TString lowgmap = mappath + "lowgain.txt";
   TString xtalkmap = mappath + "beampads_e15503b.txt";

   // -----------------------------------------------------------------
   // Logger
   /*FairLogger *fLogger = FairLogger::GetLogger();
   fLogger -> SetLogFileName(loggerFile);
   fLogger -> SetLogToScreen(kTRUE);
   fLogger -> SetLogToFile(kTRUE);
   fLogger -> SetLogVerbosityLevel("LOW");*/

   FairRunAna *run = new FairRunAna();
   run->SetSink(new FairRootFileSink(outputFile));
   run->SetGeomFile(geoManFile);

   FairRuntimeDb *rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);
   rtdb->getContainer("AtDigiPar");

   // Auxiliary channels
   // Hash table: cobo, asad, aget, channel
   std::vector<auxchannel> aux_channels;

   auxchannel ch_1{"ion_chamber", 4, 1, 0, 65};
   aux_channels.push_back(ch_1);
   auxchannel ch_2{"ion_chamber_downscale", 4, 1, 0, 66};
   aux_channels.push_back(ch_2);
   auxchannel ch_3{"unknown_1", 4, 1, 0, 67};
   aux_channels.push_back(ch_3);
   auxchannel ch_4{"unknown_2", 4, 1, 0, 61};
   aux_channels.push_back(ch_4);
   auxchannel ch_5{"unknown_3", 4, 1, 0, 64};
   aux_channels.push_back(ch_5);
   auxchannel ch_6{"unknown_4", 4, 1, 0, 59};
   aux_channels.push_back(ch_6);

   auto fAtMapPtr = std::make_shared<AtTpcMap>();
   fAtMapPtr->ParseXMLMap(scriptdir.Data());
   fAtMapPtr->GeneratePadPlane();

   for (auto iaux : aux_channels) {
      fAtMapPtr->AddAuxPad({iaux.cobo, iaux.asad, iaux.aget, iaux.channel}, iaux.name);
   }

   auto unpacker = std::make_unique<AtHDFUnpacker>(fAtMapPtr);
   unpacker->SetInputFileName(dataFile);
   unpacker->SetNumberTimestamps(1);
   unpacker->SetBaseLineSubtraction(true);

   auto unpackTask = new AtUnpackTask(std::move(unpacker));
   unpackTask->SetPersistence(false);

   auto threshold = 45;

   auto *filter = new AtFilterFFT();
   filter->SetLowPass(6, 100);
   filter->DumpFactors();
   filter->SetSaveTransform(false);

   AtFilterTask *filterTask = new AtFilterTask(filter);
   filterTask->SetPersistence(kTRUE);
   filterTask->SetFilterAux(false);

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(threshold);

   AtPSAtask *psaTask = new AtPSAtask(psa->Clone());
   // psaTask->SetInputBranch("AtRawEventFiltered");
   psaTask->SetPersistence(false);

   AtPSAtask *psaTaskFilter = new AtPSAtask(psa->Clone());
   psaTaskFilter->SetInputBranch("AtRawEventFiltered");
   psaTaskFilter->SetOutputBranch("AtEventFiltered");
   psaTaskFilter->SetPersistence(true);

   auto *wHDF = new AtHDF5WriteTask(
      "/media/yassid/bdcb3c81-adb9-4a9d-9172-0bd5935c1dd5/data/e15250_attpc/root/outputOther.h5", "AtEventFiltered");
   wHDF->SetUseEventNum(true);

   run->AddTask(unpackTask);
   run->AddTask(filterTask);
   run->AddTask(psaTask);
   run->AddTask(psaTaskFilter);
   run->AddTask(wHDF);

   run->Init();

   auto numEvents = unpackTask->GetNumEvents();
   // numEvents = 100;

   run->Run(0, numEvents);
   // run->Run(0, 1000);
   // run -> RunOnTBData();

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
