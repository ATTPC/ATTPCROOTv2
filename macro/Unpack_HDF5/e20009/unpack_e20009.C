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

void unpack_e20009(TString fileName)
{

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();
   // ------------------------------------------------------------------------

   gSystem->Load("libXMLParser.so");
   // -----------------------------------------------------------------
   // Set file names

   TString parameterFile = "ATTPC.e20009.par";
   TString mappath = "";
   TString filepath = "/mnt/daqtesting/e20009_attpc_transfer/h5/";
   TString fileExt = ".h5";
   TString dataFile = filepath + fileName + fileExt;
   TString scriptfile = "e12014_pad_mapping.xml";
   TString dir = getenv("VMCWORKDIR");
   TString scriptdir = dir + "/scripts/" + scriptfile;
   TString dataDir = dir + "/macro/data/";
   TString geomDir = dir + "/geometry/";
   gSystem->Setenv("GEOMPATH", geomDir.Data());

   // TString inputFile   = dataDir + name + ".digi.root";
   // TString outputFile  = dataDir + "output.root";
   TString outputFile = "/mnt/analysis/e20009/" + fileName + ".root";
   // TString mcParFile   = dataDir + name + ".params.root";
   TString loggerFile = dataDir + "ATTPCLog.log";
   TString digiParFile = dir + "/parameters/" + parameterFile;
   TString geoManFile = dir + "/geometry/ATTPC_He1bar_v2.root";

   TString inimap = mappath + "inhib.txt";
   TString lowgmap = mappath + "lowgain.txt";
   TString xtalkmap = mappath + "beampads_e15503b.txt";

   // -----------------------------------------------------------------
   // Logger
   FairLogger *fLogger = FairLogger::GetLogger();
   /*fLogger -> SetLogFileName(loggerFile);
   fLogger -> SetLogToScreen(kTRUE);
   fLogger -> SetLogToFile(kTRUE);
   fLogger -> SetLogVerbosityLevel("LOW");*/

   FairRunAna *run = new FairRunAna();
   run->SetOutputFile(outputFile);
   run->SetGeomFile(geoManFile);

   FairRuntimeDb *rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(digiParFile.Data(), "in");
   // FairParRootFileIo* parIo2 = new FairParRootFileIo();
   // parIo2 -> open("param.dummy_proto.root");
   // rtdb -> setFirstInput(parIo2);
   rtdb->setSecondInput(parIo1);

   AtHDFParserTask *HDFParserTask = new AtHDFParserTask();
   HDFParserTask->SetPersistence(kFALSE);
   HDFParserTask->SetAtTPCMap(scriptdir.Data());
   HDFParserTask->SetFileName(dataFile.Data());
   HDFParserTask->SetBaseLineSubtraction(kTRUE);

   //--------Auxiliary channels

   // Hash table: cobo, asad, aget, channel
   std::vector<auxchannel> aux_channels;
   auxchannel ch_1{"trigger_live", 10, 0, 0, 34};
   aux_channels.push_back(ch_1);
   auxchannel ch_2{"mesh", 10, 0, 0, 0};
   aux_channels.push_back(ch_2);
   auxchannel ch_3{"mesh_MCA", 10, 0, 1, 34};
   aux_channels.push_back(ch_3);
   auxchannel ch_4{"IC", 10, 0, 1, 0};
   aux_channels.push_back(ch_4);
   auxchannel ch_5{"IC_sca", 10, 0, 2, 34};
   aux_channels.push_back(ch_5);
   auxchannel ch_6{"trigger_free", 10, 0, 2, 0};
   aux_channels.push_back(ch_6);
   auxchannel ch_7{"DB_beam", 10, 0, 3, 34};
   aux_channels.push_back(ch_7);
   auxchannel ch_8{"unassigned", 10, 0, 3, 0};
   aux_channels.push_back(ch_8);
   //---------End of auxiliary channel setup

   for (auto iaux : aux_channels) {
      auto hash = HDFParserTask->CalculateHash(iaux.cobo, iaux.asad, iaux.aget, iaux.channel);
      auto isaux = HDFParserTask->SetAuxChannel(hash, iaux.name);
   }

   AtPSASimple2 *psa = new AtPSASimple2();
   // psa -> SetPeakFinder(); //NB: Use either peak finder of maximum finder but not both at the same time
   // psa -> SetBaseCorrection(kFALSE);
   // psa -> SetTimeCorrection(kFALSE);

   AtPSAtask *psaTask = new AtPSAtask(psa);
   psaTask->SetPersistence(kTRUE);
   psa->SetThreshold(70);
   psa->SetMaxFinder();

   AtPRAtask *praTask = new AtPRAtask();
   praTask->SetPersistence(kTRUE);

   run->AddTask(HDFParserTask);
   run->AddTask(psaTask);
   run->AddTask(praTask);

   run->Init();

   run->Run(0, 10000);
   // run->RunOnTBData();

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
