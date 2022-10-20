
void run_writeHDF(TString dataFile = "./data/GADGET-multiple-files.txt", TString parameterFile = "GADGET.sim.par",
                  TString mappath = "")

{

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();
   // ------------------------------------------------------------------------

   gSystem->Load("libXMLParser.so");
   // -----------------------------------------------------------------
   // Set file names
   TString scriptfile = "LookupGADGET08232021.xml";
   TString dir = getenv("VMCWORKDIR");
   TString scriptdir = dir + "/scripts/" + scriptfile;
   TString dataDir = dir + "/macro/Unpack_GETDecoder2/";
   TString geomDir = dir + "/geometry/";
   gSystem->Setenv("GEOMPATH", geomDir.Data());

   TString outputFile = "./data/output.root";
   TString digiParFile = dir + "/parameters/" + parameterFile;
   TString geoManFile = dir + "/geometry/GADGET_II.root";

   TString inimap = mappath + "inhib.txt";
   TString lowgmap = mappath + "lowgain.txt";
   TString xtalkmap = mappath + "beampads_e15503b.txt";

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
   rtdb->getContainer("AtDigiPar");

   auto fMapPtr = std::make_shared<AtGadgetIIMap>();
   fMapPtr->ParseXMLMap(scriptdir.Data());
   fMapPtr->GeneratePadPlane();

   auto unpacker = std::make_unique<AtGRAWUnpacker>(fMapPtr, 4);
   unpacker->SetInputFileName(dataFile.Data(), "AsAd%i");
   unpacker->SetInitialEventID(0);
   auto unpackTask = new AtUnpackTask(std::move(unpacker));
   unpackTask->SetPersistence(true);

   auto psa = std::make_unique<AtPSAMax>();
   psa->SetThreshold(10);
   AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
   psaTask->SetPersistence(kTRUE);

   auto *wHDF = new AtHDF5WriteTask("data/output.h5", "AtEventH");

   run->AddTask(unpackTask);
   run->AddTask(psaTask);
   run->AddTask(wHDF);

   run->Init();

   // run -> RunOnTBData();
   run->Run(0, 100);

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

   // gApplication->Terminate();
}
