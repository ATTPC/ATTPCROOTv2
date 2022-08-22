#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

#include <sys/stat.h>
//#include "FairLogger.h"

bool check_file(const std::string &name);

void run_unpack_SpecMAT(TString dataFile = "./data/TTreesGETrun_9901.root")
{

   if (!check_file(dataFile.Data())) {
      std::cout << cRED << " Run file " << dataFile.Data() << " not found! Terminating..." << cNORMAL << std::endl;
      exit(0);
   }

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();
   // ------------------------------------------------------------------------

   gSystem->Load("libXMLParser.so");
   // -----------------------------------------------------------------
   // Set file names
   TString scriptfile = "LookupSpecMATnoScint3seg.xml";
   TString dir = getenv("VMCWORKDIR");
   TString scriptdir = dir + "/scripts/" + scriptfile;
   TString geomDir = dir + "/geometry/";
   gSystem->Setenv("GEOMPATH", geomDir.Data());

   // Parameter file name

   TString outputFile = "./data/run_9901.root";
   // TString mcParFile   = dataDir + name + ".params.root";
   TString loggerFile = "./data/SpecMATLog.log";

   TString parameterFile = "SpecMAT.run_9901.par";
   TString triggerFile = "SpecMAT.trigger.par";
   TString trigParFile = "./data/" + triggerFile;
   TString digiParFile = "./data/" + parameterFile;

   TString geoManFile = geomDir + "SpecMAT_He1Bar.root";

   /*TString inimap = mappath + "inhib.txt";
     TString lowgmap = mappath + "lowgain.txt";
     TString xtalkmap = mappath + "beampads_e15503b.txt";
   */

   // -----------------------------------------------------------------
   // Logger
   /*auto verbSpec =
      fair::VerbositySpec::Make(fair::VerbositySpec::Info::severity, fair::VerbositySpec::Info::file_line_function);
   fair::Logger::DefineVerbosity("user1", verbSpec);
   fair::Logger::SetVerbosity("user1");
   fair::Logger::SetConsoleSeverity("debug");
   */

   FairRunAna *run = new FairRunAna();
   run->SetOutputFile(outputFile);
   run->SetGeomFile(geoManFile);

   FairRuntimeDb *rtdb = run->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);
   FairParAsciiFileIo *parIo2 = new FairParAsciiFileIo();
   parIo2->open(trigParFile.Data(), "in");
   rtdb->setSecondInput(parIo2);
   rtdb->getContainer("AtDigiPar");
   // ------------------------------------------------------------------------

   // Create the map that will be pased to tasks that require it
   auto fMapPtr = std::make_shared<AtSpecMATMap>(3174);
   fMapPtr->ParseXMLMap(scriptdir.Data());
   fMapPtr->GeneratePadPlane();

   // Set arrays required of unpacker
   std::vector<bool> isCoboPadPlane = {true, true, true, true};
   std::vector<bool> isCoboNegativePolarity;
   // Positive polarity for padplane, negative for scintillators
   for (auto elem : isCoboPadPlane)
      isCoboNegativePolarity.push_back(!elem);

   // Create unpacker and fill set required info
   auto unpacker = std::make_unique<AtROOTUnpacker>(fMapPtr);
   unpacker->SetIsPadPlaneCobo(isCoboPadPlane);
   unpacker->SetIsNegativePolarity(isCoboNegativePolarity);
   unpacker->SetInputFileName(dataFile.Data());
   // Create task and add to run
   auto unpackTask = new AtUnpackTask(std::move(unpacker));
   unpackTask->SetPersistence(true);
   run->AddTask(unpackTask);

   AtPSASimple2 *psa = new AtPSASimple2();
   psa->SetThreshold(50);
   psa->SetMaxFinder();

   AtPSAtask *psaTask = new AtPSAtask(psa);
   psaTask->SetPersistence(kTRUE);
   run->AddTask(psaTask);

   /*
    AtHoughTask *HoughTask = new AtHoughTask();
    HoughTask ->SetPersistence(kTRUE);
    HoughTask ->SetLinearHough();
    //HoughTask ->SetCircularHough();
    HoughTask ->SetHoughThreshold(50.0); // Charge threshold for Hough
    HoughTask ->SetEnableMap(); //Enables an instance of the ATTPC map:  This
    enables the MC with Q instead of position HoughTask
    ->SetMap(scriptdir.Data()); run -> AddTask(HoughTask);
    */

    AtPRAtask *praTask = new AtPRAtask();
    praTask->SetPersistence(kTRUE);
    run->AddTask(praTask);


      /*
     AtClusterizeTask *clusterizer = new AtClusterizeTask();
     clusterizer->SetPersistence(kFALSE);
     run->AddTask(clusterizer);
     */
     

   std::cout << std::endl << "**** Begining Init ****" << std::endl;
   run->Init();
   std::cout << "**** Ending Init ****" << std::endl << std::endl;

   // run -> RunOnTBData();
   run->Run(0, 30);

   auto numEvents = unpackTask->GetNumEvents();

   // numEvents = 1700;//217;
   // numEvents = 5;

   std::cout << "Unpacking 30 out of " << numEvents << " events. " << std::endl;


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

bool check_file(const std::string &name)
{
   struct stat buffer;
   return (stat(name.c_str(), &buffer) == 0);
}
