#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

#include <sys/stat.h>
#include "FairLogger.h"

bool check_file(const std::string &name);

void run_unpack_SpecMAT(TString dataFile = "./data/TTreesGETrun_9993.root", TString mappath = "../../")
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
   TString scriptfile = "LookupSpecMATnoScint.xml";
   TString dir = getenv("VMCWORKDIR");
   TString scriptdir = dir + "/scripts/" + scriptfile;
   TString dataDir = dir + "/macro/data/";
   TString geomDir = dir + "/geometry/";
   gSystem->Setenv("GEOMPATH", geomDir.Data());

   // Parameter file name
   TString parFile = "./data/attpcpar.root";

   TString outputFile = "./data/run_9993.root";
   // TString mcParFile   = dataDir + name + ".params.root";
   TString loggerFile = dataDir + "SpecMATLog.log";

   TString parameterFile = "SpecMAT.run_9993.par";
   TString triggerFile = "SpecMAT.trigger.par";
   TString trigParFile = dir + "/parameters/" + triggerFile;
   TString digiParFile = dir + "/parameters/" + parameterFile;

   TString geoManFile = geomDir + "SpecMAT_He1Bar.root";

   TString inimap = mappath + "inhib.txt";
   TString lowgmap = mappath + "lowgain.txt";
   TString xtalkmap = mappath + "beampads_e15503b.txt";

   // -----------------------------------------------------------------
   // Logger
   //FairLogger::GetLogger()->SetLogSeverity("debug");
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

   // ------------------------------------------------------------------------

   // Settings
   Bool_t fUseDecoder = kTRUE;
   if (dataFile.IsNull() == kTRUE)
      fUseDecoder = kFALSE;

   Bool_t fUseSeparatedData = kFALSE;
   if (dataFile.EndsWith(".txt"))
      fUseSeparatedData = kTRUE;

   /*
    *     Unpacking options:
    *         - SetUseSeparatedData:      To be used with 10 CoBo files without
    * merging. Mainly for the ATTPC. Enabled if the input file is a txt.
    *         - SetPseudoTopologyFrame:   Used to force the graw file to have a
    * Topology frame.
    *         - SetPersistance:           Save the unpacked data into the root
    * file.
    *         - SetMap:                   Chose the lookup table.
    *         - SetMapOpt                 Chose the pad plane geometry. In
    * addition forces the unpacker to use Basic Frames for 1 single file (p-ATTPC
    * case) of Layered Frames for Merged Data (10 Cobos merged data).
    */

   // Create the map that will be pased to tasks that require it
   auto fMapPtr = std::make_shared<AtSpecMATMap>(3174);
   fMapPtr->ParseXMLMap(scriptdir.Data());
   fMapPtr->GeneratePadPlane();
   // mapPtr->SetInhibitMaps(inimap,lowgmap,xtalkmap); // TODO: Only
   // implemented for fUseSeparatedData!!!!!!!!!!!!!!!!!!!1

   AtDecoderSpecMATTask *fDecoderTask = new AtDecoderSpecMATTask();
   fDecoderTask->SetUseSeparatedData(fUseSeparatedData);
   if (fUseSeparatedData)
      fDecoderTask->SetPseudoTopologyFrame(kTRUE); //! This calls the method 10 times so for less than 10 CoBos
                                                   //! ATCore2 must be modified
   Bool_t IsCoboPositivePolarity[4] = {true, false, true,
                                       true}; // Positive polarity for padplane, negative for scintillators
   fDecoderTask->SetPositivePolarity(IsCoboPositivePolarity);
   fDecoderTask->SetPersistence(kTRUE);
   // fDecoderTask->SetMap(scriptdir.Data());
   // fDecoderTask->SetMapOpt(2); // Does not do anything for SpecMAT for the moment
   fDecoderTask->SetMap(fMapPtr);
   fDecoderTask->SetNumCobo(4);
   Bool_t IsCoboPadPlane[4] = {true, false, true, true};
   fDecoderTask->SetIsCoboPadPlane(IsCoboPadPlane);
   fDecoderTask->AddData(dataFile);
   run->AddTask(fDecoderTask);

   AtPulseTask *pulse = new AtPulseTask();
   pulse->SetPersistence(kTRUE);
   pulse->SetSaveMCInfo();
   // pulse->SelectDetectorId(kSpecMAT);
   pulse->SetMap(fMapPtr);
   //run->AddTask(pulse); Commented out because it does nothing, we have waveforms already

   AtPSASimple2 *psa = new AtPSASimple2();
   AtPSAtask *psaTask = new AtPSAtask(psa);
   psaTask->SetPersistence(kTRUE);
   psa->SetThreshold(50);
   // psaTask -> SetPeakFinder(); //NB: Use either peak finder of maximum finder
   // but not both at the same time
   psa->SetMaxFinder();
   // psa-> SetBaseCorrection(kTRUE); //Directly apply the base line correction
   // to the pulse amplitude to correct for the mesh induction. If false the
   // correction is just saved psa -> SetTimeCorrection(kTRUE); //Interpolation
   // around the maximum of the signal peak. Only affect Z calibration at PSA
   // stage
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

    AtPRAtask *praTask = new AtPRAtask();
     praTask->SetPersistence(kTRUE);
     run->AddTask(praTask);

     AtClusterizeTask *clusterizer = new AtClusterizeTask();
     clusterizer->SetPersistence(kFALSE);
     run->AddTask(clusterizer);
     */

   std::cout << std::endl << "**** Begining Init ****" << std::endl;
   run->Init();
   std::cout << "**** Ending Init ****" << std::endl << std::endl;

   // run -> RunOnTBData();
   run->Run(0, 30);

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
