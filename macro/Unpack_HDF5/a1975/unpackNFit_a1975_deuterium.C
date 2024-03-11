#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

bool reduceFunc(AtRawEvent *evt)
{
   return (evt->GetNumPads() > 0) && evt->IsGood();
}
void unpackNFit_a1975_deuterium(TString fileName = "run_0106")
{

   // Load the library for unpacking and reconstruction
   gSystem->Load("libAtReconstruction.so");

   TStopwatch timer;
   timer.Start();

   TString parameterFile = "ATTPC.a1975_deuterium.par";
   TString mappath = "";
   TString filepath = "/media/yassid/bdcb3c81-adb9-4a9d-9172-0bd5935c1dd5/data/a1975/";
   TString fileExt = ".h5";
   TString inputFile = filepath + fileName + fileExt;
   TString scriptfile = "ANL2023.xml";
   TString dir = getenv("VMCWORKDIR");
   TString mapDir = dir + "/scripts/" + scriptfile;
   TString scriptdir = dir + "/scripts/" + scriptfile;
   TString dataDir = dir + "/macro/data/";
   TString geomDir = dir + "/geometry/";
   gSystem->Setenv("GEOMPATH", geomDir.Data());
   TString outputFile = fileName + ".root";
   TString loggerFile = dataDir + "ATTPCLog.log";
   TString digiParFile = dir + "/parameters/" + parameterFile;
   TString geoManFile = dir + "/geometry/ATTPC_H1bar_geomanager.root";

   // Specific paths for three LUT for electric field correction
   TString zlutFile = dir + "/resources/corrections/a1954/zLUT.txt";
   TString radlutFile = dir + "/resources/corrections/a1954/radLUT.txt";
   TString tralutFile = dir + "/resources/corrections/a1954/traLUT.txt";

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

   auto threshold = 35;

   Double_t clusterRadius = 15.0;
   Double_t clusterDistance = 7.5;

   auto psa = new AtPSAMax();
   psa->SetThreshold(threshold);

   // Create PSA task
   AtPSAtask *psaTask = new AtPSAtask(psa);
   psaTask->SetPersistence(true);
   // psaTask->SetInputBranch("AtRawEventFiltered");
   psaTask->SetOutputBranch("AtEventH");

   auto SCModel = std::make_unique<AtEDistortionModel>();
   SCModel->SetCorrectionMaps(zlutFile.Data(), radlutFile.Data(), tralutFile.Data());
   auto SCTask = new AtSpaceChargeCorrectionTask(std::move(SCModel));
   SCTask->SetInputBranch("AtEventH");

   AtPRAtask *praTask = new AtPRAtask();
   praTask->SetInputBranch("AtEventCorrected");
   praTask->SetOutputBranch("AtPatternEvent");
   praTask->SetPersistence(true);
   praTask->SetClusterRadius(clusterRadius);
   praTask->SetClusterDistance(clusterDistance);
   // praTask->SetMaxNumHits(3000);
   // praTask->SetMinNumHits(100);
   // praTask->SetTcluster(8.0);

   // Fitting task
   Float_t gasMediumDensity = 0.083147;
   Float_t magneticField = 2.85;
   Int_t pdg = 1000010020; // 1000010020; 2212;
   Bool_t noMatEffects = 1;
   AtFITTER::AtGenfit::Exp exp = AtFITTER::AtGenfit::a1975;
   std::string elossFile = (std::string)dir.Data() + "/resources/energy_loss/proton_D2_600torr.txt";
   auto fitter = std::make_unique<AtFITTER::AtGenfit>(magneticField, 0.00001, 1000.0, elossFile, gasMediumDensity, pdg,
                                                      5, 20, noMatEffects);
   fitter->SetIonName("deuteron"); // deuteron proton
   fitter->SetMass(2.0135532);     // 2.0135532 1.00727646
   fitter->SetAtomicNumber(1);
   fitter->SetNumFitPoints(1.0);
   fitter->SetVerbosityLevel(1);
   fitter->SetSimulationConvention(0);
   // fitter->SetExpNum(exp);
   fitter->SetFitDirection(0);
   fitter->EnableMerging(1);
   fitter->EnableSingleVertexTrack(1);
   fitter->EnableReclustering(1, 15.0, 7.5);

   AtFitterTask *fitterTask = new AtFitterTask(std::move(fitter));
   fitterTask->SetPersistence(true);

   run->AddTask(unpackTask);
   // run->AddTask(filterTask);
   run->AddTask(psaTask);
   run->AddTask(SCTask);
   run->AddTask(praTask);
   run->AddTask(fitterTask);

   std::cout << "***** Starting Init ******" << std::endl;
   run->Init();
   std::cout << "***** Ending Init ******" << std::endl;

   // Get the number of events and unpack the whole run
   auto numEvents = unpackTask->GetNumEvents();
   std::cout << "Unpacking " << numEvents << " events. " << std::endl;

   run->Run(0, numEvents);

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
