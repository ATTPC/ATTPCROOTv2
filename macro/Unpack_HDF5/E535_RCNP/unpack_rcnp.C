#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

bool reduceFunc(AtRawEvent *evt){
  return (evt->GetNumPads() > 0) && evt->IsGood();
}

void unpack_rcnp(int run_num = 2011){
  // Load the library for unpacking and reconstruction
  gSystem->Load("libAtRecoMediumnstruction.so");

  TStopwatch timer;
  timer.Start();

  TString fileName = TString::Format("run_%04d", run_num);
  TString parameterFile = "RCNP/ATTPC.E535.par";
  TString mappath = "";
  TString filepath = "/data/tempMergedData/E535/";
  TString fileExt = ".h5";
  TString outputpath = "/data/ATTPCROOTv2_results/E535/UnpackerOutput/";

  TString inputFile = filepath + fileName + fileExt;
  TString scriptfile = "rcnp_map_size.xml";
  TString dir = getenv("VMCWORKDIR");
  TString mapDir = dir + "/scripts/" + scriptfile;
  TString scriptdir = dir + "/scripts/" + scriptfile;
  TString dataDir = dir + "/macro/data/";
  TString geomDir = dir + "/geometry/";
  gSystem->Setenv("GEOMPATH", geomDir.Data());
  TString outputFile = outputpath + fileName + ".root";
  TString loggerFile = dataDir + "ATTPCLog.log";
  TString digiParFile = dir + "/parameters/" + parameterFile;
  TString geoManFile = dir + "/geometry/RCNP_ATTPC_494_3torr.root";

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

  //auto unpacker = std::make_unique<AtHDFUnpacker>(fAtMapPtr);
  //auto unpacker = std::make_unique<AtFRIBLinkedHDFUnpacker>(fAtMapPtr);
  auto unpacker = std::make_unique<AtFRIBSiUnpacker>(fAtMapPtr);
  unpacker->SetInputFileName(inputFile.Data());
  unpacker->SetNumberTimestamps(2);
  unpacker->SetBaseLineSubtraction(true);

  auto unpackTask = new AtUnpackTask(std::move(unpacker));
  unpackTask->SetPersistence(false); // true

  auto thresholdSi = 50;
  auto psaSi = std::make_unique<AtPSASi>();
  psaSi->SetThreshold(thresholdSi);

  AtSiTask *siTask = new AtSiTask(std::move(psaSi));
  siTask->SetPersistence(kTRUE);

  auto thresholdGagg = 10;
  auto psaGagg = std::make_unique<AtPSASi>();
  psaGagg->SetThreshold(thresholdGagg);
  AtGaggTask *gaggTask = new AtGaggTask(std::move(psaGagg));
  gaggTask->SetPersistence(kTRUE);

  AtFilterSubtraction *filter = new AtFilterSubtraction(fAtMapPtr);
  filter->SetThreshold(50);
  filter->SetIsGood(false);

  AtFilterTask *filterTask = new AtFilterTask(filter);
  filterTask->SetPersistence(false);
  filterTask->SetFilterAux(false);

  auto threshold = 30;

  // auto psa = new AtPSASimple2();
  auto psa = new AtPSAMax();
  psa->SetThreshold(threshold);
  // psa->SetMaxFinder();

  // Create PSA task
  AtPSAtask *psaTask = new AtPSAtask(psa);
  psaTask->SetPersistence(kTRUE);// false
  // psaTask->SetInputBranch("AtRawEventFiltered");
  psaTask->SetOutputBranch("AtEventH");

  auto SCModel = std::make_unique<AtEDistortionModel>();
  SCModel->SetCorrectionMaps(zlutFile.Data(), radlutFile.Data(), tralutFile.Data());
  auto SCTask = new AtSpaceChargeCorrectionTask(std::move(SCModel));
  SCTask->SetInputBranch("AtEventH");

  AtRansacTask *ransacTask = new AtRansacTask();
  ransacTask->SetPersistence(kTRUE);
  ransacTask->SetVerbose(kTRUE);
  ransacTask->SetDistanceThreshold(12.0); //12
  ransacTask->SetMinHitsLine(30); //10
  // in AtRansacTask pattern tyepe set to line: auto patternType = AtPatternType::kLine;
  //1=Homemade Ransac(default); 2=Homemade Mlesac; 3=Homemade Lmeds; //4
  ransacTask->SetAlgorithm(1);
  // SampleMethod{kUniform=0,kChargeWeighted=1,kGaussian=2, kWeightGaussian=3, kWeightedY=4};//2
  ransacTask->SetRanSamMode(5);
  ransacTask->SetChargeThreshold(20); //150
  // ransacTask->SetNumItera(500);

/*
  //Create the AtPatternModification task.
  std::vector<std::unique_ptr<AtPatternModification>> patternModifications;
  auto braggCurveFinder = std::make_unique<AtBraggCurveFinder>();
  braggCurveFinder->SetBinSize(6.0);
  braggCurveFinder->SetNumSmoothingSteps(200);
  braggCurveFinder->SetTSSemiWidth(15); // Looking at the traces width in the run_eve macro.
  patternModifications.push_back(std::move(braggCurveFinder));
  AtPatternModificationTask *patternModTask = new AtPatternModificationTask(std::move(patternModifications));
  //  patternModTask->SetOutputBranch("AtPatternEvent");
  patternModTask->SetPersistence(kTRUE);

   // Create the AtFitterTask task.
   std::vector<std::unique_ptr<AtTools::AtELossModel>> eLossModels;

   double density = 1.4069e-3;
   std::vector<std::tuple<int, int, int>> materialComponents;
   materialComponents.push_back(std::make_tuple(12, 6, 3));
   materialComponents.push_back(std::make_tuple(2, 1, 8));

   std::unique_ptr<AtTools::AtELossCATIMA> eLossModelC3D8_p = std::make_unique<AtTools::AtELossCATIMA>(density, "CATima_C3D8_494_3Torr_p");
   eLossModelC3D8_p->SetMaterial(materialComponents);
   eLossModelC3D8_p->SetProjectile(1, 1, 1.007825031898);
   eLossModels.push_back(std::move(eLossModelC3D8_p));

   std::unique_ptr<AtTools::AtELossCATIMA> eLossModelC3D8_12Be = std::make_unique<AtTools::AtELossCATIMA>(density, "CATima_C3D8_494_3Torr_12Be");
   eLossModelC3D8_12Be->SetMaterial(materialComponents);
   eLossModelC3D8_12Be->SetProjectile(12, 4, 12.026922082);
   eLossModels.push_back(std::move(eLossModelC3D8_12Be));

   std::unique_ptr<EventFit::AtBraggCurveFitter> braggCurveFitter = std::make_unique<EventFit::AtBraggCurveFitter>(std::move(eLossModels));
   braggCurveFitter->SetEstimatedAmplitudeFactor(50000);
   braggCurveFitter->SetEstimatedAmplitudeFactorPrecision(8000);
   braggCurveFitter->SetDistanceThreshold(10);
   braggCurveFitter->Init();

   AtFitterTask *fitterTask = new AtFitterTask(std::move(braggCurveFitter));
   fitterTask->SetPersistence(kTRUE);
   fitterTask->SetInputBranch("AtPatternEventModified");
   fitterTask->SetFitMetadataBranch("AtFitMetadata");*/

  run->AddTask(unpackTask);
  run->AddTask(siTask);
  run->AddTask(gaggTask);
  // run->AddTask(filterTask);
  run->AddTask(psaTask);
  run->AddTask(SCTask);
  run->AddTask(ransacTask);
  //run->AddTask(patternModTask);
  //run->AddTask(fitterTask);

  std::cout << "***** Starting Init ******" << std::endl;
  run->Init();
  std::cout << "***** Ending Init ******" << std::endl;

  // Get the number of events and unpack the whole run
  auto numEvents = unpackTask->GetNumEvents();
  //numEvents = 1000;
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

