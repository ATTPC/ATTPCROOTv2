#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

bool reduceFunc(AtRawEvent *evt){
  return (evt->GetNumPads() > 0) && evt->IsGood();
}

void unpack_rcnp_Si(int run_num = 1001){
  // Load the library for unpacking and reconstruction
  gSystem->Load("libAtRecoMediumnstruction.so");

  TStopwatch timer;
  timer.Start();

  TString fileName = TString::Format("run_%04d", run_num);
  TString parameterFile = "RCNP/ATTPC.E565.par";
  TString mappath = "";
  //TString filepath = "/mnt/merger/E565/h5/";
  TString filepath = "/data/tempMergedData/E565/";
  TString fileExt = ".h5";
  TString outputpath = "/data/ATTPCROOTv2_results/E565/UnpackerOutput/";

  TString inputFile = filepath + fileName + fileExt;
  TString scriptfile = "rcnp_map.xml";
  TString dir = getenv("VMCWORKDIR");
  TString mapDir = dir + "/scripts/" + scriptfile;
  TString scriptdir = dir + "/scripts/" + scriptfile;
  TString dataDir = dir + "/macro/data/";
  TString geomDir = dir + "/geometry/";
  gSystem->Setenv("GEOMPATH", geomDir.Data());
  TString outputFile = outputpath + fileName + "_onlySiData.root";
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
  unpackTask->SetPersistence(true); // true

  run->AddTask(unpackTask);

  std::cout << "***** Starting Init ******" << std::endl;
  run->Init();
  std::cout << "***** Ending Init ******" << std::endl;

  // Get the number of events and unpack the whole run
  auto numEvents = unpackTask->GetNumEvents();
  //numEvents = 100;
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
