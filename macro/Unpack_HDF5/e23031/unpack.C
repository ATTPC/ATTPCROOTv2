//Unpacks tpc files from /mnt/daqtesting/e18008_attpc_transfer to /mnt/analysis/e18008/rootMerg/

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m\e[1m"
#define WHITE   "\033[37m"

bool reduceFunc(AtRawEvent *evt)
{
   return (evt->GetNumPads() > 0) && evt->IsGood();
}

// Requires the TPC run number
void unpack(int runNumberS800 = 8, int runNumberATTPC = 8, TString parFileName = "ATTPC_e23031_H2_600Torr.par")
{
  //Load the library for unpacking and reconstruction
  gSystem->Load("libAtReconstruction.so");
  gSystem->Load("libAtS800.so");
  // gSystem->Load("libXMLParser.so");

  TStopwatch timer;
  timer.Start();

  //Set the input file
  //TString inputFile = TString::Format("/mnt/daqtesting/e23031_attpc_transfer/h5/run_%04d.h5", runNumberATTPC);
  TString inputFile = TString::Format("/media/aurio/Cris/11Li/h5/run_%04d.h5", runNumberATTPC);

  //Set the output file
  TString outputFile = TString::Format("temp/run_%04d_e23031.root", runNumberATTPC);

  std::cout << "Unpacking AT-TPC run " << runNumberATTPC << " from: " << inputFile << std::endl;
  std::cout << "Saving in: " << outputFile << std::endl;

  //Set the mapping for the TPC
 TString scriptfile = "e21018_pads_map.xml";
 TString parameterFile = "par.e23031/" + parFileName;

 // Set directories
 TString dir = gSystem->Getenv("VMCWORKDIR");
 TString scriptdir = dir + "/scripts/" + scriptfile;
 TString geomDir = dir + "/geometry/";
 gSystem->Setenv("GEOMPATH", geomDir.Data());
 TString digiParFile = dir + "/parameters/" + parameterFile;
 TString geoManFile = dir + "/geometry/ATTPC_v1.1.root";
 //TString geoManFile = dir + "/geometry/ATTPC_He1bar.root";//test 11/06/24 at 300Torr He

 // Create a run
 FairRunAna *run = new FairRunAna();
 run->SetSink(new FairRootFileSink(outputFile));
 run->SetGeomFile(geoManFile);

 // Set the parameter file
 FairRuntimeDb *rtdb = run->GetRuntimeDb();
 FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
 parIo1->open(digiParFile.Data(), "in");
 rtdb->setFirstInput(parIo1);
 rtdb->getContainer("AtDigiPar");

 // Create the detector map
 auto fAtMapPtr = std::make_shared<AtTpcMap>();
 fAtMapPtr->ParseXMLMap(scriptdir.Data());
 fAtMapPtr->GeneratePadPlane();

 /**** Should not have to change code between this line and the above star comment ****/
 fAtMapPtr->AddAuxPad({10, 0, 0, 34}, "mesh");
 fAtMapPtr->AddAuxPad({10, 0, 0, 0}, "trigger");
 fAtMapPtr->AddAuxPad({10, 0, 1, 34}, "obj_scint");
 fAtMapPtr->AddAuxPad({10, 0, 1, 0}, "E1up");
 fAtMapPtr->AddAuxPad({10, 0, 2, 34}, "E1down");
 fAtMapPtr->AddAuxPad({10, 0, 2, 0}, "obj_tac");
 fAtMapPtr->AddAuxPad({10, 0, 3, 34}, "alpha_sig");
 fAtMapPtr->AddAuxPad({10, 0, 3, 0}, "unassigned");

 // Create the unpacker task
 auto unpacker = std::make_unique<AtHDFUnpacker>(fAtMapPtr);
 unpacker->SetInputFileName(inputFile.Data());
 unpacker->SetNumberTimestamps(2);
 unpacker->SetBaseLineSubtraction(true);

 auto unpackTask = new AtUnpackTask(std::move(unpacker));
//  unpackTask->SetPersistence(false); // true keeps AtRawEvent
// unpackTask->SetPersistence(true); // true keeps AtRawEvent
 unpackTask->SetPersistence(false);

 // Create data reduction task
 AtDataReductionTask *reduceTask = new AtDataReductionTask();
 reduceTask->SetInputBranch("AtRawEvent");
 reduceTask->SetReductionFunction(std::function<bool(AtRawEvent*)>(reduceFunc));

 auto threshold = 30;

 //AtFilterSubtraction *filter = new AtFilterSubtraction(fAtMapPtr);
 //filter->SetThreshold(threshold);
 //filter->SetIsGood(false);

 //AtFilterTask *filterTask = new AtFilterTask(filter);
 //filterTask->SetPersistence(kFALSE);
 //filterTask->SetFilterAux(true);

 auto psa = std::make_unique<AtPSAMax>();
 psa->SetThreshold(threshold);

 AtPSAtask *psaTask = new AtPSAtask(std::move(psa));
 // psaTask->SetInputBranch("AtRawEventFiltered");
 // psaTask->SetOutputBranch("AtEventFiltered");
 psaTask->SetPersistence(kTRUE); // true keeps AtEventH

 /* merge task */

 AtRansacTask *ransacTask = new AtRansacTask();
 ransacTask->SetPersistence(kTRUE);
 ransacTask->SetVerbose(kFALSE);
 ransacTask->SetDistanceThreshold(15.0); // 12
 ransacTask->SetMinHitsLine(20);         // 10
 // in AtRansacTask parttern type set to line : auto patternType = AtPatterns::PatternType::kLine;
 ransacTask->SetAlgorithm(1); // 1=Homemade Ransac (default); 2=Homemade Mlesac; 3=Homemade Lmeds;//4
 ransacTask->SetRanSamMode(
    5); // SampleMethod { kUniform = 0, kChargeWeighted = 1, kGaussian = 2, kWeightedGaussian = 3, kWeightedY = 4 };//2
 ransacTask->SetChargeThreshold(50); // 150
 ransacTask->SetNumItera(500);

 // auto hash = HDFParserTask->CalculateHash(10,0,2,32);
 // HDFParserTask->SetAuxChannel(hash, "IonCb_32");
 // hash = HDFParserTask->CalculateHash(10,0,2,34);
 // HDFParserTask->SetAuxChannel(hash, "IonCb_34");




  std::vector<Double_t> S800par;
  S800par.push_back(0.);//x0_corr_tof
  S800par.push_back(0.);//afp_corr_tof
  S800par.push_back(0.);//afp_corr_dE
  S800par.push_back(0.);//x0_corr_dE
  S800par.push_back(0.);//rf_offset
  S800par.push_back(1.);//corrGainE1up
  S800par.push_back(1.);//corrGainE1down
  std::vector<Double_t> S800MTDCObjCorr;
  S800MTDCObjCorr.push_back(105);
  S800MTDCObjCorr.push_back(0.015);
  std::vector<Double_t> S800MTDCObjRange;
  S800MTDCObjRange.push_back(-300);
  S800MTDCObjRange.push_back(-100);
  std::vector<Double_t> S800MTDCXfRange;
  S800MTDCXfRange.push_back(100);
  S800MTDCXfRange.push_back(210);

  TString S800File = TString::Format("/media/aurio/Cris/11Li/dataS800/cal/run-%04d-00.root", runNumberS800);

  AtMergeTask *MergeEvt = new AtMergeTask();
  MergeEvt->SetS800File(S800File);
  MergeEvt->SetPersistence(kTRUE);
  MergeEvt->SetOptiEvtDelta(5);
  MergeEvt->SetGlom(2);
  MergeEvt->SetTsDelta(1205);//before run 56: 805//run 56 and after: 1205 (I think this comment is refered to the previous experiment)
  //MergeEvt->SetPID1cut("/mnt/analysis/e21018/codes/ATTPCROOTv2-1/macro/Unpack_HDF5/e21018_S800/rootPID/32MgBeam_New.root"); // can have multiple gates for PID1
  //MergeEvt->SetPID1cut("/mnt/analysis/e21018/codes/ATTPCROOTv2-1/macro/Unpack_HDF5/e21018_S800/rootPID/33AlBeam_New.root"); // can have multiple gates for PID1
  //MergeEvt->SetPID3cut("/mnt/analysis/e21018/codes/ATTPCROOTv2-1/macro/Unpack_HDF5/e21018_S800/rootPID/Na32CE_New.root"); // can have multiple gates for PID2
  //MergeEvt->SetPID3cut("/mnt/analysis/e21018/codes/ATTPCROOTv2-1/macro/Unpack_HDF5/e21018_S800/rootPID/Na31CE_New.root"); // can have multiple gates for PID2
  //MergeEvt->SetPID3cut("/mnt/analysis/e21018/codes/ATTPCROOTv2-1/macro/Unpack_HDF5/e21018_S800/rootPID/Mg32CE_New.root"); // can have multiple gates for PID2
  //MergeEvt->SetPID3cut("/mnt/analysis/e21018/codes/ATTPCROOTv2-1/macro/Unpack_HDF5/e21018_S800/rootPID/Mg33CE_New.root"); // can have multiple gates for

  MergeEvt->SetParameters(S800par);
  MergeEvt->SetTofObjCorr(S800MTDCObjCorr);
  MergeEvt->SetMTDCObjRange(S800MTDCObjRange);
  MergeEvt->SetMTDCXfRange(S800MTDCXfRange);
  //------------------------------------------------------------------------------
  // for runs with no clock sync (ex: 144 to 168 and 2271 to 2279, 2275 needs a special treatment see AtMergeTask.cc)
  // uncomment following

  //MergeEvt->SetATTPCClock(kTRUE);         // if kTRUE for HDFParser SetTimestampIndex(1) (use internal timestamp)
  //MergeEvt->SetATTPCClockFreq(9.9994345); // empirical frequency factor between ATTPC internal clock and S800 clock//
                                          // fluctuates that is why a glom of 1500 is defined
  //MergeEvt->SetGlom(1500);
  //MergeEvt->SetTsDelta(0); 
  //MergeEvt->ShowTSDiagnostic(kTRUE); //kTRUE If want to draw the timestamp matching figures between s800 and attpc.

  //------------------------------------------------------------------------------

  run->AddTask(unpackTask);
  run->AddTask(MergeEvt);
  run->AddTask(reduceTask);
  //run->AddTask(filterTask);
  run->AddTask(psaTask);
  run->AddTask(ransacTask);

  run -> Init();

  //Get the number of AT-TPC events
  
  //auto numATTPCEvents = 800;
  auto numATTPCEvents = unpackTask->GetNumEvents();

  //Get the number of S800 events
  auto numS800Events = MergeEvt->GetS800TsSize();

  std::cout << "Unpacking " << numATTPCEvents << " AT-TPC events." << std::endl;

  //return;
  run->Run(0, numATTPCEvents);

  //Get the number of TS matched events between S800 and AT-TPC
  auto numSyncEvents = MergeEvt->GetMergedTsSize();

  // -----   Finish   -------------------------------------------------------
  timer.Stop();
  Double_t rtime = timer.RealTime();
  Double_t ctime = timer.CpuTime();
  cout << endl << endl;
  cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
  cout << "Total AT-TPC events: " << numATTPCEvents << endl;
  cout << "Total S800 events: " << numS800Events << endl;
  cout << "Total matched events: " << numSyncEvents << endl;
  cout << "Ratio TS matched events (totalMatched/totalS800): " <<(Float_t)numSyncEvents/numS800Events<< endl;
  cout << endl;
  // ------------------------------------------------------------------------

  return 0;
}
