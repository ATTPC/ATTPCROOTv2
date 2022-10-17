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
void unpack_new(int runNumberS800 = 2272, int runNumberATTPC = 272)
{
  //Load the library for unpacking and reconstruction
  gSystem->Load("libAtReconstruction.so");
  gSystem->Load("libAtS800.so");
  // gSystem->Load("libXMLParser.so");

  TStopwatch timer;
  timer.Start();

  //Set the input file
  //TString inputFile = TString::Format("hdf5Files/run_%04d.h5", runNumber);
  //TString inputFile = TString::Format("/mnt/daqtesting/e18008_attpc_transfer/h5/run_%04d.h5", runNumberATTPC);
  TString inputFile = TString::Format("/mnt/rawdata/e18008_attpc/h5/run_%04d.h5", runNumberATTPC);
   //TString inputFile = TString::Format("/mnt/rawdata/ceclub/giraud/attpc/ATTPCROOTv2/macro/Unpack_HDF5/hdf5Files/run_0002.h5", runNumberATTPC);

  //Set the output file
  // TString outputFile = TString::Format("/mnt/analysis/e18008/rootMerg/run_2%03d_%04d.root", runNumberS800, runNumberATTPC);
  TString outputFile =
     TString::Format("/mnt/analysis/e18008/rootMerg/giraud/run_%04d_%04d_test15.root", runNumberS800, runNumberATTPC);
  // TString outputFile = TString::Format("/projects/ceclub/giraud/attpc/ATTPCROOTv2/macro/Simulation/d2He/Analyis_d2He/merged_run0002_48Ca.root", runNumberS800, runNumberATTPC);

  std::cout << "Unpacking AT-TPC run " << runNumberATTPC << " from: " << inputFile << std::endl;
  std::cout << "Saving in: " << outputFile << std::endl;

  //Set the mapping for the TPC
 TString scriptfile = "e12014_pad_map_size.xml";
 // TString parameterFile = "ATTPC.d2He.par";
 TString parameterFile = TString::Format("par.e18008/ATTPC.d2He.par_%04d_%04d_newCode", runNumberS800, runNumberATTPC);

 // Set directories
 TString dir = gSystem->Getenv("VMCWORKDIR");
 TString scriptdir = dir + "/scripts/" + scriptfile;
 TString geomDir = dir + "/geometry/";
 gSystem->Setenv("GEOMPATH", geomDir.Data());
 TString digiParFile = dir + "/parameters/" + parameterFile;
 TString geoManFile = dir + "/geometry/ATTPC_v1.1.root";

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
 unpackTask->SetPersistence(false); // true keeps AtRawEvent

 // Create data reduction task
 AtDataReductionTask *reduceTask = new AtDataReductionTask();
 reduceTask->SetInputBranch("AtRawEvent");
 reduceTask->SetReductionFunction(&reduceFunc);

 auto threshold = 100; // 45//100 run63

 AtFilterSubtraction *filter = new AtFilterSubtraction(fAtMapPtr);
 filter->SetThreshold(threshold);
 filter->SetIsGood(false);

 AtFilterTask *filterTask = new AtFilterTask(filter);
 filterTask->SetPersistence(kFALSE);
 filterTask->SetFilterAux(true);

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
 ransacTask->SetDistanceThreshold(12.0); // 12
 ransacTask->SetMinHitsLine(15);         // 10
 // in AtRansacTask parttern type set to line : auto patternType = AtPatterns::PatternType::kLine;
 ransacTask->SetAlgorithm(1); // 1=Homemade Ransac (default); 2=Homemade Mlesac; 3=Homemade Lmeds;//4
 ransacTask->SetRanSamMode(
    5); // SampleMethod { kUniform = 0, kChargeWeighted = 1, kGaussian = 2, kWeightedGaussian = 3, kWeightedY = 4 };//2
 ransacTask->SetChargeThreshold(100); // 150
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
  S800MTDCObjCorr.push_back(70.);
  S800MTDCObjCorr.push_back(0.0085);
  std::vector<Double_t> S800MTDCObjRange;
  S800MTDCObjRange.push_back(-120);
  S800MTDCObjRange.push_back(-20);
  std::vector<Double_t> S800MTDCXfRange;
  S800MTDCXfRange.push_back(160);
  S800MTDCXfRange.push_back(240);
  // TString S800File = "/projects/ceclub/giraud/s800root_jorge/s800root/files/rootFiles/cal/test-runs800-48Ca-CAL-0001-new.root";
  // TString S800File = TString::Format("/mnt/analysis/e18008/rootS800/cal/run-2%03d-00.root", runNumberS800);
  TString S800File = TString::Format("/mnt/analysis/e18008/rootS800/cal/run-%04d-00.root", runNumberS800);
  AtMergeTask *MergeEvt = new AtMergeTask();
  MergeEvt->SetS800File(S800File);
  MergeEvt->SetPersistence(kTRUE);
  MergeEvt->SetOptiEvtDelta(5);
  MergeEvt->SetGlom(2);
  MergeEvt->SetTsDelta(1192);//run60//61... //181
  //MergeEvt->SetTsDelta(792);run59/has to be adjusted, function of the drift velocity
  //MergeEvt->SetTsDelta(1272);//has to be adjusted, function of the drift velocity
  MergeEvt->SetPID1cut("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/"
                       "XfObjObj_run60to70.root"); // can have multiple gates for PID1
  MergeEvt->SetPID3cut("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/"
                       "ICSumObj_run60to70.root"); // can have multiple gates for PID2
  //MergeEvt->SetPID1cut("rootPID/XfObjObj_run115.root");//can have multiple gates for PID1
  //MergeEvt->SetPID2cut("rootPID/ICSumObj_run115.root");//can have multiple gates for PID2
  MergeEvt->SetParameters(S800par);
  MergeEvt->SetTofObjCorr(S800MTDCObjCorr);
  MergeEvt->SetMTDCObjRange(S800MTDCObjRange);
  MergeEvt->SetMTDCXfRange(S800MTDCXfRange);
  //------------------------------------------------------------------------------
  // for runs with no clock sync (ex: 144 to 168 and 2271 to 2279, 2275 needs a special treatment see AtMergeTask.cc)
  // uncomment following

  MergeEvt->SetATTPCClock(kTRUE);         // if kTRUE for HDFParser SetTimestampIndex(1) (use internal timestamp)
  MergeEvt->SetATTPCClockFreq(9.9994345); // empirical frequency factor between ATTPC internal clock and S800 clock//
                                          // fluctuates that is why a glom of 1500 is defined
  // MergeEvt->SetATTPCClockFreq(9.9994351);//for run 278
  // MergeEvt->SetATTPCClockFreq(9.9994333);//for run 257-261 //9.9994335
  // MergeEvt->SetATTPCClockFreq(9.9994339);//for run 160,161//9.9994335
  // MergeEvt->SetATTPCClockFreq(9.9994349);//for run 155 159 166 168 149
  MergeEvt->SetGlom(1500);
  MergeEvt->SetTsDelta(0); // 4186 run149, 0 for all

  //------------------------------------------------------------------------------

  run->AddTask(unpackTask);
  run->AddTask(MergeEvt);
  run->AddTask(reduceTask);
  run->AddTask(filterTask);
  run->AddTask(psaTask);
  run->AddTask(ransacTask);

  run -> Init();

  //Get the number of events and unpack the whole run
  auto numEvents = unpackTask->GetNumEvents();

  std::cout << "Unpacking " << numEvents << " events. " << std::endl;

  //return;
  run->Run(0, numEvents);
  // run->Run(0,3000);

  // -----   Finish   -------------------------------------------------------
  timer.Stop();
  Double_t rtime = timer.RealTime();
  Double_t ctime = timer.CpuTime();
  cout << endl << endl;
  cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
  cout << endl;
  // ------------------------------------------------------------------------

  return 0;
}
