//Unpacks tpc files from /mnt/daqtesting/e18008_attpc_transfer to /mnt/analysis/e18008/rootMerg/

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m\e[1m"
#define WHITE   "\033[37m"


struct auxchannel
{
  std::string name;
  uint8_t cobo;
  uint8_t asad;
  uint8_t aget;
  uint8_t channel;
};

// Requires the TPC run number
void unpack_new(int runNumberS800=2063, int runNumberATTPC=63)
{
  //Load the library for unpacking and reconstruction
  gSystem->Load("libATTPCReco.so");
  gSystem->Load("libS800.so");
  gSystem->Load("libXMLParser.so");

  TStopwatch timer;
  timer.Start();

  //Set the input file
  //TString inputFile = TString::Format("hdf5Files/run_%04d.h5", runNumber);
  //TString inputFile = TString::Format("/mnt/daqtesting/e18008_attpc_transfer/h5/run_%04d.h5", runNumberATTPC);
  TString inputFile = TString::Format("/mnt/rawdata/e18008_attpc/h5/run_%04d.h5", runNumberATTPC);
   //TString inputFile = TString::Format("/mnt/rawdata/ceclub/giraud/attpc/ATTPCROOTv2/macro/Unpack_HDF5/hdf5Files/run_0002.h5", runNumberATTPC);

  //Set the output file
  // TString outputFile = TString::Format("/mnt/analysis/e18008/rootMerg/run_2%03d_%04d.root", runNumberS800, runNumberATTPC);
  TString outputFile = TString::Format("/mnt/analysis/e18008/rootMerg/jcz_test/run_%04d_%04d.root", runNumberS800, runNumberATTPC);
  // TString outputFile = TString::Format("/projects/ceclub/giraud/attpc/ATTPCROOTv2/macro/Simulation/d2He/Analyis_d2He/merged_run0002_48Ca.root", runNumberS800, runNumberATTPC);

  std::cout << "Unpacking AT-TPC run " << runNumberATTPC << " from: " << inputFile << std::endl;
  std::cout << "Saving in: " << outputFile << std::endl;

  //Set the mapping for the TPC
 // TString scriptfile = "e12014_pad_mapping.xml";//"Lookup20150611.xml";
 TString scriptfile = "e12014_pad_map_size.xml";
 // TString scriptfile = "Lookup20150611.xml";//"Lookup20150611.xml";
  //TString parameterFile = "ATTPC.testnew.par";
  TString parameterFile = "ATTPC.d2He.par";

  //Set directories
  TString dir = gSystem->Getenv("VMCWORKDIR");
  TString scriptdir = dir + "/scripts/"+ scriptfile;
  TString geomDir   = dir + "/geometry/";
  gSystem -> Setenv("GEOMPATH", geomDir.Data());
  TString digiParFile = dir + "/parameters/" + parameterFile;
  //TString geoManFile  = dir + "/geometry/ATTPC_v1.1.root";
  //TString geoManFile  = dir + "/geometry/ATTPC_He1bar.root";
  TString geoManFile  = dir + "/geometry/ATTPC_d2He_07atm.root";


  //Create a run
  FairRunAna* run = new FairRunAna();
  run -> SetOutputFile(outputFile);
  run -> SetGeomFile(geoManFile);

  //Set the parameter file
  FairRuntimeDb* rtdb = run->GetRuntimeDb();
  FairParAsciiFileIo* parIo1 = new FairParAsciiFileIo();
  parIo1 -> open(digiParFile.Data(), "in");
  rtdb -> setSecondInput(parIo1);


  //--------Auxiliary channels

  //Hash table: cobo, asad, aget, channel
  std::vector<auxchannel> aux_channels;
  auxchannel ch_1{"mesh",10,0,0,34};
  aux_channels.push_back(ch_1);
  auxchannel ch_2{"trigger",10,0,0,0};
  aux_channels.push_back(ch_2);
  auxchannel ch_3{"obj_scint",10,0,1,34};
  aux_channels.push_back(ch_3);
  auxchannel ch_4{"E1up",10,0,1,0};
  aux_channels.push_back(ch_4);
  auxchannel ch_5{"E1down",10,0,2,34};
  aux_channels.push_back(ch_5);
  auxchannel ch_6{"obj_tac",10,0,2,0};
  aux_channels.push_back(ch_6);
  auxchannel ch_7{"alpha_sig",10,0,3,34};
  aux_channels.push_back(ch_7);
  auxchannel ch_8{"unassigned",10,0,3,0};
  aux_channels.push_back(ch_8);
   //---------End of auxiliary channel setup


  //Create the unpacker task
  ATHDFParserTask* HDFParserTask = new ATHDFParserTask();
 // HDFParserTask->SetPersistence(kFALSE); // do no write Rawevents
  HDFParserTask->SetPersistence(kTRUE);
  HDFParserTask->SetATTPCMap(scriptdir.Data());
  HDFParserTask->SetFileName(inputFile.Data());
  HDFParserTask->SetOldFormat(false);
  HDFParserTask->SetTimestampIndex(2);
  HDFParserTask->SetBaseLineSubtraction(kTRUE);

  for(auto iaux : aux_channels){
    auto hash  = HDFParserTask->CalculateHash(iaux.cobo,iaux.asad,iaux.aget,iaux.channel);
    auto isaux = HDFParserTask->SetAuxChannel(hash,iaux.name);
  }

 // auto hash = HDFParserTask->CalculateHash(10,0,2,32);
 // HDFParserTask->SetAuxChannel(hash, "IonCb_32");
 // hash = HDFParserTask->CalculateHash(10,0,2,34);
 // HDFParserTask->SetAuxChannel(hash, "IonCb_34");



 // Double_t x0_corr_tof = 0.101259;
 // Double_t afp_corr_tof = 1177.02;
 // Double_t afp_corr_dE = 61.7607;
 // Double_t x0_corr_dE = -0.0403;
 // Double_t rf_offset = 0.0;
 // Double_t corrGainE1up = 0.6754;
 // Double_t corrGainE1down = 1.;

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
  ATMergeTask *MergeEvt = new ATMergeTask();
  MergeEvt->SetS800File(S800File);
  MergeEvt->SetPersistence(kTRUE);
  // MergeEvt->SetOptiEvtDelta(150);//old way with fit, //missed few events during the tests with 100
  MergeEvt->SetOptiEvtDelta(5);
  MergeEvt->SetGlom(2);
  MergeEvt->SetTsDelta(1192);//run60//61... //181
  //MergeEvt->SetTsDelta(792);run59/has to be adjusted, function of the drift velocity
  //MergeEvt->SetTsDelta(1272);//has to be adjusted, function of the drift velocity
  MergeEvt->SetPID1cut("rootPID/XfObjObj_run60to70.root");//can have multiple gates for PID1
  MergeEvt->SetPID2cut("rootPID/ICSumObj_run60to70.root");//can have multiple gates for PID2
  //MergeEvt->SetPID1cut("rootPID/XfObjObj_run115.root");//can have multiple gates for PID1
  //MergeEvt->SetPID2cut("rootPID/ICSumObj_run115.root");//can have multiple gates for PID2
  MergeEvt->SetParameters(S800par);
  MergeEvt->SetTofObjCorr(S800MTDCObjCorr);
  MergeEvt->SetMTDCObjRange(S800MTDCObjRange);
  MergeEvt->SetMTDCXfRange(S800MTDCXfRange);



  //Create PSA task
  ATPSATask *psaTask = new ATPSATask();
  psaTask -> SetPersistence(kTRUE);//kFALSE
  psaTask -> SetThreshold(160);//50
  psaTask -> SetThresholdLow(80);
  psaTask -> SetPSAMode(3); //NB: 1 is ATTPC - 2 is pATTPC - 3 Filter for ATTPC - 4: Full Time Buckets
  psaTask -> SetMeanK(3);
  psaTask -> SetStddevMulThresh(0.00001);
  //psaTask -> SetMaxFinder();
  psaTask -> SetPeakFinder(); //TSpectrum
  psaTask -> SetBackGroundInterpolation(kTRUE); //TSpectrum


/*
  ATRansacTask *RandTask = new ATRansacTask();
  RandTask -> SetPersistence(kTRUE);
  RandTask -> SetVerbose(kFALSE);
  RandTask -> SetDistanceThreshold(10.0);
  RandTask -> SetTiltAngle(0);
  RandTask -> SetMinHitsLine(30);
  //RandTask -> SetFullMode();
*/
  ATRansacTask *RandTask = new ATRansacTask();
  RandTask ->SetPersistence(kTRUE);
  RandTask ->SetIsReprocess(kFALSE);
  //RandTask ->SetModelType(1);
  //RandTask ->SetFullMode();
  RandTask->SetTiltAngle(0.0);
  RandTask->SetDistanceThreshold(12.0);//15.
  RandTask->SetMinHitsLine(12);//7
  RandTask->SetAlgorithm(3); // 0=PCL ransac; 1=Homemade Ransac; 2=Homemade Mlesac; 3=Homemade Lmeds;
  RandTask->SetRanSamMode(3);// 0=Uniform; 1=Gaussian; 2=Weighted; 3=Gaussian+Weighted



  //Add unpacker to the run
  run -> AddTask(HDFParserTask);
  run -> AddTask(MergeEvt);
  run -> AddTask(psaTask);
  run -> AddTask(RandTask);

  run -> Init();

  //Get the number of events and unpack the whole run
  auto numEvents = HDFParserTask->GetNumEvents()/2;

  //numEvents = 1700;//217;
  std::cout << "Unpacking " << numEvents << " events. " << std::endl;

  //return;
 // run->Run(0,numEvents);
  run->Run(0,200);


  std::cout << std::endl << std::endl;
  std::cout<<" Check MergeTask :: S800 events "<<YELLOW<<MergeEvt->GetS800TsSize()<<RESET<<" AT-TPC events "<<YELLOW<<numEvents<<RESET<<"  Merged TS size "<<YELLOW<<
    MergeEvt->GetMergedTsSize()<<RESET<<std::endl;
  if((double)MergeEvt->GetMergedTsSize()/(double)MergeEvt->GetS800TsSize()<1)
    std::cout<<" ! WARNING Ratio (merged/S800) "<<RED<<(double)MergeEvt->GetMergedTsSize()/(double)MergeEvt->GetS800TsSize()<<RESET<<std::endl;
  if((double)MergeEvt->GetMergedTsSize()/(double)MergeEvt->GetS800TsSize()==1)
  std::cout<<" Ratio (merged/S800) "<<GREEN<<(double)MergeEvt->GetMergedTsSize()/(double)MergeEvt->GetS800TsSize()<<RESET<<std::endl;
  std::cout<<" Check MergeTask :: S800 events "<<MergeEvt->GetS800TsSize()<<" AT-TPC events "<<numEvents<<"  Merged TS size "<<
    MergeEvt->GetMergedTsSize()<<std::endl;
  std::cout << std::endl;
  std::cout << "Done unpacking events"  << std::endl << std::endl;
  std::cout << "- Output file : " << outputFile << std::endl << std::endl;


  //std::cout << "Done unpacking events"  << std::endl << std::endl;
  //std::cout << "- Output file : " << outputFile << std::endl << std::endl;
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
