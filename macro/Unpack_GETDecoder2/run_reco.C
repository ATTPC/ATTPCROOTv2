void run_reco
(TString dataFile = "output.root",TString parameterFile = "ATTPC.e15503b.par",
TString mappath="/data/ar46/run_0085/")
{

  // -----   Timer   --------------------------------------------------------
 TStopwatch timer;
 timer.Start();
 // ------------------------------------------------------------------------

  gSystem->Load("libXMLParser.so");
  // -----------------------------------------------------------------
  // Set file names
  TString scriptfile = "Lookup20150611.xml";
  TString dir = getenv("VMCWORKDIR");
  TString scriptdir = dir + "/scripts/"+ scriptfile;
  TString dataDir = dir + "/macro/data/";
  TString geomDir = dir + "/geometry/";
  gSystem -> Setenv("GEOMPATH", geomDir.Data());

  //TString inputFile   = dataDir + name + ".digi.root";
  //TString outputFile  = dataDir + "output.root";
  TString outputFile  = "output_reco.root";
  //TString mcParFile   = dataDir + name + ".params.root";
  TString loggerFile  = dataDir + "ATTPCLog_Reco.log";
  TString digiParFile = dir + "/parameters/" + parameterFile;
  TString geoManFile  = dir + "/geometry/ATTPC_v1.2.root";

  TString inimap   = mappath + "inhib.txt";
  TString lowgmap  = mappath + "lowgain.txt";
  TString xtalkmap = mappath + "beampads_e15503b.txt";

  // -----------------------------------------------------------------
  // Logger
  FairLogger *fLogger = FairLogger::GetLogger();
  fLogger -> SetLogFileName(loggerFile);
  fLogger -> SetLogToScreen(kTRUE);
  fLogger -> SetLogToFile(kTRUE);
  fLogger -> SetLogVerbosityLevel("LOW");

  FairRunAna* run = new FairRunAna();
  run ->SetInputFile(dataFile);
  run -> SetOutputFile(outputFile);
  //run -> SetGeomFile("../geometry/ATTPC_Proto_v1.0.root");
  run -> SetGeomFile(geoManFile);

  FairRuntimeDb* rtdb = run->GetRuntimeDb();
  FairParAsciiFileIo* parIo1 = new FairParAsciiFileIo();
  parIo1 -> open(digiParFile.Data(), "in");
  //FairParRootFileIo* parIo2 = new FairParRootFileIo();
  //parIo2 -> open("param.dummy_proto.root");
 // rtdb -> setFirstInput(parIo2);
  rtdb -> setSecondInput(parIo1);


  ATHoughTask *HoughTask = new ATHoughTask();
	HoughTask ->SetPersistence();
	//HoughTask ->SetLinearHough();
	HoughTask ->SetCircularHough();
  HoughTask ->SetHoughThreshold(100.0); // Charge threshold for Hough
	run -> AddTask(HoughTask);


  run -> Init();

  //run -> RunOnTBData();
  run->Run(0,200);

  std::cout << std::endl << std::endl;
  std::cout << "Reconstruction macro finished succesfully."  << std::endl << std::endl;
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

/*fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo0_run_0122_14-08-15_13h27m28s.graw",0);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo0_run_0122_14-08-15_13h27m28s.1.graw",0);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo0_run_0122_14-08-15_13h27m28s.2.graw",0);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo0_run_0122_14-08-15_13h27m28s.3.graw",0);

fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo1_run_0122_14-08-15_13h27m28s.graw",1);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo1_run_0122_14-08-15_13h27m28s.1.graw",1);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo1_run_0122_14-08-15_13h27m28s.2.graw",1);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo1_run_0122_14-08-15_13h27m28s.3.graw",1);

fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo2_run_0122_14-08-15_13h27m29s.graw",2);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo2_run_0122_14-08-15_13h27m29s.1.graw",2);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo2_run_0122_14-08-15_13h27m29s.2.graw",2);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo2_run_0122_14-08-15_13h27m29s.3.graw",2);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo2_run_0122_14-08-15_13h27m29s.4.graw",2);

fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo3_run_0122_14-08-15_13h27m29s.graw",3);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo3_run_0122_14-08-15_13h27m29s.1.graw",3);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo3_run_0122_14-08-15_13h27m29s.2.graw",3);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo3_run_0122_14-08-15_13h27m29s.3.graw",3);

fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo4_run_0122_14-08-15_13h27m29s.graw",4);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo4_run_0122_14-08-15_13h27m29s.1.graw",4);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo4_run_0122_14-08-15_13h27m29s.2.graw",4);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo4_run_0122_14-08-15_13h27m29s.3.graw",4);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo4_run_0122_14-08-15_13h27m29s.4.graw",4);

fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo5_run_0122_14-08-15_13h27m29s.graw",5);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo5_run_0122_14-08-15_13h27m29s.1.graw",5);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo5_run_0122_14-08-15_13h27m29s.2.graw",5);

fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo6_run_0122_14-08-15_13h27m29s.graw",6);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo6_run_0122_14-08-15_13h27m29s.1.graw",6);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo6_run_0122_14-08-15_13h27m29s.2.graw",6);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo6_run_0122_14-08-15_13h27m29s.3.graw",6);

fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo7_run_0122_14-08-15_13h27m29s.graw",7);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo7_run_0122_14-08-15_13h27m29s.1.graw",7);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo7_run_0122_14-08-15_13h27m29s.2.graw",7);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo7_run_0122_14-08-15_13h27m29s.3.graw",7);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo7_run_0122_14-08-15_13h27m29s.4.graw",7);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo7_run_0122_14-08-15_13h27m29s.5.graw",7);

fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo8_run_0122_14-08-15_13h27m29s.graw",8);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo8_run_0122_14-08-15_13h27m29s.1.graw",8);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo8_run_0122_14-08-15_13h27m29s.2.graw",8);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo8_run_0122_14-08-15_13h27m29s.3.graw",8);

fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo9_run_0122_14-08-15_13h27m29s.graw",9);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo9_run_0122_14-08-15_13h27m29s.1.graw",9);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo9_run_0122_14-08-15_13h27m29s.2.graw",9);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo9_run_0122_14-08-15_13h27m29s.3.graw",9);
fDecoderTask -> AddData("/home/ayyadlim/Desktop/ATTPC/run_0122/CoBo9_run_0122_14-08-15_13h27m29s.4.graw",9);*/
