void run_unpack_new
(TString dataFile = "runfiles/ar46_run_0085.txt",TString parameterFile = "ATTPC.fissionC02.par",
TString mappath="../../resources/")
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
  TString outputFile  = "output.root";
  //TString mcParFile   = dataDir + name + ".params.root";
  TString loggerFile  = dataDir + "ATTPCLog.log";
  TString digiParFile = dir + "/parameters/" + parameterFile;
  TString geoManFile  = dir + "/geometry/ATTPC_v1.2.root";

  //TString inimap   = mappath + "inhib.txt";
  //TString lowgmap  = mappath + "lowgain.txt";
  //TString xtalkmap = mappath + "beampads_e15503b.txt";

  TString inimap = mappath + "he4_in_pads.txt";

  // -----------------------------------------------------------------
  // Logger
  FairLogger *fLogger = FairLogger::GetLogger();
  fLogger -> SetLogFileName(loggerFile);
  fLogger -> SetLogToScreen(kTRUE);
  fLogger -> SetLogToFile(kTRUE);
  fLogger -> SetLogVerbosityLevel("LOW");

  FairRunAna* run = new FairRunAna();
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

  // Settings
  Bool_t fUseDecoder = kTRUE;
  if (dataFile.IsNull() == kTRUE)
    fUseDecoder = kFALSE;

  Bool_t fUseSeparatedData = kFALSE;
    if (dataFile.EndsWith(".txt"))
      fUseSeparatedData = kTRUE;

 /*
  *     Unpacking options:
  *         - SetUseSeparatedData:      To be used with 10 CoBo files without merging. Mainly for the ATTPC. Enabled if the input file is a txt.
  *         - SetPseudoTopologyFrame:   Used to force the graw file to have a Topology frame.
  *         - SetPersistance:           Save the unpacked data into the root file.
  *         - SetMap:                   Chose the lookup table.
  *         - SetMapOpt                 Chose the pad plane geometry. In addition forces the unpacker to use Basic Frames for 1 single file (p-ATTPC case) of Layered
  *                                     Frames for Merged Data (10 Cobos merged data).
  */
  ATDecoder2Task *fDecoderTask = new ATDecoder2Task();
  fDecoderTask -> SetPTFMask(0x1); //Number of active Asad. For the new NSCL DAQ
  fDecoderTask -> SetUseSeparatedData(fUseSeparatedData);
  if(fUseSeparatedData) fDecoderTask -> SetPseudoTopologyFrame(kTRUE);//! This calls the method 10 times so for less than 10 CoBos ATCore2 must be modified
  //fDecoderTask -> SetPositivePolarity(kTRUE);
  fDecoderTask -> SetPersistence(kFALSE);
  fDecoderTask -> SetMap(scriptdir.Data());
  fDecoderTask -> SetInhibitMaps(inimap,"0","0"); // TODO: Only implemented for fUseSeparatedData!!!!!!!!!!!!!!!!!!!1
  fDecoderTask -> SetNumCobo(40);
  fDecoderTask -> SetMapOpt(0); // ATTPC : 0  - Prototype: 1 |||| Default value = 0


  /*if (!fUseSeparatedData)
    fDecoderTask -> AddData(dataFile);
  else {
    std::ifstream listFile(dataFile.Data());
    TString dataFileWithPath;
    Int_t iCobo = 0;
    while (dataFileWithPath.ReadLine(listFile)) {
        if (dataFileWithPath.Contains(Form("CoBo%i",iCobo)) )
              fDecoderTask -> AddData(dataFileWithPath, iCobo);
        else{
          iCobo++;
          fDecoderTask -> AddData(dataFileWithPath, iCobo);
        }
    }
  }*/

   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm0/run_0004/CoBo_AsAd0_2016-10-10T11_44_55.841_0000.graw",0);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm0/run_0004/CoBo_AsAd1_2016-10-10T11_44_55.843_0000.graw",1);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm0/run_0004/CoBo_AsAd2_2016-10-10T11_44_55.912_0000.graw",2);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm0/run_0004/CoBo_AsAd3_2016-10-10T11_44_55.918_0000.graw",3);

   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm1/run_0004/CoBo_AsAd0_2016-10-10T11_48_14.171_0000.graw",4);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm1/run_0004/CoBo_AsAd1_2016-10-10T11_48_14.174_0000.graw",5);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm1/run_0004/CoBo_AsAd2_2016-10-10T11_48_14.175_0000.graw",6);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm1/run_0004/CoBo_AsAd3_2016-10-10T11_48_14.184_0000.graw",7);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm1/run_0004/CoBo_AsAd3_2016-10-10T11_48_14.184_0001.graw",7);

   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm2/run_0004/CoBo_AsAd0_2016-10-10T11_49_47.987_0000.graw",8);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm2/run_0004/CoBo_AsAd1_2016-10-10T11_49_47.990_0000.graw",9);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm2/run_0004/CoBo_AsAd2_2016-10-10T11_49_48.066_0000.graw",10);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm2/run_0004/CoBo_AsAd3_2016-10-10T11_49_48.075_0000.graw",11);

   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm3/run_0004/CoBo_AsAd0_2016-10-10T11_48_46.111_0000.graw",12);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm3/run_0004/CoBo_AsAd1_2016-10-10T11_48_46.182_0000.graw",13);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm3/run_0004/CoBo_AsAd2_2016-10-10T11_48_46.184_0000.graw",14);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm3/run_0004/CoBo_AsAd3_2016-10-10T11_48_46.185_0000.graw",15);

   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm4/run_0004/CoBo_AsAd0_2016-10-10T11_49_46.450_0000.graw",16);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm4/run_0004/CoBo_AsAd1_2016-10-10T11_49_46.453_0000.graw",17);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm4/run_0004/CoBo_AsAd2_2016-10-10T11_49_46.454_0000.graw",18);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm4/run_0004/CoBo_AsAd3_2016-10-10T11_49_46.454_0000.graw",19);

   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm5/run_0004/CoBo_AsAd0_2016-10-10T11_44_55.837_0000.graw",20);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm5/run_0004/CoBo_AsAd1_2016-10-10T11_44_55.908_0000.graw",21);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm5/run_0004/CoBo_AsAd2_2016-10-10T11_44_55.909_0000.graw",22);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm5/run_0004/CoBo_AsAd3_2016-10-10T11_44_55.911_0000.graw",23);

   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm6/run_0004/CoBo_AsAd0_2016-10-10T11_44_55.856_0000.graw",24);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm6/run_0004/CoBo_AsAd1_2016-10-10T11_44_55.927_0000.graw",25);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm6/run_0004/CoBo_AsAd2_2016-10-10T11_44_55.928_0000.graw",26);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm6/run_0004/CoBo_AsAd3_2016-10-10T11_44_55.933_0000.graw",27);

   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm7/run_0004/CoBo_AsAd0_2016-10-10T11_44_55.855_0000.graw",28);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm7/run_0004/CoBo_AsAd1_2016-10-10T11_44_55.857_0000.graw",29);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm7/run_0004/CoBo_AsAd2_2016-10-10T11_44_55.932_0000.graw",30);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm7/run_0004/CoBo_AsAd3_2016-10-10T11_44_55.934_0000.graw",31);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm7/run_0004/CoBo_AsAd2_2016-10-10T11_44_55.934_0001.graw",31);

   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm8/run_0004/CoBo_AsAd0_2016-10-10T11_49_53.572_0000.graw",32);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm8/run_0004/CoBo_AsAd1_2016-10-10T11_49_53.574_0000.graw",33);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm8/run_0004/CoBo_AsAd2_2016-10-10T11_49_53.575_0000.graw",34);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm8/run_0004/CoBo_AsAd3_2016-10-10T11_49_53.643_0000.graw",35);

   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm9/run_0004/CoBo_AsAd0_2016-10-10T11_44_55.820_0000.graw",36);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm9/run_0004/CoBo_AsAd1_2016-10-10T11_44_55.892_0000.graw",37);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm9/run_0004/CoBo_AsAd2_2016-10-10T11_44_55.893_0000.graw",38);
   fDecoderTask -> AddData("/data/NSCL/fission/co2/mm9/run_0004/CoBo_AsAd3_2016-10-10T11_44_55.902_0000.graw",39);


  run -> AddTask(fDecoderTask);

  ATPSATask *psaTask = new ATPSATask();
  psaTask -> SetPersistence(kTRUE);
  psaTask -> SetThreshold(50);
  psaTask -> SetPSAMode(1); //NB: 1 is ATTPC - 2 is pATTPC
	//psaTask -> SetPeakFinder(); //NB: Use either peak finder of maximum finder but not both at the same time
	psaTask -> SetMaxFinder();
  psaTask -> SetBaseCorrection(kTRUE); //Directly apply the base line correction to the pulse amplitude to correct for the mesh induction. If false the correction is just saved
  psaTask -> SetTimeCorrection(kTRUE); //Interpolation around the maximum of the signal peak. Only affect Z calibration at PSA stage
  run -> AddTask(psaTask);

  ATRansacTask *RansacTask = new ATRansacTask();
  RansacTask->SetPersistence(kTRUE);
  RansacTask->SetDistanceThreshold(10.0);
  RansacTask->SetFullMode(); //Without conditions on line determination
  run -> AddTask(RansacTask);



  ATAnalysisTask *AnaTask = new ATAnalysisTask();
  AnaTask->SetFullScale(); //NB: Mandatory for full scale detector
  AnaTask->SetPersistence(kTRUE);

  std::vector<Double_t> par[10];  //de/dx - E
  par[0]={8.56,0.83,2.5,1.6,1.5,0.15,-1.0,-0.2,-0.17,-8.0,-0.4};
  std::vector<Double_t> parRtoE[10]; // E - R
  parRtoE[0] ={1.23,-1.3,0.0195,0.2,0.1};
  std::vector<std::pair<Int_t,Int_t>> particle;
  particle.push_back(std::make_pair(4,2));

  AnaTask->SetELossPar(par);
  AnaTask->SetEtoRParameters(parRtoE);
  AnaTask->AddParticle(particle);
  AnaTask->SetEnableMap(); //Enables an instance of the ATTPC map:  This enables the MC with Q instead of position
  AnaTask->SetMap(scriptdir.Data());
  AnaTask->SetSimpleMode();

  run -> AddTask(AnaTask);

  run -> Init();

  //run -> RunOnTBData();
  run->Run(0,1000);

  std::cout << std::endl << std::endl;
  std::cout << "Macro finished succesfully."  << std::endl << std::endl;
  std::cout << "- Output file : " << outputFile << std::endl << std::endl;
  // -----   Finish   -------------------------------------------------------
  timer.Stop();
  Double_t rtime = timer.RealTime();
  Double_t ctime = timer.CpuTime();
  cout << endl << endl;
  cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
  cout << endl;
  // ------------------------------------------------------------------------

  //gApplication->Terminate();

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

/// New DAQ Version

/*fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm0/run_0000/CoBo_AsAd0_2016-10-07T16_44_34.319_0000.graw",0);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm0/run_0000/CoBo_AsAd1_2016-10-07T16_44_34.322_0000.graw",1);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm0/run_0000/CoBo_AsAd2_2016-10-07T16_44_34.324_0000.graw",2);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm0/run_0000/CoBo_AsAd3_2016-10-07T16_44_34.328_0000.graw",3);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm1/run_0000/CoBo_AsAd0_2016-10-07T16_47_47.204_0000.graw",4);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm1/run_0000/CoBo_AsAd1_2016-10-07T16_47_47.206_0000.graw",5);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm1/run_0000/CoBo_AsAd2_2016-10-07T16_47_47.209_0000.graw",6);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm1/run_0000/CoBo_AsAd3_2016-10-07T16_47_47.286_0000.graw",7);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm2/run_0000/CoBo_AsAd0_2016-10-07T16_49_19.013_0000.graw",8);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm2/run_0000/CoBo_AsAd1_2016-10-07T16_49_19.082_0000.graw",9);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm2/run_0000/CoBo_AsAd2_2016-10-07T16_49_19.088_0000.graw",10);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm2/run_0000/CoBo_AsAd3_2016-10-07T16_49_19.248_0000.graw",11);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm3/run_0000/CoBo_AsAd0_2016-10-07T16_48_18.379_0000.graw",12);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm3/run_0000/CoBo_AsAd1_2016-10-07T16_48_18.449_0000.graw",13);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm3/run_0000/CoBo_AsAd2_2016-10-07T16_48_18.520_0000.graw",14);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm3/run_0000/CoBo_AsAd3_2016-10-07T16_48_18.598_0000.graw",15);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm4/run_0000/CoBo_AsAd0_2016-10-07T16_49_16.650_0000.graw",16);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm4/run_0000/CoBo_AsAd1_2016-10-07T16_49_16.653_0000.graw",17);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm4/run_0000/CoBo_AsAd2_2016-10-07T16_49_16.660_0000.graw",18);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm4/run_0000/CoBo_AsAd3_2016-10-07T16_49_16.666_0000.graw",19);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm5/run_0000/CoBo_AsAd0_2016-10-07T16_44_34.178_0000.graw",20);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm5/run_0000/CoBo_AsAd1_2016-10-07T16_44_34.248_0000.graw",21);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm5/run_0000/CoBo_AsAd2_2016-10-07T16_44_34.250_0000.graw",22);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm5/run_0000/CoBo_AsAd3_2016-10-07T16_44_34.329_0000.graw",23);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm6/run_0000/CoBo_AsAd0_2016-10-07T16_44_34.269_0000.graw",24);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm6/run_0000/CoBo_AsAd1_2016-10-07T16_44_34.341_0000.graw",25);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm6/run_0000/CoBo_AsAd2_2016-10-07T16_44_34.342_0000.graw",26);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm6/run_0000/CoBo_AsAd3_2016-10-07T16_44_34.348_0000.graw",27);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm7/run_0000/CoBo_AsAd0_2016-10-07T16_44_34.286_0000.graw",28);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm7/run_0000/CoBo_AsAd1_2016-10-07T16_44_34.289_0000.graw",29);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm7/run_0000/CoBo_AsAd2_2016-10-07T16_44_34.295_0000.graw",30);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm7/run_0000/CoBo_AsAd3_2016-10-07T16_44_34.296_0000.graw",31);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm8/run_0000/CoBo_AsAd0_2016-10-07T16_49_24.139_0000.graw",32);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm8/run_0000/CoBo_AsAd1_2016-10-07T16_49_24.142_0000.graw",33);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm8/run_0000/CoBo_AsAd2_2016-10-07T16_49_24.146_0000.graw",34);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm8/run_0000/CoBo_AsAd3_2016-10-07T16_49_24.149_0000.graw",35);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm9/run_0000/CoBo_AsAd0_2016-10-07T16_44_34.135_0000.graw",36);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm9/run_0000/CoBo_AsAd1_2016-10-07T16_44_34.276_0000.graw",37);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm9/run_0000/CoBo_AsAd2_2016-10-07T16_44_34.277_0000.graw",38);
fDecoderTask -> AddData("/data/NSCL/fission/co2/fission_CO2/mm9/run_0000/CoBo_AsAd3_2016-10-07T16_44_34.431_0000.graw",39);*/
