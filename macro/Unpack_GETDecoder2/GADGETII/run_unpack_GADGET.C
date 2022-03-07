void run_unpack_GADGET(TString dataFile ="GADGET-alpha-source-data1.txt", TString parameterFile = "GADGET.sim.par" , TString mappath="") 
//void run_unpack_GADGET(TString dataFile ="",TString parameterFile = "GADGET.sim.par", TString mappath="") 

{

  // -----   Timer   --------------------------------------------------------
 TStopwatch timer;
 timer.Start();
 // ------------------------------------------------------------------------

  gSystem->Load("libXMLParser.so");
  // -----------------------------------------------------------------
  // Set file names  
  TString scriptfile = "LookupGADGET08232021.xml";
  TString dir = getenv("VMCWORKDIR");
  TString scriptdir = dir + "/scripts/"+ scriptfile;
  TString dataDir = dir + "/macro/Unpack_GETDecoder2/";
  TString geomDir = dir + "/geometry/";
  gSystem -> Setenv("GEOMPATH", geomDir.Data());
  //TString loggerFile  = dataDir + "ATTPCLog.log";

  TString outputFile  = "output.root";
  TString digiParFile = dir + "/parameters/" + parameterFile;
  TString geoManFile  = dir + "/geometry/GADGET_II.root";
 

  TString inimap   = mappath + "inhib.txt";
  TString lowgmap  = mappath + "lowgain.txt";
  TString xtalkmap = mappath + "beampads_e15503b.txt";

  // -----------------------------------------------------------------
  // Logger
  FairLogger *fLogger = FairLogger::GetLogger();
  /*fLogger -> SetLogFileName(loggerFile);
  fLogger -> SetLogToScreen(kTRUE);
  fLogger -> SetLogToFile(kTRUE);
  fLogger -> SetLogVerbosityLevel("MEDIUM");*/

  FairRunAna* run = new FairRunAna();
  run -> SetOutputFile(outputFile); 
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
    
  AtDecoder2Task *fDecoderTask = new AtDecoder2Task();
  fDecoderTask -> SetNumCobo(4);
 // fDecoderTask -> SetPTFMask(0x1);
  fDecoderTask -> SetUseSeparatedData(fUseSeparatedData);
  if(fUseSeparatedData) fDecoderTask -> SetPseudoTopologyFrame(kFALSE);
  //fDecoderTask -> SetPositivePolarity(kTRUE);
  fDecoderTask -> SetPersistence(kTRUE);
  fDecoderTask -> SetMap(scriptdir.Data());
  //fDecoderTask -> SetInhibitMaps(inimap,lowgmap,xtalkmap); // TODO: Only implemented for fUseSeparatedData!!!!!!!!!!!!!!!!!!!1
  fDecoderTask -> SetMapOpt(0); // ATTPC : 0  - Prototype: 1 |||| Default value = 0  
  //fDecoderTask -> SetEventID(0);
   

  if (!fUseSeparatedData)
    fDecoderTask -> AddData(dataFile);
    else {
    std::ifstream listFile(dataFile.Data());
    TString dataFileWithPath;
    Int_t iAsAd = 0;
    while (dataFileWithPath.ReadLine(listFile)) {
      if (dataFileWithPath.Contains(Form("AsAd%i",iAsAd)) ){
              fDecoderTask -> AddData(dataFileWithPath, iAsAd);
        	std::cout<<" Added file : "<<dataFileWithPath<<" - iAsAd : "<<iAsAd<<"\n";
    }else{
          
          fDecoderTask -> AddData(dataFileWithPath, iAsAd);
          cout<<" Add data file "<<endl;
        }

      iAsAd++;
    }
  }
   
 
   AtPSASimple2 *psa = new AtPSASimple2();

   AtPSAtask *psaTask = new AtPSAtask(psa);
   
   psaTask->SetPersistence(kTRUE);   
   //psaTask->SetPSAMode(1);
   //psaTask->SetMaxFinder();
   //psaTask -> SetBaseCorrection(kTRUE);
   //psaTask -> SetTimeCorrection(kTRUE);
   psa->SetThreshold(5);
   psa->SetMaxFinder();
  

   run -> AddTask(fDecoderTask);
   run -> AddTask(psaTask);

   run -> Init();
   
   //run -> RunOnTBData();
    run->Run(0,100);

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
