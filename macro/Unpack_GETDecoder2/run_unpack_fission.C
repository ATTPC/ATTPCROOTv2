void run_unpack_fission
(TString runNumber = "0100",TString parameterFile = "ATTPC.e17504.par",
TString mappath="/mnt/projects/attpcroot/santamar/ATTPCROOTv2/resources/")
{

  // -----   Timer   --------------------------------------------------------
 TStopwatch timer;
 timer.Start();
 // ------------------------------------------------------------------------

  gSystem->Load("libXMLParser.so");
  // -----------------------------------------------------------------
  // Set file names 
  TString dir = getenv("VMCWORKDIR");
  //TString dataFile = "runfiles/e17504_run_" + runNumber + ".txt";
  TString dataFile = dir + "/macro/perso/runfiles/e17504_run_" + runNumber + ".txt";
  TString scriptfile = "e17504_fission.xml";
  TString scriptdir = dir + "/scripts/"+ scriptfile;
  TString dataDir = dir + "/macro/perso/rootfiles/";
  TString geomDir = dir + "/geometry/";
  TString outputDir = "/mnt/analysis/attpc/e17504/rootfiles/";
  gSystem -> Setenv("GEOMPATH", geomDir.Data());

  TString outputFile  = /*outputDir*/ + "run_" + runNumber + "_output.root"; 
  //TString mcParFile   = dataDir + name + ".params.root";
  TString loggerFile  = dataDir + "ATTPCLog.log";
  TString digiParFile = dir + "/parameters/" + parameterFile;
  TString geoManFile  = dir + "/geometry/ATTPC_v1.1.root";

  TString inimap   = mappath + "inhib_e17504.txt";
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
  *         - SetUseSeparatedData:      To be used with 40 CoBo files without merging. Mainly for the ATTPC. Enabled if the input file is a txt.
  *         - SetPseudoTopologyFrame:   Used to force the graw file to have a Topology frame.
  *         - SetPersistance:           Save the unpacked data into the root file.
  *         - SetMap:                   Chose the lookup table.
  *         - SetMapOpt                 Chose the pad plane geometry. In addition forces the unpacker to use Basic Frames for 1 single file (p-ATTPC case) of Layered
  *                                     Frames for Merged Data (10 Cobos merged data).
  */
  ATDecoder2Task *fDecoderTask = new ATDecoder2Task();
  fDecoderTask -> SetPTFMask(0x1); //Number of active Asad. For the new NSCL DAQ
  fDecoderTask -> SetUseSeparatedData(fUseSeparatedData);
  if(fUseSeparatedData) fDecoderTask -> SetPseudoTopologyFrame(kTRUE);//! This calls the method 40 times so for less than 40 Asads ATCore2 must be modified
  //fDecoderTask -> SetPositivePolarity(kTRUE);
  fDecoderTask -> SetPersistence(kFALSE);
  fDecoderTask -> SetMap(scriptdir.Data());
  fDecoderTask -> SetInhibitMaps(inimap,lowgmap,xtalkmap); // TODO: Only implemented for fUseSeparatedData!!!!!!!!!!!!!!!!!!!1
  fDecoderTask -> SetNumCobo(40);
  fDecoderTask -> SetMapOpt(0); // ATTPC : 0  - Prototype: 1 |||| Default value = 0

//#NOTE: Used if more than 1 .graw file per Asad AND in numerical order, which the current macro "list_filename.pl" does not do!!
  if (!fUseSeparatedData)
    fDecoderTask -> AddData(dataFile);
  else {
    std::ifstream listFile(dataFile.Data());
    TString dataFileWithPath;
    std::cout<<"FileName Check!!->"<<dataFileWithPath<<std::endl;
    Int_t iAsad = 0;
    Int_t iThread = 0;
    Int_t counter = 0;
    while (dataFileWithPath.ReadLine(listFile)) {
	std::cout << counter << " : " << dataFileWithPath << std::endl;
        //std::cout<<"iCobo"<</home/attpc/bigATTPC/HeCO2/Run59_February2017/mm9/iCobo<<std::endl;
        if (dataFileWithPath.Contains(Form("AsAd%i",iAsad))){
              fDecoderTask -> AddData(dataFileWithPath, iThread);
	}
        else if(iAsad == 3 || (iAsad == 2 && (dataFileWithPath.Contains(Form("mm%i",0))
                                            || dataFileWithPath.Contains(Form("mm%i",8))
                                            || dataFileWithPath.Contains(Form("mm%i",9))
                                            || dataFileWithPath.Contains(Form("mm%i",10))))){
          iAsad = 0;
          iThread++;
          fDecoderTask->AddData(dataFileWithPath, iThread);
        }
        else{
          iAsad++;
          iThread++;
          fDecoderTask -> AddData(dataFileWithPath, iThread);
        }
	counter ++;
    }
  }
  std::cout << "youhou" << std::endl;

//#NOTE: Can only be used if only 1 .graw file per Asad!!
  /*if (!fUseSeparatedData){
    fDecoderTask -> AddData(dataFile);
  }
  else {
    std::ifstream listFile(dataFile.Data());
    TString dataFileWithPath;
    std::cout<<"FileName Check!!->"<<dataFileWithPath<<std::endl;
    Int_t iThread = 0;
    while (dataFileWithPath.ReadLine(listFile)) {
      fDecoderTask -> AddData(dataFileWithPath, iThread);
      iThread++;
    }
  }*/
  
  run -> AddTask(fDecoderTask);

  ATPSATask *psaTask = new ATPSATask();
  psaTask -> SetPersistence(kTRUE);
  psaTask -> SetThreshold(100);
  psaTask -> SetPSAMode(1); //NB: 1 is ATTPC - 2 is pATTPC
	//psaTask -> SetPeakFinder(); //NB: Use either peak finder of maximum finder but not both at the same time
	psaTask -> SetMaxFinder();
  psaTask -> SetBaseCorrection(kFALSE); //Directly apply the base line correction to the pulse amplitude to correct for the mesh induction. If false the correction is just saved
  psaTask -> SetTimeCorrection(kTRUE); //Interpolation around the maximum of the signal peak. Only affect Z calibration at PSA stage
  run -> AddTask(psaTask);


  run -> Init();

  run -> RunOnTBData();
  //run->Run(0,5);
  //run->Run(0,1000000);

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
