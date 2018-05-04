void run_unpack_alphasource
(TString runNumber = "0664",TString parameterFile = "ATTPC.e17504.par",
TString mappath="/Users/csantama/ATTPCROOT/resources/")
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
  TString dataFile = dir + "/macro/Unpack_GETDecoder2/runfiles/NSCL/tests/run_" + runNumber + ".txt";
  TString scriptfile = "e17504_fission.xml";
  TString scriptdir = dir + "/scripts/"+ scriptfile;
  TString geomDir = dir + "/geometry/";
  TString outputDir = dir + "/macro/Unpack_GETDecoder2/rootfiles/";
  gSystem -> Setenv("GEOMPATH", geomDir.Data());

  TString outputFile  = outputDir + "run_" + runNumber + "_output.root";
  //TString mcParFile   = outputDir + name + ".params.root";
  TString loggerFile  = outputDir + "ATTPCLog_run" + runNumber +".log";
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
    int iAsad = 0;
    int iAsad_Last = 0;
    Int_t iThread = 0;
    string pch;
    string pchf;
    while (dataFileWithPath.ReadLine(listFile)) {
	pchf = dataFileWithPath;
        pch = pchf.substr(pchf.find("AsAd") + 4);
	char Ctry = pch[0];
	iAsad = (int)Ctry - '0';
	if(abs(iAsad - iAsad_Last) > 0.1) iThread++;
	//std::cout << "File = " << dataFileWithPath << std::endl;
	//std::cout << "AsAd = " << iAsad << " ; previous AsAd = " << iAsad_Last << " ; iThread = " << iThread << std::endl;
        fDecoderTask -> AddData(dataFileWithPath, iThread);
	iAsad_Last = iAsad;
    }
  }

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

    /*fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm0/run_0093/CoBo_AsAd0_2017-10-08T02:11:06.485_0000.graw",0);

    fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm0/run_0093/CoBo_AsAd1_2017-10-08T02:11:06.487_0000.graw",1);

    fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm0/run_0093/CoBo_AsAd2_2017-10-08T02:11:06.487_0000.graw",2);

    fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm1/run_0093/CoBo_AsAd0_2017-10-08T02:14:01.761_0005.graw",3);
    fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm1/run_0093/CoBo_AsAd0_2017-10-08T02:14:01.761_0006.graw",3);

    fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm1/run_0093/CoBo_AsAd1_2017-10-08T02:14:01.760_0000.graw",4);

    fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm1/run_0093/CoBo_AsAd2_2017-10-08T02:14:01.761_0009.graw",5);
    fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm1/run_0093/CoBo_AsAd2_2017-10-08T02:14:01.761_0010.graw",5);

    fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm1/run_0093/CoBo_AsAd3_2017-10-08T02:14:01.761_0004.graw",6);
    fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm1/run_0093/CoBo_AsAd3_2017-10-08T02:14:01.761_0005.graw",6);

    fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm2/run_0093/CoBo_AsAd0_2017-10-08T02:14:36.346_0011.graw",7);
    fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm2/run_0093/CoBo_AsAd0_2017-10-08T02:14:36.346_0012.graw",7);
    fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm2/run_0093/CoBo_AsAd0_2017-10-08T02:14:36.346_0013.graw",7);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm2/run_0093/CoBo_AsAd1_2017-10-08T02:14:36.346_0000.graw",8);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm2/run_0093/CoBo_AsAd2_2017-10-08T02:14:36.346_0000.graw",9);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm2/run_0093/CoBo_AsAd3_2017-10-08T02:14:36.346_0007.graw",10);
     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm2/run_0093/CoBo_AsAd3_2017-10-08T02:14:36.346_0008.graw",10);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm3/run_0093/CoBo_AsAd0_2017-10-08T02:13:22.056_0000.graw",11);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm3/run_0093/CoBo_AsAd1_2017-10-08T02:13:22.058_0000.graw",12);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm3/run_0093/CoBo_AsAd2_2017-10-08T02:13:22.058_0000.graw",13);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm3/run_0093/CoBo_AsAd3_2017-10-08T02:13:22.058_0000.graw",14);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm4/run_0093/CoBo_AsAd0_2017-10-08T02:18:35.596_0000.graw",15);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm4/run_0093/CoBo_AsAd1_2017-10-08T02:18:35.598_0001.graw",16);
     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm4/run_0093/CoBo_AsAd1_2017-10-08T02:18:35.598_0002.graw",16);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm4/run_0093/CoBo_AsAd2_2017-10-08T02:18:35.598_0000.graw",17);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm4/run_0093/CoBo_AsAd3_2017-10-08T02:18:35.598_0000.graw",18);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm5/run_0093/CoBo_AsAd0_2017-10-08T02:10:53.729_0000.graw",19);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm5/run_0093/CoBo_AsAd1_2017-10-08T02:10:53.730_0000.graw",20);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm5/run_0093/CoBo_AsAd2_2017-10-08T02:10:53.731_0000.graw",21);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm5/run_0093/CoBo_AsAd3_2017-10-08T02:10:53.731_0000.graw",22);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm6/run_0093/CoBo_AsAd0_2017-10-08T02:15:00.148_0000.graw",23);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm6/run_0093/CoBo_AsAd1_2017-10-08T02:15:00.149_0000.graw",24);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm6/run_0093/CoBo_AsAd2_2017-10-08T02:15:00.149_0000.graw",25);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm6/run_0093/CoBo_AsAd3_2017-10-08T02:15:00.217_0005.graw",26);
     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm6/run_0093/CoBo_AsAd3_2017-10-08T02:15:00.217_0006.graw",26);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm7/run_0093/CoBo_AsAd0_2017-10-08T02:14:20.689_0000.graw",27);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm7/run_0093/CoBo_AsAd1_2017-10-08T02:14:20.692_0002.graw",28);
     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm7/run_0093/CoBo_AsAd1_2017-10-08T02:14:20.692_0003.graw",28);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm7/run_0093/CoBo_AsAd2_2017-10-08T02:14:20.691_0000.graw",29);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm7/run_0093/CoBo_AsAd3_2017-10-08T02:14:20.692_0002.graw",30);
     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm7/run_0093/CoBo_AsAd3_2017-10-08T02:14:20.692_0003.graw",30);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm8/run_0093/CoBo_AsAd0_2017-10-08T02:14:06.994_0000.graw",31);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm8/run_0093/CoBo_AsAd1_2017-10-08T02:14:06.995_0000.graw",32);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm8/run_0093/CoBo_AsAd2_2017-10-08T02:14:07.003_0005.graw",33);
     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm8/run_0093/CoBo_AsAd2_2017-10-08T02:14:07.003_0006.graw",33);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm9/run_0093/CoBo_AsAd0_2017-10-08T02:13:33.492_0000.graw",34);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm9/run_0093/CoBo_AsAd1_2017-10-08T02:13:33.494_0000.graw",35);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm9/run_0093/CoBo_AsAd2_2017-10-08T02:13:33.565_0004.graw",36);
     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm9/run_0093/CoBo_AsAd2_2017-10-08T02:13:33.565_0005.graw",36);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm10/run_0093/CoBo10_AsAd0_2017-10-08T02:12:29.626_0007.graw",37);
     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm10/run_0093/CoBo10_AsAd0_2017-10-08T02:12:29.626_0008.graw",37);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm10/run_0093/CoBo10_AsAd1_2017-10-08T02:12:29.626_0000.graw",38);

     fDecoderTask->AddData("/mnt/analysis/attpc/e17504/mm10/run_0093/CoBo10_AsAd2_2017-10-08T02:12:29.626_0000.graw",39);
  */

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

    ATRansacTask *RansacTask = new ATRansacTask();
    RansacTask->SetPersistence(kTRUE);
    RansacTask->SetDistanceThreshold(6.0);
    RansacTask->SetFullMode();
    run -> AddTask(RansacTask);

    /*
    ATAnalysisTask *AnaTask = new ATAnalysisTask();
    AnaTask->SetFullScale(); //NB: Mandatory for full scale detector
    AnaTask->SetPersistence(kTRUE);

    // Setting Monte Carlo Energy Loss parameters
    std::vector<Double_t> par[10];  //de/dx - E
    par[0]={8.56,0.83,2.5,1.6,1.5,0.15,-1.0,-0.2,-0.17,-8.0,-0.4};
    std::vector<Double_t> parRtoE[10]; // E - R
    parRtoE[0] ={0.63,-1.66,-1.0,0.5,-19.0,-10.0,+40.0};
    std::vector<std::pair<Int_t,Int_t>> particle;
    particle.push_back(std::make_pair(4,2));

    AnaTask->SetELossPar(par);
    AnaTask->SetEtoRParameters(parRtoE);
    AnaTask->AddParticle(particle);
    AnaTask ->SetEnableMap(); //Enables an instance of the ATTPC map:  This enables the MC with Q instead of position
    AnaTask ->SetMap(scriptdir.Data());

    run -> AddTask(AnaTask);
*/

  run -> Init();

  //run -> RunOnTBData();
  run->Run(0,50);

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
