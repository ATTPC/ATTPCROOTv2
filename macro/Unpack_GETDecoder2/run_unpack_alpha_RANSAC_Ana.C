#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

bool check_file(const std::string& name);

void run_unpack_alpha_RANSAC_Ana
(TString dataFile = "runfiles/NSCL/alphas/alpha_run_0100.txt",TString parameterFile = "ATTPC.alpha.par",
TString mappath="/data/ar46/run_0085/")
{

  if(!check_file(dataFile.Data())){
    std::cout<<cRED<<" Run file "<<dataFile.Data()<<" not found! Terminating..."<<cNORMAL<<std::endl;
    exit(0);
  }

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
  fDecoderTask -> SetUseSeparatedData(fUseSeparatedData);
  if(fUseSeparatedData) fDecoderTask -> SetPseudoTopologyFrame(kTRUE);//! This calls the method 10 times so for less than 10 CoBos ATCore2 must be modified
  //fDecoderTask -> SetPositivePolarity(kTRUE);
  fDecoderTask -> SetPersistence(kFALSE);
  fDecoderTask -> SetMap(scriptdir.Data());
  //fDecoderTask -> SetInhibitMaps(inimap,lowgmap,xtalkmap); // TODO: Only implemented for fUseSeparatedData!!!!!!!!!!!!!!!!!!!1
  fDecoderTask -> SetMapOpt(0); // ATTPC : 0  - Prototype: 1 |||| Default value = 0
  fDecoderTask -> SetNumCobo(9);
  fDecoderTask -> SetEventID(0);


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



  if (!fUseSeparatedData)
    fDecoderTask -> AddData(dataFile);
  else {
    std::ifstream listFile(dataFile.Data());

    Int_t numLin = std::count(std::istreambuf_iterator<char>(listFile),
               std::istreambuf_iterator<char>(), '\n');

    std::cout<<cRED<<" Number of GRAW files found : "<<numLin<<cNORMAL<<std::endl;

    listFile.clear();
    listFile.seekg(0, ios::beg);

    TString dataFileWithPath;
    Int_t iCobo = 0;
    Int_t nCobo = 0;

    TString cobo_str = "CoBo0";

      for(Int_t fi=0;fi<numLin;fi++)
      {
            dataFileWithPath.ReadLine(listFile);

              if(dataFileWithPath.Contains(cobo_str))
              {
                  fDecoderTask->AddData(dataFileWithPath, nCobo);
                  //std::cout<<cYELLOW<<dataFileWithPath<<cNORMAL<<std::endl;
                  //std::cout<<cYELLOW<<cobo_str<<cNORMAL<<std::endl;
                  //std::cout<<cRED<<nCobo<<cNORMAL<<std::endl;
              }else{
                  iCobo++;
                  cobo_str = Form("CoBo%i",iCobo);
                  while(!dataFileWithPath.Contains(cobo_str)){
                    iCobo++;
                    cobo_str = Form("CoBo%i",iCobo);
                  }
                      nCobo++;
                      //std::cout<<cYELLOW<<dataFileWithPath<<cNORMAL<<std::endl;
                      //std::cout<<cYELLOW<<cobo_str<<cNORMAL<<std::endl;
                      //std::cout<<cRED<<nCobo<<cNORMAL<<std::endl;
                      fDecoderTask->AddData(dataFileWithPath, nCobo);
              }
      }


  }


  /*fDecoderTask -> AddData("/data/ND/2013/buffer/NSCL_Alpha/run_0100/CoBo0_run_0100_11Dec14_22h03m15s.graw",0);
  fDecoderTask -> AddData("/data/ND/2013/buffer/NSCL_Alpha/run_0100/CoBo1_run_0100_11Dec14_22h03m15s.graw",1);
  fDecoderTask -> AddData("/data/ND/2013/buffer/NSCL_Alpha/run_0100/CoBo2_run_0100_11Dec14_22h03m15s.graw",2);
  fDecoderTask -> AddData("/data/ND/2013/buffer/NSCL_Alpha/run_0100/CoBo2_run_0100_11Dec14_22h03m15s.1.graw",2);
  fDecoderTask -> AddData("/data/ND/2013/buffer/NSCL_Alpha/run_0100/CoBo3_run_0100_11Dec14_22h03m16s.graw",3);
  fDecoderTask -> AddData("/data/ND/2013/buffer/NSCL_Alpha/run_0100/CoBo4_run_0100_11Dec14_22h03m16s.graw",4);
  fDecoderTask -> AddData("/data/ND/2013/buffer/NSCL_Alpha/run_0100/CoBo6_run_0100_11Dec14_22h03m16s.graw",5);
  fDecoderTask -> AddData("/data/ND/2013/buffer/NSCL_Alpha/run_0100/CoBo7_run_0100_11Dec14_22h03m16s.graw",6);
  fDecoderTask -> AddData("/data/ND/2013/buffer/NSCL_Alpha/run_0100/CoBo7_run_0100_11Dec14_22h03m16s.1.graw",6);
  fDecoderTask -> AddData("/data/ND/2013/buffer/NSCL_Alpha/run_0100/CoBo8_run_0100_11Dec14_22h03m16s.graw",7);
  fDecoderTask -> AddData("/data/ND/2013/buffer/NSCL_Alpha/run_0100/CoBo9_run_0100_11Dec14_22h03m16s.graw",8);*/

  run -> AddTask(fDecoderTask);

  ATPSATask *psaTask = new ATPSATask();
  psaTask -> SetPersistence(kTRUE);
  psaTask -> SetThreshold(20);
  psaTask -> SetPSAMode(1); //NB: 1 is ATTPC - 2 is pATTPC - 3 Filter for ATTPC
	//psaTask -> SetPeakFinder(); //NB: Use either peak finder of maximum finder but not both at the same time
	psaTask -> SetMaxFinder();
  psaTask -> SetBaseCorrection(kTRUE); //Directly apply the base line correction to the pulse amplitude to correct for the mesh induction. If false the correction is just saved
  psaTask -> SetTimeCorrection(kFALSE); //Interpolation around the maximum of the signal peak
  run -> AddTask(psaTask);

  ATRansacTask *RansacTask = new ATRansacTask();
	RansacTask->SetPersistence(kTRUE);
  RansacTask->SetDistanceThreshold(6.0);
	run -> AddTask(RansacTask);

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

  run -> Init();


  //run->Run(0,10);
  run -> RunOnTBData();


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

  gApplication->Terminate();

}

bool check_file(const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
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
