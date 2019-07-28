#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

struct auxchannel
{
  std::string name;
  uint8_t cobo;
  uint8_t asad;
  uint8_t aget;
  uint8_t channel;
};


void run_unpack_HC(std::string dataFile = "/home/ayyadlim/Desktop/get/files/run_0168.h5",TString parameterFile = "pATTPC.S1845.par",TString mappath="")
{

  // -----   Timer   --------------------------------------------------------
 TStopwatch timer;
 timer.Start();
 // ------------------------------------------------------------------------

  gSystem->Load("libXMLParser.so");
  // -----------------------------------------------------------------
  // Set file names
  TString scriptfile = "LookupProto20181201v2.xml";
  TString protomapfile = "proto20181201.map";
  TString dir = getenv("VMCWORKDIR");
  TString scriptdir = dir + "/scripts/"+ scriptfile;
  TString dataDir = dir + "/macro/data/";
  TString geomDir = dir + "/geometry/";
  TString protomapdir = dir + "/scripts/"+ protomapfile;
  TString geo = "proto20181201_geo_hires.root";
  gSystem -> Setenv("GEOMPATH", geomDir.Data());

  //TString inputFile   = dataDir + name + ".digi.root";
  //TString outputFile  = dataDir + "output.root";
  TString outputFile  = "output_proto.root";
  //TString mcParFile   = dataDir + name + ".params.root";
  TString loggerFile  = dataDir + "ATTPCLog.log";
  TString digiParFile = dir + "/parameters/" + parameterFile;
  TString geoManFile  = dir + "/geometry/ATTPC_Proto_v1.0.root";


  // -----------------------------------------------------------------
  // Logger
  FairLogger *fLogger = FairLogger::GetLogger();
  /*fLogger -> SetLogFileName(loggerFile);
  fLogger -> SetLogToScreen(kTRUE);
  fLogger -> SetLogToFile(kTRUE);
  fLogger -> SetLogVerbosityLevel("LOW");*/

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

  //Auxiliary channels
  //Hash table: cobo, asad, aget, channel
  std::vector<auxchannel> aux_channels;

  auxchannel ch_1{"mutant",5,0,0,65};
  aux_channels.push_back(ch_1);
  auxchannel ch_2{"mesh",5,0,0,66};
  aux_channels.push_back(ch_2);
  auxchannel ch_3{"protons",5,0,0,67};
  aux_channels.push_back(ch_3);
  auxchannel ch_4{"begin_DAQ",5,0,0,61};
  aux_channels.push_back(ch_4);
  auxchannel ch_5{"unknown",5,0,0,64};
  aux_channels.push_back(ch_5);
  auxchannel ch_6{"downscaled_alpha",5,0,0,59};
  aux_channels.push_back(ch_6);

   //End of auxiliary channel setup 



  ATHDFParserTask* HDFParserTask = new ATHDFParserTask(1);
  HDFParserTask->SetPersistence(kTRUE);
  HDFParserTask->SetATTPCMap(scriptdir.Data());
  HDFParserTask->SetProtoGeoFile(geo.Data());
  HDFParserTask->SetProtoMapFile(protomapdir.Data());
  HDFParserTask->SetFileName(dataFile);

   for(auto iaux : aux_channels)
   {
    auto hash  = HDFParserTask->CalculateHash(iaux.cobo,iaux.asad,iaux.aget,iaux.channel);  
    auto isaux = HDFParserTask->SetAuxChannel(hash,iaux.name);  
   }


  ATPSATask *psaTask = new ATPSATask();
  psaTask -> SetPersistence(kTRUE);
  psaTask -> SetThreshold(20);
  psaTask -> SetPSAMode(2); //NB: 1 is ATTPC - 2 is pATTPC - 3 Filter for ATTPC - 4: Full Time Buckets - 5: Proto Full

  psaTask -> SetTBLimits(std::make_pair<Int_t,Int_t>(160,270)); 
  // Set the limits of integration for the total charge Q (only implemented in PSA modes 2 and 5)
  // For example (160,270) is used for the proton run
  //psaTask -> SetPeakFinder(); //NB: Use either peak finder of maximum finder but not both at the same time
  psaTask -> SetMaxFinder();
  //psaTask -> SetBaseCorrection(kTRUE); //Directly apply the base line correction to the pulse amplitude to correct for the mesh induction. If false the correction is just saved
  //psaTask -> SetTimeCorrection(kFALSE); //Interpolation around the maximum of the signal peak

  ATPRATask *praTask = new ATPRATask();
  praTask -> SetPersistence(kTRUE);
  
  
  
  run -> AddTask(HDFParserTask);
  run -> AddTask(psaTask);
  //run -> AddTask(praTask);

  run -> Init();

  run->Run(0,20000);
  //run->Run(0,309412);
  //run -> RunOnTBData();


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

