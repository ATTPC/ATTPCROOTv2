void run_eve_proto_lite(TString  InputDataFile = "run_0080.root",TString  OutputDataFile = "output_proto.reco_display.root", TString unpackDir="/Unpack_HDF5/S1845/")
{

   TString dir = getenv("VMCWORKDIR");
   TString protomapfile = "proto20181201.map";
   TString protomapdir = dir + "/scripts/"+ protomapfile;
   TString geoFile = "ATTPC_Proto_v1.0_geomanager.root";

   TString InputDataPath = dir + "/macro/"+ unpackDir + InputDataFile;
   TString OutputDataPath = dir + "/macro/"+ unpackDir + OutputDataFile;
   TString GeoDataPath = dir + "/geometry/" + geoFile;


  FairLogger *fLogger = FairLogger::GetLogger();
  fLogger -> SetLogToScreen(kTRUE);
  fLogger->SetLogVerbosityLevel("MEDIUM");

  FairRunAna *fRun= new FairRunAna();
  fRun -> SetInputFile(InputDataPath);
  fRun -> SetOutputFile(OutputDataPath);
  fRun -> SetGeomFile(GeoDataPath);

  FairRuntimeDb* rtdb = fRun->GetRuntimeDb();
  FairParRootFileIo* parIo1 = new FairParRootFileIo();
  //parIo1->open("param.dummy_proto.root");
  rtdb->setFirstInput(parIo1);

  FairRootManager* ioman = FairRootManager::Instance();

  ATEventManagerProto *eveMan = new ATEventManagerProto();
  ATEventDrawTaskProto* eve = new ATEventDrawTaskProto();
  eve->Set3DHitStyleBox();
  eve->SetProtoMap(protomapdir.Data());


  eveMan->AddTask(eve);
  eveMan->Init();
}
