void run_eve_proto_lite(TString InputDataFile = "output_proto.root",
                        TString OutputDataFile = "output_proto.reco_display.root",
                        TString unpackDir = "/Unpack_HDF5/eX17/")
{

   TString dir = getenv("VMCWORKDIR");
   TString protomapfile = "proto20181201.map";
   TString scriptfile = "LookupProtoX17.xml";
   TString protomapdir = dir + "/scripts/" + protomapfile;
   TString scriptdir = dir + "/scripts/" + scriptfile;
   TString geoFile = "ATTPC_Proto_v1.0_geomanager.root";

   TString InputDataPath = dir + "/macro/" + unpackDir + InputDataFile;
   TString OutputDataPath = dir + "/macro/" + unpackDir + OutputDataFile;
   TString GeoDataPath = dir + "/geometry/" + geoFile;

   TString geomDir = dir + "/geometry/";
   TString geo = "proto20181201_geo_hires.root";
   gSystem->Setenv("GEOMPATH", geomDir.Data());

   FairLogger *fLogger = FairLogger::GetLogger();
   fLogger->SetLogToScreen(kTRUE);
   fLogger->SetLogVerbosityLevel("MEDIUM");

   FairRunAna *fRun = new FairRunAna();
   FairRootFileSink *sink = new FairRootFileSink(OutputDataFile);
   FairFileSource *source = new FairFileSource(InputDataFile);
   fRun->SetSource(source);
   fRun->SetSink(sink);
   fRun->SetGeomFile(GeoDataPath);

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParRootFileIo *parIo1 = new FairParRootFileIo();
   // parIo1->open("param.dummy_proto.root");
   rtdb->setFirstInput(parIo1);

   auto fMap = std::make_shared<AtTpcProtoMap>();
   fMap->ParseXMLMap(scriptdir.Data());
   fMap->SetGeoFile(geo.Data());
   fMap->SetProtoMap(protomapdir.Data());
   fMap->GeneratePadPlane();

   AtEventManager *eveMan = new AtEventManager();
   AtEventDrawTask *eve = new AtEventDrawTask();
   eve->SetMap(fMap);
   eve->Set3DHitStyleBox();
   eve->SetMultiHit(100);
   eve->SetRawEventBranch("AtRawEventFiltered");
   /*
     AtEventDrawTaskProto *eve = new AtEventDrawTaskProto();
     eve->Set3DHitStyleBox();
     eve->SetProtoMap(protomapdir.Data());
   */

   eveMan->AddTask(eve);
   eveMan->Init();
}
