void eventDisplay()
{
   //-----User Settings:-----------------------------------------------
   TString InputFile = "data/SeGA.root";
   TString ParFile = "data/SeGApar.root";
   TString OutFile = "data/SeGAtest.root";
TString unpackDir = "Simulation/SeGA/";
	TString dir = getenv("VMCWORKDIR");
   TString geoFile = "SeGA_geomanager.root";
TString GeoDataPath = dir + "/geometry/" + geoFile;
TString InputDataPath = dir + "/macro/" + unpackDir + InputFile;
   TString OutputDataPath = dir + "/macro/" + unpackDir + OutFile;
   // -----   Reconstruction run   -------------------------------------------
  FairRunAna *fRun = new FairRunAna();
   FairRootFileSink *sink = new FairRootFileSink(OutputDataPath);
   FairFileSource *source = new FairFileSource(InputDataPath);
   fRun->SetSource(source);
   fRun->SetSink(sink);
   fRun->SetGeomFile(GeoDataPath);

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParRootFileIo *parInput1 = new FairParRootFileIo();
   parInput1->open(ParFile.Data());
   rtdb->setFirstInput(parInput1);

   FairEventManager *fMan = new FairEventManager();

   //----------------------Traks and points -------------------------------------
   //FairMCTracks *Track = new FairMCTracks("Monte-Carlo Tracks");
   FairMCPointDraw *AtSeGAPoints = new FairMCPointDraw("AtSeGAPoint", kRed, kFullSquare);

   //fMan->AddTask(Track);
   fMan->AddTask(AtSeGAPoints);

   fMan->Init();
}
