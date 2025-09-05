void eventDisplay()
{
  //-----User Settings:-----------------------------------------------
  //TString  InputFile     ="attpcsim_13Be_p_0.0_180.0_550Torr_Xiaobin.root";
  TString  InputFile     ="attpcsim_13Be_p_0.0_180.0_550Torr.root";
  TString  ParFile       ="attpcpar_13Be_p.root";
  TString  OutFile	     ="attpctest.root";


  // -----   Reconstruction run   -------------------------------------------
   FairRunAna *fRun = new FairRunAna();
   FairRootFileSink *sink = new FairRootFileSink(OutFile);
   FairFileSource *source = new FairFileSource(InputFile);
   fRun->SetSource(source);
   fRun->SetSink(sink);

  FairRuntimeDb* rtdb = fRun->GetRuntimeDb();
  FairParRootFileIo* parInput1 = new FairParRootFileIo();
  parInput1->open(ParFile.Data());
  rtdb->setFirstInput(parInput1);

  FairEventManager *fMan= new FairEventManager();

  //----------------------Traks and points -------------------------------------
  //FairMCTracks    *Track     = new FairMCTracks("Monte-Carlo Tracks");
  FairMCPointDraw *AtTpcPoints = new FairMCPointDraw("AtTpcPoint", kBlue, kFullSquare);

  //fMan->AddTask(Track);
  fMan->AddTask(AtTpcPoints);


  fMan->Init();

}
