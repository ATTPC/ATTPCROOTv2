void eventDisplay_proto()
{
  //-----User Settings:-----------------------------------------------
  TString  InputFile     ="./data/attpcsim_proto.root";
  TString  ParFile       ="./data/attpcpar_proto.root";
  TString  OutFile	 ="./data/attpctest_proto.root";


  // -----   Reconstruction run   -------------------------------------------
  FairRunAna *fRun= new FairRunAna();
  fRun->SetInputFile(InputFile.Data());
  fRun->SetOutputFile(OutFile.Data());

  FairRuntimeDb* rtdb = fRun->GetRuntimeDb();
  FairParRootFileIo* parInput1 = new FairParRootFileIo();
  parInput1->open(ParFile.Data());
  rtdb->setFirstInput(parInput1);

  FairEventManager *fMan= new FairEventManager();

  //----------------------Traks and points -------------------------------------
  FairMCTracks    *Track     = new FairMCTracks("Monte-Carlo Tracks");
  FairMCPointDraw *AtTpcPoints = new FairMCPointDraw("AtTpcPoint", kBlue, kFullSquare);

  fMan->AddTask(Track);
  fMan->AddTask(AtTpcPoints);


  fMan->Init();

}
