void eventDisplayApollo()
{
  //-----User Settings:-----------------------------------------------
  TString  InputFile     ="apollosim.root";
  TString  ParFile       ="apollopar.root";
  TString  OutFile	 ="apollotest.root";


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
  FairMCPointDraw *AtApolloPoints = new FairMCPointDraw("AtApolloPoint", kBlue, kFullSquare);

  fMan->AddTask(Track);
  fMan->AddTask(AtApolloPoints);


  fMan->Init();

}
