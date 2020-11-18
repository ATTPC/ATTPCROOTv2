void eventDisplay()
{
  //-----User Settings:-----------------------------------------------
  TString  InputFile     ="heliossim.root";
  TString  ParFile       ="heliospar.root";
  TString  OutFile	 ="heliostest.root";


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
  FairMCPointDraw *AtSiArrayPoints = new FairMCPointDraw("AtSiArrayPoint", kBlue, kFullSquare);

  fMan->AddTask(Track);
  fMan->AddTask(AtSiArrayPoints);


  fMan->Init();

}
