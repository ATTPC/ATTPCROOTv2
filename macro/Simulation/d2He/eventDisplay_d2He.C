void eventDisplay_d2He()
{
  //-----User Settings:-----------------------------------------------
  //TString  InputFile     ="/mnt/simulations/attpcroot/data/attpcsim_d2He_12C_1atm.root";
  //TString  ParFile       ="/mnt/simulations/attpcroot/data/attpcpar_d2He_12C_1atm.root";
  TString  OutFile	 ="attpctest_14O_test1.root";

  TString  InputFile     ="./d2He/attpcsim_d2He_test1_new.root";
  TString  ParFile       ="./d2He/attpcpar_d2He_test1_new.root";


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
