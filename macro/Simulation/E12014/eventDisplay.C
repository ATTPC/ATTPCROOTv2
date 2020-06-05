/* Displays events for input fileName.root and parameter file fileNamePar.root
 * 
 * Adam Anthony 8/21/19
 */

void eventDisplay(TString fileName="PbFission_sim")
{

  //Form file names
  TString  InputFile = TString::Format("./data/%s.root", fileName.Data());
  TString  ParFile   = TString::Format("./data/%sPar.root", fileName.Data());
  TString  OutFile   = TString::Format("./data/%sOut.root", fileName.Data());



  // -----   Reconstruction run   -------------------------------------------
  //FairRunAna *fRun= new FairRunAna();
  //fRun->SetSource( new FairFileSource(InputFile) );
  //fRun->SetSink( new FairFileSink(OutFile) );

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
