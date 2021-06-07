/* Displays events for input fileName.root and parameter file fileNamePar.root
 *
 * Adam Anthony 8/21/19
 */

void eventDisplay(TString fileName = "attpc")
{

   // Form file names
   TString InputFile = TString::Format("./data/%ssim.root", fileName.Data());
   TString ParFile = TString::Format("./data/%spar.root", fileName.Data());
   TString OutFile = TString::Format("./data/%sout.root", fileName.Data());

   // -----   Reconstruction run   -------------------------------------------
   // FairRunAna *fRun= new FairRunAna();
   // fRun->SetSource( new FairFileSource(InputFile) );
   // fRun->SetSink( new FairFileSink(OutFile) );

   FairFileSource *source = new FairFileSource(InputFile);

   FairRunAna *fRun = new FairRunAna();
   fRun->SetSource(source);
   fRun->SetOutputFile(OutFile.Data());

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParRootFileIo *parInput1 = new FairParRootFileIo();
   parInput1->open(ParFile.Data());
   rtdb->setFirstInput(parInput1);

   FairEventManager *fMan = new FairEventManager();

   //----------------------Traks and points -------------------------------------
   FairMCTracksDraw *Track = new FairMCTracksDraw();
   FairMCPointDraw *AtTpcPoints = new FairMCPointDraw("AtTpcPoint", kBlue, kFullSquare);

   fMan->AddTask(Track);
   fMan->AddTask(AtTpcPoints);

   fMan->Init();
}
