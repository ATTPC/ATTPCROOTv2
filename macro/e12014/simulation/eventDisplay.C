/* Displays events for input fileName.root and parameter file fileNamePar.root
 *
 * Adam Anthony 8/21/19
 */

void eventDisplay(TString fileName = "attpc")
{
   // Form file names
   TString InputFile = TString::Format("./data/sim_%s.root", fileName.Data());
   TString ParFile = TString::Format("./data/par_%s.root", fileName.Data());
   TString OutFile = TString::Format("./data/out_%s.root", fileName.Data());
   
   // -----   Reconstruction run   -------------------------------------------
   // FairRunAna *fRun= new FairRunAna();
   // fRun->SetSource( new FairFileSource(InputFile) );
   // fRun->SetSink( new FairFileSink(OutFile) );
   
   FairFileSource *source = new FairFileSource(InputFile);
   FairRunAna *fRun = new FairRunAna();
   fRun->SetSource(source);
   fRun->SetSink(new FairRootFileSink(OutFile));
   
   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParRootFileIo *parInput1 = new FairParRootFileIo();
   std::cout << "Opening file name: " << ParFile << std::endl;
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
