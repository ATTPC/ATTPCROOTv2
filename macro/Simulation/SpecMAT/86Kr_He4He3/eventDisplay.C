void eventDisplay()
{
   //-----User Settings:-----------------------------------------------
   TString InputFile = "./data/attpcsim.root";
   TString ParFile = "./data/attpcpar.root";
   TString OutFile = "./data/attpctest.root";

   // -----   Reconstruction run   -------------------------------------------
   FairRunAna *fRun = new FairRunAna();
   fRun->SetSource(new FairFileSource(InputFile));
   fRun->SetSink(new FairRootFileSink(OutFile));

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParRootFileIo *parInput1 = new FairParRootFileIo();
   parInput1->open(ParFile.Data());
   rtdb->setFirstInput(parInput1);

   FairEventManager *fMan = new FairEventManager();

   //----------------------Traks and points -------------------------------------
   FairMCTracksDraw *Track = new FairMCTracksDraw("Monte-Carlo Tracks");
   FairMCPointDraw *AtTpcPoints = new FairMCPointDraw("AtTpcPoint", kBlue, kFullSquare);

   fMan->AddTask(Track);
   fMan->AddTask(AtTpcPoints);

   fMan->Init();
}
