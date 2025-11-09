void eventDisplay()
{
   //-----User Settings:-----------------------------------------------
   //TString  InputDataFile     ="./simData/attpcsim_13Be_p_0.0_40.0_600Torr.root";
   TString  InputDataFile     ="/data/ATTPCROOTv2_results/E565/Simulation/simFiles/attpcsim_13Be_p_0.0_40.0_500Torr.root";
   //TString  InputDataFile     ="/data/ATTPCROOTv2_results/E565/Simulation/simFiles/attpcsim_13Be_p_0.0_40.0_600Torr.root";
   //TString  InputDataFile     ="/data/ATTPCROOTv2_results/E565/Simulation/simFiles/attpcsim_13Be_p_0.0_40.0_700Torr.root";
   //TString  ParFile       ="./simData/attpcpar_13Be_p_0.0_40.0_600Torr.root";
   TString  ParFile       ="/data/ATTPCROOTv2_results/E565/Simulation/simFiles/attpcpar_13Be_p_0.0_40.0_500Torr.root";
   //TString  ParFile       ="/data/ATTPCROOTv2_results/E565/Simulation/simFiles/attpcpar_13Be_p_0.0_40.0_600Torr.root";
   //TString  ParFile       ="/data/ATTPCROOTv2_results/E565/Simulation/simFiles/attpcpar_13Be_p_0.0_40.0_700Torr.root";
   TString  OutputDataFile	 ="rcnpe565_600torr.root";

   // -----   Reconstruction run   -------------------------------------------
   FairRunAna *fRun = new FairRunAna();
   FairRootFileSink *sink = new FairRootFileSink(OutputDataFile);
   FairFileSource *source = new FairFileSource(InputDataFile);
   fRun->SetSource(source);
   fRun->SetSink(sink);
   // fRun->SetGeomFile(GeoDataPath);

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParRootFileIo *parInput1 = new FairParRootFileIo();
   parInput1->open(ParFile.Data());
   rtdb->setFirstInput(parInput1);

   FairEventManager *fMan = new FairEventManager();

   //----------------------Traks and points -------------------------------------
   // FairMCTracks *Track = new FairMCTracks("Monte-Carlo Tracks");
   FairMCPointDraw *AtTpcPoints = new FairMCPointDraw("AtTpcPoint", kBlue, kFullSquare);
   FairMCPointDraw *AtSiArrayPoints = new FairMCPointDraw("AtSiArrayPoint", kBlue, kFullSquare);


   // fMan->AddTask(Track);
   fMan->AddTask(AtTpcPoints);
   fMan->AddTask(AtSiArrayPoints);

   fMan->Init();
}
