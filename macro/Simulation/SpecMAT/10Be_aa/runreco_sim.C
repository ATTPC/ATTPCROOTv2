void runreco_sim(TString mcFile = "output_digi.root")
{
   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();

   TString scriptfile = "LookupSpecMATnoScint.xml";
   TString dir = getenv("VMCWORKDIR");
   TString scriptdir = dir + "/scripts/" + scriptfile;
   TString dataDir = dir + "/macro/data/";
   TString geomDir = dir + "/geometry/";
   gSystem->Setenv("GEOMPATH", geomDir.Data());
   TString outFile = "output_digi_PRA.root"; 

   // ------------------------------------------------------------------------
   // __ Run ____________________________________________
   FairRunAna *fRun = new FairRunAna();
   fRun->SetSource(new FairFileSource(mcFile));
   fRun->SetSink(new FairRootFileSink(outFile));
   //fRun->SetGeomFile(GeoDataPath);

   TString parameterFile = "SpecMAT.10Be_aa_sim.par";
   TString digiParFile = dir + "/parameters/" + parameterFile;

   TString triggerFile = "SpecMAT.trigger.par";
   TString trigParFile = dir + "/parameters/" + triggerFile;

   TString mapParFile = scriptdir;

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);
   FairParAsciiFileIo *parIo2 = new FairParAsciiFileIo();
   parIo2->open(trigParFile.Data(), "in");
   rtdb->setSecondInput(parIo2);

   // __ AT digi tasks___________________________________

   AtPRAtask *praTask = new AtPRAtask();
   praTask->SetPersistence(kTRUE);

   /*AtTriggertask *trigTask = new AtTriggertask();
   trigTask  ->  SetAtMap(mapParFile);
   trigTask  ->  SetPersistence(kTRUE);*/

   fRun->AddTask(praTask);
   // fRun -> AddTask(trigTask);

   // __ Init and run ___________________________________

   fRun->Init();
   fRun->Run(0, 20);

   std::cout << std::endl << std::endl;
   std::cout << "Macro finished succesfully." << std::endl << std::endl;
   // -----   Finish   -------------------------------------------------------
   timer.Stop();
   Double_t rtime = timer.RealTime();
   Double_t ctime = timer.CpuTime();
   cout << endl;
   cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
   cout << endl;
   // ------------------------------------------------------------------------
}
