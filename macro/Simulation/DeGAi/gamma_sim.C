void gamma_sim(Double_t energy,Int_t nEvents, TString mcEngine = "TGeant4" )
{

   TString dir = getenv("VMCWORKDIR");

   // Output file name
   TString outFile = "./DeGAi_test.root";

   // Parameter file name
   TString parFile = "./data/DeGAipar.root";

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();
   // ------------------------------------------------------------------------

   // -----   Create simulation run   ----------------------------------------
   FairRunSim *run = new FairRunSim();
   run->SetName(mcEngine);      // Transport engine
   run->SetOutputFile(outFile); // Output file
   FairRuntimeDb *rtdb = run->GetRuntimeDb();
   // ------------------------------------------------------------------------

   // -----   Create media   -------------------------------------------------
   run->SetMaterials("media.geo"); // Materials
   // ------------------------------------------------------------------------

   // -----   Create geometry   ----------------------------------------------

   FairModule *cave = new AtCave("CAVE");
   cave->SetGeometryFileName("cave.geo");
   run->AddModule(cave);

   // FairModule* magnet = new AtMagnet("Magnet");
   // run->AddModule(magnet);

   /*FairModule* pipe = new AtPipe("Pipe");
   run->AddModule(pipe);*/

   FairDetector *DeGAi = new AtDeGAi("AtDeGAi", kTRUE);
   DeGAi->SetGeometryFileName("DeGAi.root");
   // ATTPC->SetModifyGeometry(kTRUE);
   run->AddModule(DeGAi);

   // ------------------------------------------------------------------------

   // -----   Create PrimaryGenerator   --------------------------------------
   FairPrimaryGenerator *primGen = new FairPrimaryGenerator();

   Double_t pdgId = 22;       // 22 for gamma emission, 2212 for proton emission
     Double_t theta1 = 0;      // polar angle distribution: lower edge (50)
     Double_t theta2 = 180.;    // polar angle distribution: upper edge (51)
     // GeV/c
     
     Int_t multiplicity = 1;
     auto boxGen = new FairBoxGenerator(22, 1);
   boxGen->SetXYZ(0, 0, 20);
    boxGen->SetThetaRange(theta1, theta2);
    boxGen->SetPhiRange(0, 360);
    //boxGen->SetPRange();
   boxGen->SetEkinRange(energy/1000, energy/1000);

   primGen->AddGenerator(boxGen);
  run->SetGenerator(primGen);

   // ------------------------------------------------------------------------

   //---Store the visualiztion info of the tracks, this make the output file very large!!
   //--- Use it only to display but not for production!
   run->SetStoreTraj(kFALSE);

   // -----   Initialize simulation run   ------------------------------------
   run->Init();
   // ------------------------------------------------------------------------

   // -----   Runtime database   ---------------------------------------------

   Bool_t kParameterMerged = kTRUE;
   FairParRootFileIo *parOut = new FairParRootFileIo(kParameterMerged);
   parOut->open(parFile.Data());
   rtdb->setOutput(parOut);
   rtdb->saveOutput();
   rtdb->print();
   // ------------------------------------------------------------------------

   // -----   Start run   ----------------------------------------------------
   run->Run(nEvents);

   // You can export your ROOT geometry ot a separate file
   run->CreateGeometryFile("./data/geofile_gamma_full.root");
   // ------------------------------------------------------------------------

   // -----   Finish   -------------------------------------------------------
   timer.Stop();
   Double_t rtime = timer.RealTime();
   Double_t ctime = timer.CpuTime();
   cout << endl << endl;
   cout << "Macro finished succesfully." << endl;
   cout << "Output file is " << outFile << endl;
   cout << "Parameter file is " << parFile << endl;
   cout << "Real time " << rtime << " s, CPU time " << ctime << "s" << endl << endl;
   // ------------------------------------------------------------------------
}
