void Mg20_test_sim(Int_t nEvents = 500, TString mcEngine = "TGeant4")
{

   TString dir = getenv("VMCWORKDIR");

   // Output file name
   TString outFile = "./data/gadgetsim.root";

   // Parameter file name
   TString parFile = "./data/gadgetpar.root";

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

   FairDetector *ATTPC = new AtTpc("ATTPC", kTRUE);
   ATTPC->SetGeometryFileName("GADGET_II.root");
   // ATTPC->SetModifyGeometry(kTRUE);
   run->AddModule(ATTPC);

   // ------------------------------------------------------------------------

   // -----   Create PrimaryGenerator   --------------------------------------
   
    FairPrimaryGenerator* primGen = new FairPrimaryGenerator();

  AtTPC20MgDecay* decay = new AtTPC20MgDecay();
  decay->SetNuclearDecayChain();
  //decay->SetDecayChainPoint(1000040020,0.005485,0.853);
//decay->SetDecayChainPoint(0.005443,0.146);
//decay->SetDecayChainPoint(22,0.000059,0.37);
//decay->SetDecayChainPoint(0.007443,1);
//decay->SetDecayChainPoint(0.001210,1);
//decay->SetDecayChainPoint(0.000506,1);//alpha
decay->SetDecayChainPoint(0.00628,1);

 decay->SetBoxXYZ(-4.0,-4.0,0.0,4.0,4.0,40);
 // decay->SetBoxXYZ(-0.15875,-0.15875,0.1500,0.15875,0.15875,0.1596);
  primGen->AddGenerator(decay);

  run->SetGenerator(primGen);


   // ------------------------------------------------------------------------

   //---Store the visualiztion info of the tracks, this make the output file very large!!
   //--- Use it only to display but not for production!
   run->SetStoreTraj(kTRUE);

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
   run->CreateGeometryFile("./data/geofile_proto_full.root");
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
