void Mg20_test_sim(Int_t nEvents = 20, TString mcEngine = "TGeant4")
{

  TString dir = getenv("VMCWORKDIR");

  // Output file name
  TString outFile ="./data/gadgetsim.root";

  // Parameter file name
  TString parFile="./data/gadgetpar.root";

  // -----   Timer   --------------------------------------------------------
  TStopwatch timer;
  timer.Start();
  // ------------------------------------------------------------------------

  //gSystem->Load("libAtGen.so");
  AtVertexPropagator* vertex_prop = new AtVertexPropagator();


  // -----   Create simulation run   ----------------------------------------
  FairRunSim* run = new FairRunSim();
  run->SetName(mcEngine);              // Transport engine
  run->SetOutputFile(outFile);          // Output file
  FairRuntimeDb* rtdb = run->GetRuntimeDb();
  // ------------------------------------------------------------------------


  // -----   Create media   -------------------------------------------------
  run->SetMaterials("media.geo");       // Materials
  // ------------------------------------------------------------------------

  // -----   Create geometry   ----------------------------------------------

  FairModule* cave= new AtCave("CAVE");
  cave->SetGeometryFileName("cave.geo");
  run->AddModule(cave);

  //FairModule* magnet = new AtMagnet("Magnet");
  //run->AddModule(magnet);

  /*FairModule* pipe = new AtPipe("Pipe");
  run->AddModule(pipe);*/

  FairDetector* ATTPC = new AtTpc("ATTPC", kTRUE);
  ATTPC->SetGeometryFileName("GADGET_II.root");
  //ATTPC->SetModifyGeometry(kTRUE);
  run->AddModule(ATTPC);

  

  // ------------------------------------------------------------------------


  // -----   Create PrimaryGenerator   --------------------------------------
  FairPrimaryGenerator* primGen = new FairPrimaryGenerator();

  AtTPC20MgDecay* decay = new AtTPC20MgDecay();
  decay->SetNuclearDecayChain();
  //decay->SetDecayChainPoint(1000040020,0.005485,0.853);
//decay->SetDecayChainPoint(0.005443,0.146);
//decay->SetDecayChainPoint(22,0.000059,0.37);
decay->SetDecayChainPoint(0.001210,1);
//decay->SetDecayChainPoint(22, 0.004033,0.284);
decay->SetDecayChainPoint(0.000506,0.716);
decay->SetDecayChainPoint(0.004033,0.284);
//decay->SetDecayChainPoint(0.001298,0.2860);
//decay->SetDecayChainPoint(0.000238,0.7062);
//decay->SetDecayChainPoint(1000040020,0.000756,0.15);
/*decay->SetDecayChainPoint(0.001423,0.038);
decay->SetDecayChainPoint(0.001622,0.019);
decay->SetDecayChainPoint(0.001666,0.051);
decay->SetDecayChainPoint(0.001853,0.0003);
decay->SetDecayChainPoint(0.001905,0.0048);
decay->SetDecayChainPoint(0.002120,0.0011);
decay->SetDecayChainPoint(0.002335,0.006);
decay->SetDecayChainPoint(0.002344,0.029);
decay->SetDecayChainPoint(0.002560,0.031);
decay->SetDecayChainPoint(0.002567,0.023);
decay->SetDecayChainPoint(0.002620,0.004);
decay->SetDecayChainPoint(0.002700,0.00212);
decay->SetDecayChainPoint(0.002782,0.0047);
decay->SetDecayChainPoint(0.002830,0.0010);
decay->SetDecayChainPoint(0.003033,0.0046);
decay->SetDecayChainPoint(0.003096,0.0054);
decay->SetDecayChainPoint(0.003389,0.0008);
decay->SetDecayChainPoint(0.003813,0.0028);
decay->SetDecayChainPoint(0.003820,0.0044);
decay->SetDecayChainPoint(0.004033,0.0031);
decay->SetDecayChainPoint(0.004051,0.009);
decay->SetDecayChainPoint(0.004053,0.003);
decay->SetDecayChainPoint(0.004305,0.102);
decay->SetDecayChainPoint(0.004347,0.0027);
decay->SetDecayChainPoint(0.004544,0.00319);
decay->SetDecayChainPoint(0.004993,0.0008);*/
 decay->SetBoxXYZ(0.0,0.0,10,0.0,0.0,20);
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
  FairParRootFileIo* parOut = new FairParRootFileIo(kParameterMerged);
  parOut->open(parFile.Data());
  rtdb->setOutput(parOut);
  rtdb->saveOutput();
  rtdb->print();
  // ------------------------------------------------------------------------

  // -----   Start run   ----------------------------------------------------
   run->Run(nEvents);

  //You can export your ROOT geometry ot a separate file
  run->CreateGeometryFile("./data/geofile_proto_full.root");
  // ------------------------------------------------------------------------

  // -----   Finish   -------------------------------------------------------
  timer.Stop();
  Double_t rtime = timer.RealTime();
  Double_t ctime = timer.CpuTime();
  cout << endl << endl;
  cout << "Macro finished succesfully." << endl;
  cout << "Output file is "    << outFile << endl;
  cout << "Parameter file is " << parFile << endl;
  cout << "Real time " << rtime << " s, CPU time " << ctime
       << "s" << endl << endl;
  // ------------------------------------------------------------------------
}

