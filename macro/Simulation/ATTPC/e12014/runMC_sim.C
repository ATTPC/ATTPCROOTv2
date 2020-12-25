//Code to simulate fission event from a file

void runMC_sim(Int_t nEvents= 20, TString mcEngine = "TGeant4")
{

  TString dir = getenv("VMCWORKDIR");

  //TString ionList = dir + TString("macro/Simulation/E12014/data/PbIonList.txt");
  TString ionList = "./data/PbIonList.txt";
  TString fissionDistro = "./data/PbFissionEvents.root";
  
  // Output file name
  TString outFile ="./data/PbFission_sim.root";

  // Parameter file name
  TString parFile="./data/PbFission_simPar.root";

  // -----   Timer   --------------------------------------------------------
  TStopwatch timer;
  timer.Start();
  // ------------------------------------------------------------------------

  //gSystem->Load("libAtGen.so");

  ATVertexPropagator* vertex_prop = new ATVertexPropagator();


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
  ATTPC->SetGeometryFileName("ATTPC_He1bar.root");

  //ATTPC->SetVerboseLevel(2);
  //ATTPC->SetModifyGeometry(kTRUE);
  run->AddModule(ATTPC);

  // ------------------------------------------------------------------------

  // -----   Create PrimaryGenerator   --------------------------------------
  //This is what everyother generator is added to
  FairPrimaryGenerator* primGen = new FairPrimaryGenerator();
  
  
  /***** Primary Beam Information *****/
  Int_t z = 81; // Atomic number
  Int_t a = 195; // Mass number
  Int_t q = 78;  // Charge State

  // Multiplicity  NOTE: Due the limitation of the TGenPhaseSpace accepting only pointers/arrays
  // the maximum multiplicity has been set to 10 particles.
  Int_t m = 1;
  
  Double_t px = 0.000/a;    // X-Momentum / per nucleon!!!!!!
  Double_t py = 0.000/a;    // Y-Momentum / per nucleon!!!!!!

  // 70 MeV / nucleon
  Double_t pz = 42348.9/a;  // Z-Momentum (MeV)/ per nucleon!!!!!!
  pz /= 1000; // change to GeV/c for FairSoft
  
  Double_t BExcEner = 0.0;

  //TODO: Fix to right mass
  Double_t Bmass = 194.9259; //Mass in amu

  // Nominal Energy of the beam: Only used for cross section calculation
  // (Tracking energy is determined with momentum).
  Double_t NomEnergy = 25.0*a; //Depricated

  //E loss until reaction occurs in MeV
  Double_t eLoss = 1000;

  //Create the ion generator
  ATTPCIonGenerator* ionGen = new ATTPCIonGenerator("Ion", z, a, q, m,
						    px, py, pz, BExcEner,
						    Bmass, NomEnergy, eLoss);
  //Set the beam at enterance of TPC
  ionGen->SetSpotRadius(0,-100,0);

  
  // Create the fission generator
  ATTPCFissionGeneratorV3 *fissionGen =
    new ATTPCFissionGeneratorV3("FissionGenerator", ionList, fissionDistro);

  //Add all of the generators
  primGen->AddGenerator(ionGen);
  primGen->AddGenerator(fissionGen);

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
  run->CreateGeometryFile("./data/geofile_full.root");
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
