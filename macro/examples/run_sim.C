void run_sim(Int_t nEvents = 100, TString mcEngine = "TGeant4")
{
    
  TString dir = getenv("VMCWORKDIR");

  // Output file name
  TString outFile ="attpcsim.root";
    
  // Parameter file name
  TString parFile="attpcpar.root";
  
  // -----   Timer   --------------------------------------------------------
  TStopwatch timer;
  timer.Start();
  // ------------------------------------------------------------------------

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

  FairModule* magnet = new AtMagnet("Magnet");
  run->AddModule(magnet);

  FairModule* pipe = new AtPipe("Pipe");
  run->AddModule(pipe);
    
  FairDetector* ATTPC = new AtTpc("ATTPC", kTRUE);
  ATTPC->SetGeometryFileName("ATTPC_v1.1.root"); 
  //ATTPC->SetModifyGeometry(kTRUE);
  run->AddModule(ATTPC);

 // ------------------------------------------------------------------------


    // -----   Magnetic field   -------------------------------------------
    // Constant Field
    AtConstField  *fMagField = new AtConstField();
    fMagField->SetField(0., 0. ,20. ); // values are in kG
    fMagField->SetFieldRegion(-50, 50,-50, 50, -10,230); // values are in cm
                          //  (xmin,xmax,ymin,ymax,zmin,zmax)
    run->SetField(fMagField);
    // --------------------------------------------------------------------

    
    
  // -----   Create PrimaryGenerator   --------------------------------------
  FairPrimaryGenerator* primGen = new FairPrimaryGenerator();
  

    // Pythia8
    /* Pythia8Generator* P8gen = new Pythia8Generator();
    P8gen->UseRandom3(); //# TRandom1 or TRandom3 ?
    P8gen->SetParameters("SoftQCD:inelastic = on");
    P8gen->SetParameters("PhotonCollision:gmgm2mumu = on");
    P8gen->SetParameters("PromptPhoton:all = on");
    P8gen->SetParameters("WeakBosonExchange:all = on");
    P8gen->SetMom(40);  //# beam momentum in GeV
    primGen->AddGenerator(P8gen);*/

 
    // Add a box generator also to the run
   /* FairBoxGenerator* boxGen = new FairBoxGenerator(13, 5); // 13 = muon; 1 = multipl.
    boxGen->SetPRange(20,25); // GeV/c
    boxGen->SetPhiRange(0., 360.); // Azimuth angle range [degree]
    boxGen->SetThetaRange(0., 90.); // Polar angle in lab system range [degree]
    boxGen->SetXYZ(0., 0., 0.); // cm
    primGen->AddGenerator(boxGen);*/

      // Box Generator
 /* FairBoxGenerator* boxGen = new FairBoxGenerator(2212, 10); // 13 = muon; 1 = multipl.
  boxGen->SetPRange(2., 2.); // GeV/c //setPRange vs setPtRange
  boxGen->SetPhiRange(0, 360); // Azimuth angle range [degree]
  boxGen->SetThetaRange(3, 10); // Polar angle in lab system range [degree]
  boxGen->SetCosTheta();//uniform generation on all the solid angle(default)*/

                /*  Int_t z = 18;  // Atomic number
	          Int_t a = 34; // Mass number
	          Int_t q = 0;   // Charge State
	          Int_t m = 1;   // Multiplicity
	          Double_t px = 0.01/a;  // X-Momentum / per nucleon!!!!!!
	          Double_t py = 0.01/a;  // Y-Momentum / per nucleon!!!!!!
	          Double_t pz = 4./a;  // Z-Momentum / per nucleon!!!!!!
	          ATTPCIonGenerator* ionGen = new ATTPCIonGenerator(z,a,q,m,px,py,pz);
	          ionGen->SetSpotRadius(1,1,0);
	          // add the ion generator
	          primGen->AddGenerator(ionGen);*/

		      Int_t z = 2;  // Atomic number
	          Int_t a = 4; // Mass number
	          Int_t q = 0;   // Charge State
	          Int_t m = 1;   // Multiplicity
	          Double_t px = 0.01/a;  // X-Momentum / per nucleon!!!!!!
	          Double_t py = 0.01/a;  // Y-Momentum / per nucleon!!!!!!
	          Double_t pz = 0.300/a;  // Z-Momentum / per nucleon!!!!!!
	          ATTPCIonGenerator* ionGen = new ATTPCIonGenerator(z,a,q,m,px,py,pz);
	          ionGen->SetSpotRadius(1,-20,0);
	          // add the ion generator
	          primGen->AddGenerator(ionGen);
  
 
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
  run->CreateGeometryFile("geofile_full.root");
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


