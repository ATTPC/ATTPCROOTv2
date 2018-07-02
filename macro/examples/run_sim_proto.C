void run_sim_proto(Int_t nEvents = 10, TString mcEngine = "TGeant4")
{
    
  TString dir = getenv("VMCWORKDIR");

  // Output file name
  TString outFile ="attpcsim_proto.root";
    
  // Parameter file name
  TString parFile="attpcpar_proto.root";
  
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

  /*FairModule* magnet = new AtMagnet("Magnet");
  run->AddModule(magnet);*/

  FairModule* pipe = new AtPipe("Pipe");
  run->AddModule(pipe);
    
  FairDetector* ATTPC = new AtTpc("ATTPC_Proto", kTRUE);
  ATTPC->SetGeometryFileName("ATTPC_Proto_v1.0.root");
  //ATTPC->SetModifyGeometry(kTRUE);
  run->AddModule(ATTPC);

 // ------------------------------------------------------------------------


    // -----   Magnetic field   -------------------------------------------
    // Constant Field
   /* AtConstField  *fMagField = new AtConstField();
    fMagField->SetField(0., 0. ,20. ); // values are in kG
    fMagField->SetFieldRegion(-50, 50,-50, 50, -10,230); // values are in cm
                          //  (xmin,xmax,ymin,ymax,zmin,zmax)
    run->SetField(fMagField);*/
    // --------------------------------------------------------------------

    
    
  // -----   Create PrimaryGenerator   --------------------------------------
  FairPrimaryGenerator* primGen = new FairPrimaryGenerator();
  

   
		 
                  // Beam Information
                  Int_t z = 4;  // Atomic number
	          Int_t a = 10; // Mass number
	          Int_t q = 0;   // Charge State
	          Int_t m = 1;   // Multiplicity  NOTE: Due the limitation of the TGenPhaseSpace accepting only pointers/arrays the maximum multiplicity has been set to 10 particles.
	          Double_t px = 0.001/a;  // X-Momentum / per nucleon!!!!!!
	          Double_t py = 0.001/a;  // Y-Momentum / per nucleon!!!!!!
	          Double_t pz = 0.809/a;  // Z-Momentum / per nucleon!!!!!!
  		  Double_t ExcEner = 0.0;
                  Double_t Bmass = 9.32755; //Mass in GeV
                  Double_t NomEnergy = 35.0; //Nominal Energy of the beam: Only used for cross section calculation (Tracking energy is determined with momentum). Must be consistent with pz
                  Double_t TargetMass = 3.72840;//Mass in GeV


	          ATTPCIonGenerator* ionGen = new ATTPCIonGenerator("Ion",z,a,q,m,px,py,pz,ExcEner,Bmass,NomEnergy);
	          ionGen->SetSpotRadius(1,-20,0);
	          // add the ion generator
		 
	          primGen->AddGenerator(ionGen);
		  
  		  //primGen->SetBeam(1,1,0,0); //These parameters change the position of the vertex of every track added to the Primary Generator
		  // primGen->SetTarget(30,0);
 
                 


		 // Variable definition for Phase Space Calculation
                  std::vector<Int_t> Zp; // Zp of the reaction products      
		  std::vector<Int_t> Ap; // Ap of the reaction products
                  std::vector<Int_t> Qp;//Electric charge of gthe reaction products
                  Int_t mult;  //Number of decaying particles        
 		  std::vector<Double_t> Pxp; //Px momentum X
		  std::vector<Double_t> Pyp; //Py momentum Y
		  std::vector<Double_t> Pzp; //Pz momentum Z
                  std::vector<Double_t> Mass; // Masses of the reaction products
 		  Double_t ResEner; // Energy of the beam 
	          

		  // Note: Momentum will be calculated from the phase Space according to the residual energy of the beam

                  //Initialization of variables for physic case (10Be + 4He -> 6He + 4He + 4He)
	          mult = 3; //THIS DEFINITION IS MANDATORY (and the number of particles must be the same)
                  ResEner = 35.0;

                   //--- Particle 1 -----
		  Zp.push_back(2); // 6He 
		  Ap.push_back(6); // 		  
		  Qp.push_back(0); // 
		  Pxp.push_back(0.0);
	          Pyp.push_back(0.0);
		  Pzp.push_back(0.0);
                  Mass.push_back(5606.56); //In MeV
		 
		  // ---- Particle 2 -----
		  Zp.push_back(2); // 4He 
		  Ap.push_back(4); // 
		  Qp.push_back(0); 
		  Pxp.push_back(0.0);
		  Pyp.push_back(0.0);
		  Pzp.push_back(0.0);
		  Mass.push_back(3728.40);

		  // ----- Particle 3 -----
		  Zp.push_back(2); // 4He 
		  Ap.push_back(4); // 
		  Qp.push_back(0); // 
		  Pxp.push_back(0.0);
		  Pyp.push_back(0.0);
		  Pzp.push_back(0.0);
		  Mass.push_back(3728.40);

		 
		 
                  
        ATTPCIonPhaseSpace* ReacDecay = new ATTPCIonPhaseSpace("Phase",&Zp,&Ap,&Qp,mult,&Pxp,&Pyp,&Pzp,&Mass,ResEner,z,a,px,py,pz,Bmass,TargetMass); 
        primGen->AddGenerator(ReacDecay);

    
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
  run->CreateGeometryFile("geofile_proto_full.root");
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


