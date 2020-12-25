void d_2He_sim_Bk(Int_t nEvents = 100000, TString mcEngine = "TGeant4", TString geovar = "_1atm" )
{

  TString dir = getenv("VMCWORKDIR");

 const TString pathtodata = "/mnt/simulations/attpcroot/data/";
  // Output file name
  TString outFile = pathtodata + "attpcsim_d2He_Bg12C" + geovar + ".root";
//TString outFile = pathtodata + "attpcsim_test.root";


  // Parameter file name
 TString parFile= pathtodata + "attpcpar_d2He_Bg12C" + geovar + ".root";
  //TString parFile= pathtodata + "attpcpar_test.root";

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
  TString geoname = "ATTPC_d2He" + geovar + ".root";

  FairDetector* ATTPC = new AtTpc("ATTPC", kTRUE);
  ATTPC->SetGeometryFileName(geoname);
  //ATTPC->SetModifyGeometry(kTRUE);
  run->AddModule(ATTPC);

 // ------------------------------------------------------------------------


    // -----   Magnetic field   -------------------------------------------
    // Constant Field
    //AtConstField  *fMagField = new AtConstField();
    //fMagField->SetField(0., 0. ,0. ); // values are in kG
    //fMagField->SetFieldRegion(-50, 50,-50, 50, -10,230); // values are in cm
                          //  (xmin,xmax,ymin,ymax,zmin,zmax)
    //run->SetField(fMagField);
    // --------------------------------------------------------------------



  // -----   Create PrimaryGenerator   --------------------------------------
  FairPrimaryGenerator* primGen = new FairPrimaryGenerator();




                  // Beam Information
                  Int_t z = 6;  // Atomic number
	          Int_t a = 12; // Mass number
	          Int_t q = 0;   // Charge State
	          Int_t m = 1;   // Multiplicity  NOTE: Due the limitation of the TGenPhaseSpace accepting only pointers/arrays the maximum multiplicity has been set to 10 particles.
	          Double_t px = 0.000/a;  
	          Double_t py = 0.000/a;  // Y-Momentum / per nucleon!!!!!!
	          Double_t pz = 5316.6744/(a*1000.0);  // Z-Momentum / per nucleon!!!!!!
  	          Double_t BExcEner = 0.0;
                  Double_t Bmass = 12.00*931.494/1000.0; //Mass in GeV
                  Double_t NomEnergy = 1; //Nominal Energy of the beam: Only used for cross section calculation (Tracking energy is determined with momentum). TODO: Change this to the energy after the IC
                


	          ATTPCIonGenerator* ionGen = new ATTPCIonGenerator("Ion",z,a,q,m,px,py,pz,BExcEner,Bmass,NomEnergy);
	          //ionGen->SetSpotRadius(0,-100,0);
	          // add the ion generator

	          primGen->AddGenerator(ionGen);

  		  //primGen->SetBeam(1,1,0,0); //These parameters change the position of the vertex of every track added to the Primary Generator
		  // primGen->SetTarget(30,0);




		 // Variables for 2-Body kinematics reaction
                  std::vector<Int_t> Zp; // Zp
		   std::vector<Int_t> Ap; // Ap
                  std::vector<Int_t> Qp;//Electric charge
                  Int_t mult;  //Number of particles
 		  std::vector<Double_t> Pxp; //Px momentum X
		  std::vector<Double_t> Pyp; //Py momentum Y
		  std::vector<Double_t> Pzp; //Pz momentum Z
                  std::vector<Double_t> Mass; // Masses
		  std::vector<Double_t> ExE; // Excitation energy
 		  Double_t ResEner; // Energy of the beam (Useless for the moment)


		  // Note: Momentum will be calculated from the phase Space according to the residual energy of the beam


	          mult = 6; //Number of Nuclei involved in the reaction (Should be always 4) THIS DEFINITION IS MANDATORY (and the number of particles must be the same)
                  ResEner = 0.0; // Useless

                  // ---- Beam ----
                  Zp.push_back(z); 
		  Ap.push_back(a); //
		  Qp.push_back(0);
		  Pxp.push_back((a*1000.0)*px);
		  Pyp.push_back((a*1000.0)*py);
		  Pzp.push_back((a*1000.0)*pz);
		  Mass.push_back(Bmass*1000.0/931.494);
		  ExE.push_back(0);

                  // ---- Target ----
                 Zp.push_back(1); // 
		 Ap.push_back(2); //
		 Qp.push_back(0); //
		 Pxp.push_back(0.0);
                 Pyp.push_back(0.0);
		 Pzp.push_back(0.0);
                 Mass.push_back(2.01410177812);
		 ExE.push_back(0.0);//In MeV

                  //--- recoil 1 -----
                Zp.push_back(1); // 
          	Ap.push_back(1); //
          	Qp.push_back(0);
          	Pxp.push_back(0.0);
          	Pyp.push_back(0.0);
          	Pzp.push_back(0.0);
          	Mass.push_back(1.007825);
          	ExE.push_back(0.0);


                  // ---- ejectile 1 -----
		Zp.push_back(6); // 
          	Ap.push_back(13); //
          	Qp.push_back(0);
          	Pxp.push_back(0.0);
          	Pyp.push_back(0.0);
          	Pzp.push_back(0.0);
          	Mass.push_back(13.005);
          	ExE.push_back(0.0);

		  // ---- recoil 2 -----
		Zp.push_back(1); // 
          	Ap.push_back(2); //
          	Qp.push_back(0);
          	Pxp.push_back(0.0);
          	Pyp.push_back(0.0);
          	Pzp.push_back(0.0);
          	Mass.push_back(2.01410177812);
          	ExE.push_back(0.0);

		  // ---- ejectile 2 -----
		Zp.push_back(6); 
          	Ap.push_back(12); //
          	Qp.push_back(0);
          	Pxp.push_back(0.0);
          	Pyp.push_back(0.0);
          	Pzp.push_back(0.0);
          	Mass.push_back(12.00000000000);
          	ExE.push_back(0.0);

                  Double_t ThetaMinCMS = 0.0;
                  Double_t ThetaMaxCMS = 180.0;
	
        
	ATTPC_Background* Backg = new ATTPC_Background("Backg",&Zp,&Ap,&Qp,mult,&Pxp,&Pyp,&Pzp,&Mass,&ExE);
        primGen->AddGenerator(Backg);

	

	run->SetGenerator(primGen);
	
// ------------------------------------------------------------------------

  //---Store the visualiztion info of the tracks, this make the output file very large!!
  //--- Use it only to display but not for production!
  //run->SetStoreTraj(kTRUE);



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

  //You can export your ROOT geometry to a separate file
  //run->CreateGeometryFile("./data/geofile_d2He_full.root");
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

