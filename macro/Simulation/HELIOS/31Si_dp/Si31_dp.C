void Si31_dp(Int_t nEvents = 2000, TString mcEngine = "TGeant4")
{

  TString dir = getenv("VMCWORKDIR");

  // Output file name
  TString outFile ="heliossim.root";

  // Parameter file name
  TString parFile="heliospar.root";

  // -----   Timer   --------------------------------------------------------
  TStopwatch timer;
  timer.Start();
  // ------------------------------------------------------------------------

  ATVertexPropagator* vertex_prop = new ATVertexPropagator();

  Bool_t fApolloDigitizer = true; // Apply hit digitizer task for Apollo

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

  FairDetector* HELIOS = new AtSiArray("HELIOS", kTRUE);
  HELIOS->SetGeometryFileName("SOLARIS_SiArray_v1.0.root");
  //HELIOS->SetModifyGeometry(kTRUE);
  run->AddModule(HELIOS);

  FairDetector* APOLLO = new AtApollo("APOLLO", kTRUE);
  APOLLO->SetGeometryFileName("APOLLO_v0.root");
  //ATTPC->SetModifyGeometry(kTRUE);
  run->AddModule(APOLLO);

 // ------------------------------------------------------------------------


    // -----   Magnetic field   -------------------------------------------
    // Constant Field
    AtConstField  *fMagField = new AtConstField();
    fMagField->SetField(0., 0. ,-20. ); // values are in kG
    fMagField->SetFieldRegion(-100, 100,-100, 100, -250,250); // values are in cm
    fMagField->Print();
                          //  (xmin,xmax,ymin,ymax,zmin,zmax)
    run->SetField(fMagField);
    // --------------------------------------------------------------------



  // -----   Create PrimaryGenerator   --------------------------------------
  FairPrimaryGenerator* primGen = new FairPrimaryGenerator();




                  // Beam Information
                Int_t z = 14;  // Atomic number
	        Int_t a = 31; // Mass number
	        Int_t q = 0;   // Charge State
	        Int_t m = 1;   // Multiplicity  NOTE: Due the limitation of the TGenPhaseSpace accepting only pointers/arrays the maximum multiplicity has been set to 10 particles.
	        Double_t px = 0.000/a;  // X-Momentum / per nucleon!!!!!!
	        Double_t py = 0.000/a;  // Y-Momentum / per nucleon!!!!!!
	        Double_t pz = 4.021/a;  // Z-Momentum / per nucleon!!!!!!
  	        Double_t BExcEner = 0.0;
                Double_t Bmass = 30.975363226999998; //
                Double_t NomEnergy = 1.2; //Used to force the beam to stop within a certain energy range.



	          //ATTPCIonGenerator* ionGen = new ATTPCIonGenerator("Ion",z,a,q,m,px,py,pz,BExcEner,Bmass,NomEnergy);
	          //ionGen->SetSpotRadius(0,-100,0);
	          // add the ion generator

	         // primGen->AddGenerator(ionGen);

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


	          mult = 4; //Number of Nuclei involved in the reaction (Should be always 4) THIS DEFINITION IS MANDATORY (and the number of particles must be the same)
                  ResEner = 278.4; // For fixed target mode (Si Array) in MeV

                  // ---- Beam ----
                  Zp.push_back(z); //
            	  Ap.push_back(a); //
            	  Qp.push_back(q);
                  Pxp.push_back(px);
            	  Pyp.push_back(py);
            	  Pzp.push_back(pz);
            	  Mass.push_back(30.975363226999998);
            	  ExE.push_back(BExcEner);

                  // ---- Target ----
                 Zp.push_back(1); //
		 Ap.push_back(2); //
		 Qp.push_back(0); //
		 Pxp.push_back(0.0);
                 Pyp.push_back(0.0);
		 Pzp.push_back(0.0);
                 Mass.push_back(2.0135532);
		 ExE.push_back(0.0);//In MeV

                  //--- Scattered -----
                Zp.push_back(14); //
                Ap.push_back(32); //
                Qp.push_back(0);
          	Pxp.push_back(0.0);
          	Pyp.push_back(0.0);
          	Pzp.push_back(0.0);
          	Mass.push_back(31.974148082);
          	ExE.push_back(0.0);


                 // ---- Recoil -----
		 Zp.push_back(1); // p
		 Ap.push_back(1); //
		 Qp.push_back(0); //
		 Pxp.push_back(0.0);
                 Pyp.push_back(0.0);
		 Pzp.push_back(0.0);
                 Mass.push_back(1.00783);
		 ExE.push_back(0.0);//In MeV


                 Double_t ThetaMinCMS = 10.0;
                 Double_t ThetaMaxCMS = 35.0;


        ATTPC2Body* TwoBody = new ATTPC2Body("TwoBody",&Zp,&Ap,&Qp,mult,&Pxp,&Pyp,&Pzp,&Mass,&ExE,ResEner, ThetaMinCMS,ThetaMaxCMS);
        TwoBody->SetFixedTargetPosition(0.0,0.0,0.0);
        TwoBody->SetFixedBeamMomentum(0.0,0.0,pz*a);
        primGen->AddGenerator(TwoBody);


        // -----   Create GammaDummyGenerator
             Double_t pdgId = 22;       // 22 for gamma emission, 2212 for proton emission
             Double_t theta1 = 0.;      // polar angle distribution: lower edge (50)
             Double_t theta2 = 90.;    // polar angle distribution: upper edge (51)
             Double_t momentum = 0.001; // GeV/c
             Int_t multiplicity = 1;
             ATTPCGammaDummyGenerator* gammasGen = new ATTPCGammaDummyGenerator(pdgId, multiplicity);
             gammasGen->SetThetaRange(theta1, theta2);
             gammasGen->SetCosTheta();
             gammasGen->SetPRange(momentum, momentum);
             gammasGen->SetDecayChainPoint(0.001,0.1);
             gammasGen->SetDecayChainPoint(0.009701,0.1);
             gammasGen->SetDecayChainPoint(0.009934,0.4);
             gammasGen->SetDecayChainPoint(0.010279,0.2);
             gammasGen->SetDecayChainPoint(0.010846,0.2);
             gammasGen->SetPhiRange(0., 360.); //(2.5,4)
             gammasGen->SetBoxXYZ(-0.1, 0.1, -0.1, 0.1, -0.1, 0.1);
             gammasGen->SetLorentzBoost(0.0); // for instance beta=0.8197505718204776 for 700 A MeV
             // add the gamma generator
             primGen->AddGenerator(gammasGen);

             /*
             31Si beam 9 MeV/u
             target CD2 = ~100 ug/cm2
             silicon array roughly from -10cm to -40cm (upstream of the target (edited)
             to look at excitation energies of 32Si ranging from 9.2 MeV to ~ 10 - 11 MeV
             @Héctor por favor anade tambien las gammas desde un estado 9.5 MeV por
             encima de la neutron separation energy de 32Si, cualquier cosa inventada vale por ahora
              9701, 9934, 10279, 10846
              */

	run->SetGenerator(primGen);

// ------------------------------------------------------------------------

  //---Store the visualiztion info of the tracks, this make the output file very large!!
  //--- Use it only to display but not for production!
  run->SetStoreTraj(kTRUE);

  // ----- Initialize ApolloDigitizer task (from Point Level to Cal Level)
  if (fApolloDigitizer)
    {
      AtApolloDigitizer* apolloDig = new AtApolloDigitizer();
      apolloDig->SetNonUniformity(1.0);           // Non-uniformity: 1 means +-1% max deviation);
      apolloDig->SetExpEnergyRes(6.,3.);          // 5. means 5% at 1 MeV (first CsI, second LaBr)
      apolloDig->SetDetectionThreshold(0.000010); // in GeV!! 0.000010 means 10 keV
      run->AddTask(apolloDig);
    }

  // -----   Initialize simulation run   ------------------------------------
  run->Init();
  // ------------------------------------------------------------------------

  //Trajectory filters




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
  run->CreateGeometryFile("geofile_helios_full.root");
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
