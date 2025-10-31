// 12Be (d,p) 13Be
// Beam energy 21 MeV/u

void Be12_dp_sim(Int_t nEvents = 10000, TString mcEngine = "TGeant4")
{
   Double_t ThetaMinCMS = 0.0;
   Double_t ThetaMaxCMS = 50.0;

   TString dir = getenv("VMCWORKDIR");

   // Output file name
   TString outFile = TString::Format("./simData/attpcsim_13Be_p_%.1f_%.1f_600Torr.root", ThetaMinCMS, ThetaMaxCMS);

   // Parameter file name
   TString parFile = TString::Format("./simData/attpcpar_13Be_p_%.1f_%.1f_600Torr.root", ThetaMinCMS, ThetaMaxCMS);

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();
   // ------------------------------------------------------------------------

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

   FairDetector* ATTPC = new AtTpc("ATTPC", kTRUE);
   ATTPC->SetGeometryFileName("RCNP_ATTPC_600torr.root");
   run->AddModule(ATTPC);

   FairDetector* RCNP_Si = new AtSiArray("Si", kTRUE);
   RCNP_Si->SetGeometryFileName("RCNP_Si.root");
   run->AddModule(RCNP_Si);

   FairDetector* RCNP_GAGG = new AtApollo("GAGG", kTRUE);
   RCNP_GAGG->SetGeometryFileName("RCNP_GAGG.root");
   run->AddModule(RCNP_GAGG);

   // ------------------------------------------------------------------------

   // -----   Create PrimaryGenerator   --------------------------------------
   FairPrimaryGenerator* primGen = new FairPrimaryGenerator();

   // Beam Information
   Int_t z = 4;  // Atomic number
	Int_t a = 12; // Mass number
	Int_t q = 0;   // Charge State
	Int_t m = 1;   // Multiplicity  NOTE: Due the limitation of the TGenPhaseSpace accepting only pointers/arrays the maximum multiplicity has been set to 10 particles.
	Double_t px = 0.000/a;  // X-Momentum / per nucleon!!!!!!
	Double_t py = 0.000/a;  // Y-Momentum / per nucleon!!!!!!
	Double_t pz = 2.392/a;  // Z-Momentum / per nucleon!!!!!!
  	Double_t BExcEner = 0.0;
   Double_t Bmass = 12.02473; //
   Double_t NomEnergy = 74.52; //Used to force the beam to stop within a certain energy range.

   AtTPCIonGenerator* ionGen = new AtTPCIonGenerator("Ion",z,a,q,m,px,py,pz,BExcEner,Bmass,NomEnergy);
	ionGen->SetSpotRadius(0,-100,0);
	primGen->AddGenerator(ionGen); // add the ion generator

   //primGen->SetBeam(1,1,0,0); //These parameters change the position of the vertex of every track added to the Primary Generator
   //primGen->SetTarget(30,0);

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
   ResEner = 0.0; // For fixed target mode (Si Array) in MeV

   // ---- Beam ----
   Zp.push_back(z); //
   Ap.push_back(a); //
   Qp.push_back(q);
   Pxp.push_back(px);
   Pyp.push_back(py);
   Pzp.push_back(pz);
   Mass.push_back(Bmass);
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
   Zp.push_back(4); //
   Ap.push_back(13); //
   Qp.push_back(0);
   Pxp.push_back(0.0);
   Pyp.push_back(0.0);
   Pzp.push_back(0.0);
   Mass.push_back(13.03394);
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

   AtTPC2Body* TwoBody = new AtTPC2Body("TwoBody",&Zp,&Ap,&Qp,mult,&Pxp,&Pyp,&Pzp,&Mass,&ExE,ResEner, ThetaMinCMS,ThetaMaxCMS);
   //TwoBody->SetFixedTargetPosition(0.0,0.0,0.0);
   //TwoBody->SetFixedBeamMomentum(0.0,0.0,pz*a);
   //TwoBody->SetSequentialDecay(kTRUE);
   primGen->AddGenerator(TwoBody);


        // -----   Create GammaDummyGenerator
          /*   Double_t pdgId = 22;       // 22 for gamma emission, 2212 for proton emission
             Double_t theta1 = 0.;      // polar angle distribution: lower edge (50)
             Double_t theta2 = 90.;    // polar angle distribution: upper edge (51)
             Double_t momentum = 0.001; // GeV/c
             Int_t multiplicity = 1;
             ATTPCGammaDummyGenerator* gammasGen = new ATTPCGammaDummyGenerator(pdgId, multiplicity);
             gammasGen->SetThetaRange(theta1, theta2);
             gammasGen->SetCosTheta();
             gammasGen->SetPRange(momentum, momentum);
             gammasGen->SetNuclearDecayChain();
             gammasGen->SetDecayChainPoint(0.001329,0.90);
             gammasGen->SetDecayChainPoint(0.000036,0.10);
             gammasGen->SetPhiRange(0., 360.); //(2.5,4)
             gammasGen->SetBoxXYZ(-0.1, 0.1, -0.1, 0.1, -0.1, 0.1);
             gammasGen->SetLorentzBoost(0.0); // for instance beta=0.8197505718204776 for 700 A MeV
             // add the gamma generator
             primGen->AddGenerator(gammasGen);*/

// Setting decay
   // Set the parameters of the decay generator
/*
   std::vector<std::vector<Int_t>> zDecay;
   std::vector<std::vector<Int_t>> aDecay;
   std::vector<std::vector<Int_t>> qDecay;
   std::vector<std::vector<Double_t>> massDecay;

   Int_t zB;
   Int_t aB;
   Double_t massDecayB;
   Double_t massTarget;
   Double_t exEnergy;
   std::vector<Double_t> SepEne;

   Int_t TotDecayCases = 1; // the number of decay channel (case) to be considered

   zDecay.resize(TotDecayCases);
   aDecay.resize(TotDecayCases);
   qDecay.resize(TotDecayCases);
   massDecay.resize(TotDecayCases);

   zB = 4; // 12Be
   aB = 13;
   massDecayB = 13.03394;
   massTarget = 0.0;
   exEnergy = 0.0; // NB: Set to zero for sequential decay

      SepEne.push_back(-0.435); // Separation energy for the first decay
      zDecay.at(0).push_back(0);
      zDecay.at(0).push_back(4);
      aDecay.at(0).push_back(1);
      aDecay.at(0).push_back(12);
      qDecay.at(0).push_back(0);
      qDecay.at(0).push_back(0);
      massDecay.at(0).push_back(1.0087);
      massDecay.at(0).push_back(12.02473);
   
   

   AtTPCIonDecay *decay =
      new AtTPCIonDecay(&zDecay, &aDecay, &qDecay, &massDecay, zB, aB, massDecayB, massTarget, exEnergy, &SepEne);
   decay->SetSequentialDecay(kTRUE);
   //primGen->AddGenerator(decay);
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
  run->CreateGeometryFile("./simData/RCNP_geo_e565.root");
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
