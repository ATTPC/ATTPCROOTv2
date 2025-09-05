void Be13_p_sim(Int_t nEvents = 10000, Int_t subnum = 0, Double_t angle_cm = 180)
{
   // Int_t subnum = 0,
   srand((unsigned)time(NULL));
   gRandom->SetSeed(subnum);

   TString dir = getenv("VMCWORKDIR");

   // Output file name
   double minangle, maxangle;
   minangle = 0, maxangle = angle_cm;
   TString outFile = TString::Format("./attpcsim_13Be_p_%.1f_%.1f_550Torr.root", minangle, maxangle);

   // Parameter file name
   TString parFile = "./attpcpar_13Be_p.root";

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();
   // ------------------------------------------------------------------------

   // -----   Create simulation run   ----------------------------------------
   FairRunSim *run = new FairRunSim();
   run->SetName("TGeant4");     // Transport engine
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

   FairDetector *ATTPC = new AtTpc("ATTPC", kTRUE);
   ATTPC->SetGeometryFileName("ATTPC_C3D8_550torr.root");
   run->AddModule(ATTPC);
   // ------------------------------------------------------------------------

   // -----   Create PrimaryGenerator   --------------------------------------
   FairPrimaryGenerator *primGen = new FairPrimaryGenerator();

   // Beam Information
   Int_t z = 4;  // Atomic number
   Int_t a = 12; // Mass number
   Int_t q = 4;  // Charge State
   Int_t m = 1;  // Multiplicity  NOTE: Due the limitation of the TGenPhaseSpace accepting only pointers/arrays the
                 // maximum multiplicity has been set to 10 particles.
   Double_t kBeam = 18;                // Beam energy per nucleon(MeV/u)
   Double_t BExcEner = 0.0;            // Excitation energy of beam
   Double_t Beam_mass = 12.026922;     // Target mass in MeV
   Double_t Target_mass = 2.014101;    // Beam mass in MeV
   Double_t Recoil_mass = 1.007825;    // Recoil mass in MeV
   Double_t Residual_mass = 13.036134; // Residual mass in MeV
   Double_t px = 0.000 / a;            // X-Momentum GeV/ per nucleon!!!!!!
   Double_t py = 0.000 / a;            // Y-Momentum GeV/ per nucleon!!!!!!
   Double_t pz = 2.21261 / a;          // Z-Momentum GeV/ per nucleon!!!!!!
   Double_t NomEnergy = 71.9428;

   AtTPCIonGenerator *ionGen = new AtTPCIonGenerator("Ion", z, a, q, m, px, py, pz, BExcEner, Beam_mass, NomEnergy);
   ionGen->SetSpotRadius(0, -100, 0);

   primGen->AddGenerator(ionGen); // add the ion generator

   // primGen->SetBeam(1,1,0,0); //These parameters change the position of the vertex of every track added to the
   // Primary Generator
   //  primGen->SetTarget(30,0);

   // Variables for 2-Body kinematics reaction
   std::vector<Int_t> Zp;      // Zp
   std::vector<Int_t> Ap;      // Ap
   std::vector<Int_t> Qp;      // Electric charge
   Int_t mult;                 // Number of particles
   std::vector<Double_t> Pxp;  // Px momentum X
   std::vector<Double_t> Pyp;  // Py momentum Y
   std::vector<Double_t> Pzp;  // Pz momentum Z
   std::vector<Double_t> Mass; // Masses
   std::vector<Double_t> ExE;  // Excitation energy
   Double_t ResEner;           // Energy of the beam (Useless for the moment)

   // Note: Momentum will be calculated from the phase Space according to the residual energy of the beam

   mult = 4; // Number of Nuclei involved in the reaction (Should be always 4) THIS DEFINITION IS MANDATORY (and the
             // number of particles must be the same)
   // ResEner = 0.06; // MeV

   // ---- Beam ----
   Zp.push_back(z); // 12Be TRACKID=0
   Ap.push_back(a); //
   Qp.push_back(q);
   Pxp.push_back(px);
   Pyp.push_back(py);
   Pzp.push_back(pz);
   Mass.push_back(Beam_mass); // uma
   ExE.push_back(BExcEner);

   // ---- Target ----
   Zp.push_back(1); // 2H
   Ap.push_back(2); //
   Qp.push_back(0); //
   Pxp.push_back(0.0);
   Pyp.push_back(0.0);
   Pzp.push_back(0.0);
   Mass.push_back(Target_mass); // uma
   ExE.push_back(0.0);          // In MeV
                                //

   //--- Scattered -----
   Zp.push_back(4);  // 13Be  TRACKID=2
   Ap.push_back(13); //
   Qp.push_back(0);
   Pxp.push_back(0.0);
   Pyp.push_back(0.0);
   Pzp.push_back(0.0);
   Mass.push_back(Residual_mass); // uma
   ExE.push_back(0.0);            //

   // ---- Recoil ------
   Zp.push_back(1);
   Ap.push_back(1); //
   Qp.push_back(0); //
   Pxp.push_back(0.0);
   Pyp.push_back(0.0);
   Pzp.push_back(0.0);
   Mass.push_back(Recoil_mass); // uma
   ExE.push_back(0.0);          // In MeV

   Double_t ThetaMinCMS = minangle;
   Double_t ThetaMaxCMS = maxangle;

   AtTPC2Body *TwoBody =
      new AtTPC2Body("TwoBody", &Zp, &Ap, &Qp, mult, &Pxp, &Pyp, &Pzp, &Mass, &ExE, ResEner, ThetaMinCMS, ThetaMaxCMS);
   primGen->AddGenerator(TwoBody);

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

   TString outgeoFile = "./geofile_full_13Be_p.root";
   run->CreateGeometryFile(outgeoFile);
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
