void Be10_He6aa_sim(Int_t nEvents = 1000, TString mcEngine = "TGeant4")
{

   TString dir = getenv("VMCWORKDIR");

   // Output file name
   TString outFile = "attpcsim.root";

   // Parameter file name
   TString parFile = "attpcpar.root";

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
   ATTPC->SetGeometryFileName("ATTPC_He1bar_v2.root");
   // ATTPC->SetModifyGeometry(kTRUE);
   run->AddModule(ATTPC);

   // -----   Magnetic field   -------------------------------------------
   // Constant Field
   AtConstField *fMagField = new AtConstField();
   fMagField->SetField(0., 0., 30.);                      // values are in kG
   fMagField->SetFieldRegion(-50, 50, -50, 50, -10, 230); // values are in cm (xmin,xmax,ymin,ymax,zmin,zmax)
   run->SetField(fMagField);
   // --------------------------------------------------------------------

   // -----   Create PrimaryGenerator   --------------------------------------
   FairPrimaryGenerator *primGen = new FairPrimaryGenerator();

   // Beam Information
   Int_t z = 4;  // Atomic number
   Int_t a = 10; // Mass number
   Int_t q = 0;  // Charge State
   Int_t m = 1;  // Multiplicity
   // Double_t kBeam = 0.222;
   Double_t BExcEner = 0.0;
   Double_t Bmass = 10.013533818; // Mass in amu
   Double_t NomEnergy = 10; // Nominal Energy of the beam: Only used for cross section calculation (Tracking energy is
                            // determined with momentum). TODO: Change this to the energy after the IC

   Double_t px = 0.000 / a;
   Double_t py = 0.000 / a;   // Y-Momentum / per nucleon!!!!!!
   Double_t pz = 0.86515 / a; // 1.369 / a; // Z-Momentum / per nucleon!!!!!!
   // Double_t pz = sqrt( pow(kBeam * a / 1000.0 + Bmass,2) - pow(Bmass,2) )/a;  // Z-Momentum / per nucleon!!!!!!

   AtTPCIonGenerator *ionGen = new AtTPCIonGenerator("Ion", z, a, q, m, px, py, pz, BExcEner, Bmass, NomEnergy, -1);

   primGen->AddGenerator(ionGen);

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

   //-------------------------------------------------------------------------
   // Set the parameters of the decay generator

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

   // NB: There are two ways of running the simulation
   //  1) Beam generator + decay
   //	1.a) Decay starting from the beam (decaying nucleus) + target four momentum + excitation energy -> decay products
   //     1.b) Decay from compound nucleus formed in fusion (decaying nucleus) + excitation energy -> decay products
   //  2) Beam generator + reaction generator (i.e. two-body) + decay. This case is analogous to 1.b) but the compound
   //  nucleus is the scattered particle in the       reaction that carries Ex energy. This mode has to be enabled with
   //  decay->SetSequentialDecay(kTRUE)

   // Example of 1.b)
   //--- decaying nucleus (sequential decay case) -----
   // should be a reaction product (its momentum is set in the reaction generator)
   /*zB=21; // 41Sc
   aB=41;
   massDecayB=40.96925110;
   massTarget= 0.0;
   exEnergy = 0;*/

   // Example of 1.a)
   //--- decaying nucleus (Beam + decay case or simulatenous) -----
   zB = 4; //
   aB = 10;
   massDecayB = 10.013533818;  // 10Be
   massTarget = 4.00260325415; // 4He //Only applicable for one-step decay (i.e.: simultaneous Beam+Decay)
   exEnergy = 0;

   // Case 1
   SepEne.push_back(6.812); // separation energy of mother nucleus
   zDecay.at(0).push_back(2);
   aDecay.at(0).push_back(4);
   qDecay.at(0).push_back(0);
   massDecay.at(0).push_back(4.00260325415);

   zDecay.at(0).push_back(2);
   aDecay.at(0).push_back(4);
   qDecay.at(0).push_back(0);
   massDecay.at(0).push_back(4.00260325415);

   zDecay.at(0).push_back(2); //
   aDecay.at(0).push_back(6);
   qDecay.at(0).push_back(0);
   massDecay.at(0).push_back(6.018885);

   AtTPCIonDecay *decay =
      new AtTPCIonDecay(&zDecay, &aDecay, &qDecay, &massDecay, zB, aB, massDecayB, massTarget, exEnergy, &SepEne);

   primGen->AddGenerator(decay);

   // ------------------------------------------------------------------------

   run->SetGenerator(primGen);

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

   // You can export your ROOT geometry to a separate file
   // run->CreateGeometryFile("./data/geofile_d2He_full.root");
   //  ------------------------------------------------------------------------

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
