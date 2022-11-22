void runsim_d2He(Int_t nEvents = 10000, TString mcEngine = "TGeant4")
{

   TString dir = getenv("VMCWORKDIR");

   // Output file name
   TString outFile = "outputFiles/attpcsim_d2He.root";

   // Parameter file name
   TString parFile = "outputFiles/attpcpar_d2He.root";

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();
   // ------------------------------------------------------------------------

   // gSystem->Load("libAtGen.so");

   ATVertexPropagator *vertex_prop = new ATVertexPropagator();

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
   ATTPC->SetGeometryFileName("ATTPC_d2He_07atm.root");
   // ATTPC->SetModifyGeometry(kTRUE);
   run->AddModule(ATTPC);

   // ------------------------------------------------------------------------

   // -----   Magnetic field   -------------------------------------------
   // Constant Field
   // AtConstField  *fMagField = new AtConstField();
   // fMagField->SetField(0., 0. ,0. ); // values are in kG
   // fMagField->SetFieldRegion(-50, 50,-50, 50, -10,230); // values are in cm
   //  (xmin,xmax,ymin,ymax,zmin,zmax)
   // run->SetField(fMagField);
   // --------------------------------------------------------------------

   // -----   Create PrimaryGenerator   --------------------------------------
   FairPrimaryGenerator *primGen = new FairPrimaryGenerator();

   // Beam Information
   Int_t z = 8;  // Atomic number
   Int_t a = 14; // Mass number
   Int_t q = 0;  // Charge State
   Int_t m = 1;  // Multiplicity  NOTE: Due the limitation of the TGenPhaseSpace accepting only pointers/arrays the
                 // maximum multiplicity has been set to 10 particles.
   Double_t kBeam = 115.;
   Double_t BExcEner = 0.0;
   Double_t Bmass = 14.008596359 * 931.494 / 1000.0; // Mass in GeV
   Double_t NomEnergy = 0.0858025; // Nominal Energy of the beam: Only used for cross section calculation (Tracking
                                   // energy is determined with momentum). TODO: Change this to the energy after the IC
   // Double_t kBeam = 1000.*(sqrt(Bmass*Bmass+pow(pz*a,2))-Bmass)/a;

   Double_t px = 0.000 / a;
   Double_t py = 0.000 / a; // Y-Momentum / per nucleon!!!!!!
   // Double_t pz = 24798.97727/(a*1000.0);  // Z-Momentum / per nucleon!!!!!!
   Double_t pz = sqrt(pow(kBeam * a / 1000.0 + Bmass, 2) - pow(Bmass, 2)) / a; // Z-Momentum / per nucleon!!!!!!

   // set the following three variables to zero if do not want beam spot
   Double_t fwhmFocus = 0.5;     // cm, FWHM of the gaussian distribution at beam spot
   Double_t angularDiv = 10.E-3; // rad, angular divergence of the beam
   Double_t zFocus = 50.;        // cm, z position (beam direction) of the beam spot
   Double_t rHole = 2.;          // cm, hole radius in the pad plane

   // ATTPCIonGenerator* ionGen = new ATTPCIonGenerator("Ion",z,a,q,m,px,py,pz,BExcEner,Bmass,NomEnergy,kBeam);
   // kBeam was set to -1 because the random energy loss is not working for fast beams. In our case this eloss is
   // negligible
   ATTPCIonGenerator *ionGen = new ATTPCIonGenerator("Ion", z, a, q, m, px, py, pz, BExcEner, Bmass, NomEnergy, -1);
   ionGen->SetBeamEmittance(fwhmFocus, angularDiv, zFocus, rHole);
   // ionGen->SetSpotRadius(0,-100,0);
   // add the ion generator

   primGen->AddGenerator(ionGen);

   // primGen->SetBeam(1,1,0,0); //These parameters change the position of the vertex of every track added to the
   // Primary Generator
   // primGen->SetTarget(30,0);

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

   mult = 6; // Number of Nuclei involved in the reaction (Should be always 4) THIS DEFINITION IS MANDATORY (and the
             // number of particles must be the same)
   ResEner = 0.0; // Useless

   // ---- Beam ----
   Zp.push_back(z); // 14O
   Ap.push_back(a); //
   Qp.push_back(0);
   Pxp.push_back((a * 1000.0) * px);
   Pyp.push_back((a * 1000.0) * py);
   Pzp.push_back((a * 1000.0) * pz);
   Mass.push_back(Bmass * 1000.0 / 931.494);
   ExE.push_back(0);

   // ---- Target ----
   Zp.push_back(1); // p
   Ap.push_back(2); //
   Qp.push_back(0); //
   Pxp.push_back(0.0);
   Pyp.push_back(0.0);
   Pzp.push_back(0.0);
   Mass.push_back(2.01410177812);
   ExE.push_back(0.0); // In MeV

   //--- Scattered -----
   Zp.push_back(7);  // 14N
   Ap.push_back(14); //
   Qp.push_back(0);
   Pxp.push_back(0.0);
   Pyp.push_back(0.0);
   Pzp.push_back(0.0);
   Mass.push_back(14.00307400443); //
   ExE.push_back(0.0);

   // ---- Recoil -----
   Zp.push_back(2); //
   Ap.push_back(2); //
   Qp.push_back(0);
   Pxp.push_back(0.0);
   Pyp.push_back(0.0);
   Pzp.push_back(0.0);
   Mass.push_back(2.0 * 1.0078250322);
   ExE.push_back(0.0);

   // ---- proton 1 -----
   Zp.push_back(1); //
   Ap.push_back(1); //
   Qp.push_back(0);
   Pxp.push_back(0.0);
   Pyp.push_back(0.0);
   Pzp.push_back(0.0);
   Mass.push_back(1.0078250322);
   ExE.push_back(0.0);

   // ---- proton 2 -----
   Zp.push_back(1); //
   Ap.push_back(1); //
   Qp.push_back(0);
   Pxp.push_back(0.0);
   Pyp.push_back(0.0);
   Pzp.push_back(0.0);
   Mass.push_back(1.0078250322);
   ExE.push_back(0.0);

   Double_t ThetaMinCMS = 0.0;
   Double_t ThetaMaxCMS = 180.0;
   Int_t N_cross = 1760; // 1760 6560
   std::vector<Double_t> Arr1(N_cross), Arr2(N_cross), Arr3(N_cross);
   Double_t col1, col2, col3;

   // lee la seccion eficaz desde una tabla
   string filename = "0to5_14O.dat"; // all2_14O.dat//0to5_14O.dat
   ifstream inputfile;
   inputfile.open(filename.c_str());
   if (inputfile.fail()) {
      cerr << "error abriendo " << filename << endl;
      exit(1);
   }

   for (Int_t i = 0; i < N_cross; i++) {
      inputfile >> col1 >> col2 >> col3;
      Arr1.at(i) = col1;
      Arr2.at(i) = col2;
      Arr3.at(i) = col3;
   }
   inputfile.close();

   ATTPC_d2He *d2He =
      new ATTPC_d2He("d_2He", &Zp, &Ap, &Qp, mult, &Pxp, &Pyp, &Pzp, &Mass, &ExE, &Arr1, &Arr2, &Arr3, N_cross);
   primGen->AddGenerator(d2He);

   //-------------------------------------------------------------------------
   // Set the parameters of the decay generator

   std::vector<std::vector<Int_t>> zDecay;
   std::vector<std::vector<Int_t>> aDecay;
   std::vector<std::vector<Int_t>> qDecay;
   std::vector<std::vector<Double_t>> massDecay;

   Int_t zB;
   Int_t aB;
   Double_t massDecayB;
   std::vector<Double_t> SepEne;

   Int_t TotDecayCases = 4; // the number of decay channel (case) to be considered

   zDecay.resize(TotDecayCases);
   aDecay.resize(TotDecayCases);
   qDecay.resize(TotDecayCases);
   massDecay.resize(TotDecayCases);

   //--- decaying nucleus -----
   // should be a reaction product (its momentum is set in the reaction generator)
   zB = 7; // 14N
   aB = 14;
   massDecayB = 14.00307400443;

   // ---- Products ----
   // as many first indexes (zDecay.at(0)...) as the value TotDecayCases
   // Case 1
   SepEne.push_back(7.55056);
   zDecay.at(0).push_back(6); // 13C
   aDecay.at(0).push_back(13);
   qDecay.at(0).push_back(0);
   massDecay.at(0).push_back(13.0033548352);

   zDecay.at(0).push_back(1); // proton
   aDecay.at(0).push_back(1);
   qDecay.at(0).push_back(0);
   massDecay.at(0).push_back(1.0078250322);

   // Case 2
   SepEne.push_back(10.55338);
   zDecay.at(1).push_back(7); // 13N
   aDecay.at(1).push_back(13);
   qDecay.at(1).push_back(0);
   massDecay.at(1).push_back(13.005738609);

   zDecay.at(1).push_back(0); // neutron
   aDecay.at(1).push_back(1);
   qDecay.at(1).push_back(0);
   massDecay.at(1).push_back(1.0086649158);

   // Case 3
   SepEne.push_back(10.262305); // obtained by mass excess difference
   zDecay.at(2).push_back(6);   // 12C
   aDecay.at(2).push_back(12);
   qDecay.at(2).push_back(0);
   massDecay.at(2).push_back(12.0);

   zDecay.at(2).push_back(1); // d
   aDecay.at(2).push_back(2);
   qDecay.at(2).push_back(0);
   massDecay.at(2).push_back(2.01410177812);

   // Case 4
   SepEne.push_back(12.496871); // obtained by mass excess difference
   zDecay.at(3).push_back(6);   // 12C
   aDecay.at(3).push_back(12);
   qDecay.at(3).push_back(0);
   massDecay.at(3).push_back(12.0);

   zDecay.at(3).push_back(1); // proton
   aDecay.at(3).push_back(1);
   qDecay.at(3).push_back(0);
   massDecay.at(3).push_back(1.0078250322);

   zDecay.at(3).push_back(0); // neutron
   aDecay.at(3).push_back(1);
   qDecay.at(3).push_back(0);
   massDecay.at(3).push_back(1.0086649158);

   ATTPCIonDecay *decay_14N = new ATTPCIonDecay(&zDecay, &aDecay, &qDecay, &massDecay, zB, aB, massDecayB, &SepEne);

   // primGen->AddGenerator(decay_14N);

   // ------------------------------------------------------------------------

   run->SetGenerator(primGen);

   // ------------------------------------------------------------------------

   //---Store the visualiztion info of the tracks, this make the output file very large!!
   //--- Use it only to display but not for production!
   // run->SetStoreTraj(kTRUE);

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
