void Kr86_He4He3_sim(Int_t nEvents = 100, TString mcEngine = "TGeant4")
{

   TString dir = getenv("VMCWORKDIR");

   // Output file name
   TString outFile = "./data/attpcsim.root";

   // Parameter file name
   TString parFile = "./data/attpcpar.root";

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();
   // ------------------------------------------------------------------------

   // gSystem->Load("libAtGen.so");

   // -----   Create simulation run   ----------------------------------------
   FairRunSim *run = new FairRunSim();
   run->SetName(mcEngine);                      // Transport engine
   run->SetSink(new FairRootFileSink(outFile)); // Output file
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

   FairDetector *ATTPC = new AtTpc("SpecMAT", kTRUE);
   ATTPC->SetGeometryFileName("SpecMAT_He1Bar_v2.root");
   // ATTPC->SetModifyGeometry(kTRUE);
   run->AddModule(ATTPC);

   // ------------------------------------------------------------------------

   // -----   Magnetic field   -------------------------------------------
   // Constant Field
   AtConstField *fMagField = new AtConstField();
   fMagField->SetField(0., 0., -25.);                      // values are in kG
   fMagField->SetFieldRegion(-50, 50, -50, 50, -100, 230); // values are in cm
                                                           //  (xmin,xmax,ymin,ymax,zmin,zmax)
   run->SetField(fMagField);
   // --------------------------------------------------------------------

   // -----   Create PrimaryGenerator   --------------------------------------
   FairPrimaryGenerator *primGen = new FairPrimaryGenerator();

   // Beam Information
   Int_t z = 36; // Atomic number
   Int_t a = 86; // Mass number
   Int_t q = 0;  // Charge State
   Int_t m = 1;  // Multiplicity  NOTE: Due the limitation of the TGenPhaseSpace accepting only pointers/arrays the
                 // maximum multiplicity has been set to 10 particles.
   Double_t px = 0.000 / a; // X-Momentum / per nucleon!!!!!!
   Double_t py = 0.000 / a; // Y-Momentum / per nucleon!!!!!!
   Double_t pz = 9834. / a; // Z-Momentum (MeV/c)/ per nucleon!!!!!!
   pz /= 1000;              // change to GeV/c for FairSoft
   // Double_t pz = 9.834;
   Double_t BExcEner = 0.0;
   Double_t Bmass = 85.9106;
   // Double_t NomEnergy = 40.0;
   Double_t NomEnergy = 260.0;

   // set the following three variables to zero if do not want beam spot
   Double_t fwhmFocus = 0.0; // cm, FWHM of the gaussian distribution at beam spot
   Double_t angularDiv = 0;  // rad, angular divergence of the beam
   Double_t zFocus = 0;      // cm, z position (beam direction) of the beam spot
   Double_t rHole = 0.1;     // cm, hole radius in the pad plane

   AtTPCIonGenerator *ionGen = new AtTPCIonGenerator("Ion", z, a, q, m, px, py, pz, BExcEner, Bmass, NomEnergy);
   ionGen->SetBeamEmittance(fwhmFocus, angularDiv, zFocus, rHole);
   // ionGen->SetSpotRadius(0, -100, 0);
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

   mult = 4; // Number of Nuclei involved in the reaction (Should be always 4) THIS DEFINITION IS MANDATORY (and the
             // number of particles must be the same)
   ResEner = 602.; // MeV

   // ---- Beam ----
   Zp.push_back(z); // 86Kr TRACKID=0
   Ap.push_back(a); //
   Qp.push_back(q);
   Pxp.push_back(px);
   Pyp.push_back(py);
   Pzp.push_back(pz);
   Mass.push_back(Bmass); // uma
   ExE.push_back(BExcEner);

   // ---- Target ----
   Zp.push_back(2); // He4
   Ap.push_back(4); //
   Qp.push_back(0); //
   Pxp.push_back(0.0);
   Pyp.push_back(0.0);
   Pzp.push_back(0.0);
   Mass.push_back(4.0026); // uma
   ExE.push_back(0.0);     // In MeV

   // ---- Recoil -----
   Zp.push_back(2); // He3  TRACKID=2
   Ap.push_back(3); //
   Qp.push_back(0); //
   Pxp.push_back(0.0);
   Pyp.push_back(0.0);
   Pzp.push_back(0.0);
   Mass.push_back(3.0160); // uma
   ExE.push_back(0.0);     // In MeV

   //--- Scattered -----
   Zp.push_back(36); // Kr87 TRACKID=1
   Ap.push_back(87); //
   Qp.push_back(0);
   Pxp.push_back(0.0);
   Pyp.push_back(0.0);
   Pzp.push_back(0.0);
   Mass.push_back(86.9134); // uma
   ExE.push_back(0.0);

   Double_t ThetaMinCMS = 140.0;
   Double_t ThetaMaxCMS = 180.0;

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

   // You can export your ROOT geometry to a separate file
   run->CreateGeometryFile("./data/geofile_full.root");
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
