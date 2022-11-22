#include <time.h> /* time */

#include <stdio.h>  /* printf, NULL */
#include <stdlib.h> /* srand, rand */

void runsim_d2He(Int_t runNumber = 0, Double_t ExEje = 0, Int_t nEvents = 10, TString mcEngine = "TGeant4")
{

   srand((unsigned)time(NULL));
   UInt_t seed = (float)rand() / RAND_MAX * 100000;
   gRandom->SetSeed(seed);

   TString dir = getenv("VMCWORKDIR");

   // Output file name
   TString outFile = Form("/mnt/analysis/e18008/rootAna/giraud/simulation/g4/attpcsim_d2He_run%d_Ex%d_testUpdates.root",
                          runNumber, (Int_t)ExEje);

   // Parameter file name
   TString parFile = Form("/mnt/analysis/e18008/rootAna/giraud/simulation/g4/attpcpar_d2He_run%d_Ex%d_testUpdates.root",
                          runNumber, (Int_t)ExEje);

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();
   // ------------------------------------------------------------------------

   // gSystem->Load("libAtGen.so");

   // AtVertexPropagator* vertex_prop = new AtVertexPropagator();

   // -----   Create simulation run   ----------------------------------------
   FairRunSim *run = new FairRunSim();
   run->SetName(mcEngine);      // Transport engine
   run->SetSink(new FairRootFileSink(outFile)); // Output file
                                                // run->SetOutputFile(outFile); // Output file
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
   ATTPC->SetGeometryFileName("ATTPC_v1.1.root");
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
   Double_t kBeam = 105.16;
   Double_t BExcEner = 0.0;
   Double_t Bmass = 14.008596359 * 931.494 / 1000.0; // Mass in GeV
   Double_t NomEnergy = 0.08; // Nominal Energy of the beam: Only used for cross section calculation (Tracking
                              // energy is determined with momentum). TODO: Change this to the energy after the IC
   // Double_t NomEnergy = 100.;
   // Double_t kBeam = 1000.*(sqrt(Bmass*Bmass+pow(pz*a,2))-Bmass)/a;

   Double_t px = 0.000 / a;
   Double_t py = 0.000 / a; // Y-Momentum / per nucleon!!!!!!
   // Double_t pz = 24798.97727/(a*1000.0);  // Z-Momentum / per nucleon!!!!!!
   Double_t pz = sqrt(pow(kBeam * a / 1000.0 + Bmass, 2) - pow(Bmass, 2)) / a; // Z-Momentum / per nucleon!!!!!!

   // set the following three variables to zero if do not want beam spot
   Double_t fwhmFocus = 0.5;                          // 0.5//cm, FWHM of the gaussian distribution at beam spot
   Double_t angularDiv = 17.E-3;                      // ex:1.E-3 (rad), angular divergence of the beam
   Double_t zFocus = 50.;                             // cm, z position (beam direction) of the beam spot
   Double_t rHole = 1.5;                              // cm, hole radius in the pad plane (entrance)
   Double_t momAcc = 0.0025;                          // percentage, beam momentum acceptance
   Double_t beamAx = -0.00915252 * TMath::RadToDeg(); // deg, beam angle x direction
   Double_t beamAy = -0.00221027 * TMath::RadToDeg(); // deg, beam angle y direction
   Double_t beamOx = -2.24859e-1;                     // cm, beam offset at entrance window x direction
   Double_t beamOy = +0.988841e-1;                    // cm, beam offset at entrance window y direction

   // TString sAta = "";
   // TString sBta = "";
   TString sAta = "ataBeam.root";
   TString sBta = "btaBeam.root";

   // AtTPCIonGenerator* ionGen = new AtTPCIonGenerator("Ion",z,a,q,m,px,py,pz,BExcEner,Bmass,NomEnergy,-1);
   auto ionGen = new AtTPCIonGeneratorS800("Ion", z, a, q, m, px, py, pz, BExcEner, Bmass, NomEnergy, -1, sAta, sBta);
   // AtTPCIonGenerator* ionGen = new AtTPCIonGenerator("Ion",z,a,q,m,px,py,pz,BExcEner,Bmass,NomEnergy,-1);
   ionGen->SetBeamEmittance(fwhmFocus, angularDiv, zFocus, rHole, momAcc, beamAx, beamAy, beamOx, beamOy); //,sAta,sBta
   // ionGen->SetBeamEmittance(fwhmFocus,angularDiv,zFocus,rHole);

   // add the ion generator
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
   Zp.push_back(1); //
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
   ExE.push_back(ExEje);

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
   Int_t N_cross = 630; // wrapAll
   std::vector<Double_t> Arr1(N_cross), Arr2(N_cross), Arr3(N_cross);
   Double_t col1, col2, col3;

   // lee la seccion eficaz desde una tabla
   string filename = Form("cs_accba/dl0_dj1/wrapAll2_dl0dj1_%dMeV.dat", (Int_t)ExEje);
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

   AtTPC_d2He *d2He =
      new AtTPC_d2He("d_2He", &Zp, &Ap, &Qp, mult, &Pxp, &Pyp, &Pzp, &Mass, &ExE, &Arr1, &Arr2, &Arr3, N_cross);
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

   Int_t TotDecayCases = 1; // the number of decay channel (case) to be considered

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

   /*
     //Case 2
     SepEne.push_back(10.55338);
     zDecay.at(0).push_back(7); // 13N
     aDecay.at(0).push_back(13);
     qDecay.at(0).push_back(0);
     massDecay.at(0).push_back(13.005738609);

     zDecay.at(0).push_back(0); //neutron
     aDecay.at(0).push_back(1);
     qDecay.at(0).push_back(0);
     massDecay.at(0).push_back(1.0086649158);
   */

   /*
     //Case 3
     SepEne.push_back(10.262305);//obtained by mass excess difference
     zDecay.at(0).push_back(6); // 12C
     aDecay.at(0).push_back(12);
     qDecay.at(0).push_back(0);
     massDecay.at(0).push_back(12.0);

     zDecay.at(0).push_back(1); //d
     aDecay.at(0).push_back(2);
     qDecay.at(0).push_back(0);
     massDecay.at(0).push_back(2.01410177812);
   */

   /*
     //Case 4
     SepEne.push_back(12.496871);//obtained by mass excess difference
     zDecay.at(0).push_back(6); // 12C
     aDecay.at(0).push_back(12);
     qDecay.at(0).push_back(0);
     massDecay.at(0).push_back(12.0);

     zDecay.at(0).push_back(1); //proton
     aDecay.at(0).push_back(1);
     qDecay.at(0).push_back(0);
     massDecay.at(0).push_back(1.0078250322);

     zDecay.at(0).push_back(0); //neutron
     aDecay.at(0).push_back(1);
     qDecay.at(0).push_back(0);
     massDecay.at(0).push_back(1.0086649158);
   */

   // Case 5
   /*  SepEne.push_back(11.6121);//obtained by mass excess difference
     zDecay.at(0).push_back(5); // 10B
     aDecay.at(0).push_back(10);
     qDecay.at(0).push_back(0);
     massDecay.at(0).push_back(10.012936862);

     zDecay.at(0).push_back(2); //alpha
     aDecay.at(0).push_back(4);
     qDecay.at(0).push_back(0);
     massDecay.at(0).push_back(4.00260325413);
   */

   AtTPCIonDecay *decay_14N = new AtTPCIonDecay(&zDecay, &aDecay, &qDecay, &massDecay, zB, aB, massDecayB, 0, 0,
                                                &SepEne); // 0, 0 are TMass, ExEnergy in AtTPCIonDecay
   decay_14N->SetSequentialDecay(kTRUE);
   // primGen->AddGenerator(decay_14N);

   //-----------------------------------------------------------------------------
   // custom track generator
   /*
     std::vector<Int_t> Zrandp;
     std::vector<Int_t> Arandp;
     std::vector<Int_t> Qrandp;
     std::vector<Double_t> Mrandp;
     std::vector<Double_t> Ep;
     std::vector<Double_t> theta;
     std::vector<Double_t> phi;
     std::vector<TVector3> posVtx;
     std::vector<TVector3> posOffset;

   gRandom->SetSeed (0);
   //TVector3 posip(-0.5+gRandom->Uniform(),-0.5+gRandom->Uniform(),10.);//cm
   //TVector3 posOf(-0.5+gRandom->Uniform(),-0.5+gRandom->Uniform(),gRandom->Uniform(-2,2));//cm
   //Double_t itheta = TMath::DegToRad()*90.;
   //Double_t iphi = TMath::DegToRad()*45.;
   Double_t eneip=900;
   int nTracks = 2;


   for(int ip=1; ip<nTracks+1; ip++){
     Zrandp.push_back(1);
     //Zrandp.push_back(2);
     Arandp.push_back(2);
     //Arandp.push_back(4);
     Qrandp.push_back(0);
     //Mrandp.push_back(4.00260325413*931.494/1000.0);//alpha
     Mrandp.push_back(2.01410177812*931.494);
     //Double_t ir=gRandom->Uniform(700,1300);
     Ep.push_back(eneip/1e6);//GeV
   //Ep.push_back(ir/1e6);//GeV

     //if(ip%2!=0) {
      //posip= {0,0,ip*10.};//cm
      //posip= {0,0,50.};//cm
      //iphi=-TMath::DegToRad()*45.;
     //}
     //posVtx.push_back(posip);
     //posOffset.push_back(posOf);
     //theta.push_back(itheta);
     //phi.push_back(iphi);

   }

   Double_t lambda = 1e5*40.e-6;

   // ATTPC_cusTrack* rand2p = new ATTPC_cusTrack(&Zrandp,&Arandp,&Qrandp,&Mrandp,&Ep);
   ATTPC_cusTrack* rand2p = new ATTPC_cusTrack(&Zrandp,&Arandp,&Qrandp,&Mrandp);
     rand2p->SetPoisson(lambda,Arandp.at(0),Zrandp.at(0),Qrandp.at(0),Mrandp.at(0));

     //if following Set functions not called the angles and vtx positions are random
     // rand2p->SetVtxPosition(&posVtx);
     // rand2p->SetVtxPosition(nTracks,1.);//number of tracks originating from the same vtxZ, maxDistZ (cm) btw tracks
     //rand2p->SetVtxPosition(nTracks,0.,3.5);//number of tracks originating from the same vtxZ, maxDistZ (cm) btw
   tracks

     //rand2p->SetAngles(&theta,&phi);
     rand2p->SetElastic(14.008596359*931.494/1000., 2.01410177812*931.494/1000.);//Mproj,Mtarget,MheavyRecoil//use the
   beam (E,mom) from ATVP, doesnt update the beam (E,m) after scat or d,2He..
     //rand2p->SetAngles(75,95,0,360);
     //rand2p->SetEnergy(0.2,2.);

     //primGen->AddGenerator(rand2p);
   */
   // ---------------------------------------------------------------------------

   run->SetGenerator(primGen);

   // ---------------------------------------------------------------------------

   //---Store the visualiztion info of the tracks, this make the output file very large!!
   //--- Use it only to display but not for production!
   // run->SetStoreTraj(kTRUE);
   // -----   Initialize simulation run   ------------------------------------
   Bool_t kParameterMerged = kTRUE;
   FairParRootFileIo *parOut = new FairParRootFileIo(kParameterMerged);
   parOut->open(parFile);
   rtdb->setOutput(parOut);

   run->Init();
   // ------------------------------------------------------------------------

   // -----   Runtime database   ---------------------------------------------

   // ------------------------------------------------------------------------

   // -----   Start run   ----------------------------------------------------
   run->Run(nEvents);
   // rtdb->saveOutput();
   //   rtdb->Print();
   parOut->close();
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
