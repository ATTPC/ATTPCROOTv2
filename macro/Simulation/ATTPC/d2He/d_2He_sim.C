void d_2He_sim(Int_t nEvents = 20, TString mcEngine = "TGeant4")
{

  TString dir = getenv("VMCWORKDIR");

  // Output file name
  TString outFile ="./data/attpcsim_d2He.root";

  // Parameter file name
  TString parFile="./data/attpcpar_d2He.root";

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

  FairDetector* ATTPC = new AtTpc("ATTPC", kTRUE);
  ATTPC->SetGeometryFileName("ATTPC_d2He.root");
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
                  Int_t z = 28;  // Atomic number
	          Int_t a = 56; // Mass number
	          Int_t q = 0;   // Charge State
	          Int_t m = 1;   // Multiplicity  NOTE: Due the limitation of the TGenPhaseSpace accepting only pointers/arrays the maximum multiplicity has been set to 10 particles.
	          Double_t px = 0.000/a;  
	          Double_t py = 0.000/a;  // Y-Momentum / per nucleon!!!!!!
	          Double_t pz = 24798.97727/(a*1000.0);  // Z-Momentum / per nucleon!!!!!!
  	          Double_t BExcEner = 0.0;
                  Double_t Bmass = 55.942128549*931.494/1000.0; //Mass in GeV
                  Double_t NomEnergy = 100; //Nominal Energy of the beam: Only used for cross section calculation (Tracking energy is determined with momentum). TODO: Change this to the energy after the IC
                


	          ATTPCIonGenerator* ionGen = new ATTPCIonGenerator("Ion",z,a,q,m,px,py,pz,BExcEner,Bmass,NomEnergy);
	          ionGen->SetSpotRadius(0,-100,0);
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
                  Zp.push_back(z); // 40Ar TRACKID=0
		  Ap.push_back(a); //
		  Qp.push_back(0);
		  Pxp.push_back((a*1000.0)*px);
		  Pyp.push_back((a*1000.0)*py);
		  Pzp.push_back((a*1000.0)*pz);
		  Mass.push_back(Bmass*1000.0/931.494);
		  ExE.push_back(0);

                  // ---- Target ----
                 Zp.push_back(1); // p
		 Ap.push_back(2); //
		 Qp.push_back(0); //
		 Pxp.push_back(0.0);
                 Pyp.push_back(0.0);
		 Pzp.push_back(0.0);
                 Mass.push_back(2.01410177812);
		 ExE.push_back(0.0);//In MeV

                  //--- Scattered -----
                Zp.push_back(27); // 40Ar TRACKID=1
          	Ap.push_back(56); //
          	Qp.push_back(0);
          	Pxp.push_back(0.0);
          	Pyp.push_back(0.0);
          	Pzp.push_back(0.0);
          	Mass.push_back(55.939838798);
          	ExE.push_back(0.0);


                  // ---- Recoil -----
		Zp.push_back(2); // 40Ar TRACKID=1
          	Ap.push_back(2); //
          	Qp.push_back(0);
          	Pxp.push_back(0.0);
          	Pyp.push_back(0.0);
          	Pzp.push_back(0.0);
          	Mass.push_back(2.0*1.0078250322);
          	ExE.push_back(0.0);

		  // ---- proton 1 -----
		Zp.push_back(1); // 40Ar TRACKID=1
          	Ap.push_back(1); //
          	Qp.push_back(0);
          	Pxp.push_back(0.0);
          	Pyp.push_back(0.0);
          	Pzp.push_back(0.0);
          	Mass.push_back(1.0078250322);
          	ExE.push_back(0.0);

		  // ---- proton 2 -----
		Zp.push_back(1); // 40Ar TRACKID=1
          	Ap.push_back(1); //
          	Qp.push_back(0);
          	Pxp.push_back(0.0);
          	Pyp.push_back(0.0);
          	Pzp.push_back(0.0);
          	Mass.push_back(1.0078250322);
          	ExE.push_back(0.0);

                  Double_t ThetaMinCMS = 0.0;
                  Double_t ThetaMaxCMS = 180.0;
		Int_t N_cross = 6560;
		std::vector<Double_t> Arr1(N_cross), Arr2(N_cross), Arr3(N_cross);
		Double_t col1, col2, col3;

	//lee la seccion eficaz desde una tabla
	string filename= "all2.dat";
	ifstream  inputfile;
	inputfile. open(filename.c_str());
      	if(inputfile.fail() ){
                       cerr << "error abriendo "<< filename << endl;
 			exit(1);
                      }  

	for(Int_t i=0;i<N_cross;i++){
		inputfile >> col1 >> col2 >> col3 ;
		Arr1.at(i) = col1;
		Arr2.at(i) = col2;
		Arr3.at(i) = col3;
		}
	inputfile.close();
 

        
	ATTPC_d2He* d2He = new ATTPC_d2He("d_2He",&Zp,&Ap,&Qp,mult,&Pxp,&Pyp,&Pzp,&Mass,&ExE, &Arr1, &Arr2, &Arr3, N_cross);
        primGen->AddGenerator(d2He);

	

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
