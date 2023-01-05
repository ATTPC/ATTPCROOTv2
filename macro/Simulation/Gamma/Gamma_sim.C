void Gamma_sim(  Int_t nEvents = 100000 ,TString mcEngine = "TGeant4")
{ // Initialization of simulation run
    //Int_t nEvents = 100000 ; // Number of events to be simulated (per thread)
    TString detector = "DeGAi"; // Detector to be used: DeGAi
    TString isotopeName = "Cs137";

   TString dir = getenv("VMCWORKDIR");

   // Output file name
   TString outFile = "./data/"+detector+ "_"+ isotopeName+".root";

   // Parameter file name
   TString parFile = "./data/"+detector +"_"+isotopeName+"par.root";

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

  
    // -----   Setup detector information   ----------------------------------------
   FairDetector *DeGAi = new AtDeGAi("AtDeGAi", kTRUE);
   DeGAi->SetGeometryFileName("DeGAi.root");
   // ATTPC->SetModifyGeometry(kTRUE);
   run->AddModule(DeGAi);

   // ------------------------------------------------------------------------

   // -----   Create PrimaryGenerator   --------------------------------------
   FairPrimaryGenerator *primGen = new FairPrimaryGenerator();

   Double_t pdgId = 22;       // 22 for gamma emission, 2212 for proton emission
     Double_t theta1 = 0;      // polar angle distribution: lower edge (50)
     Double_t theta2 = 180.;    // polar angle distribution: upper edge (51)
     Double_t momentum = 0.01; // GeV/c
     Int_t multiplicity = 1;
     AtTPCGammaDummyGenerator* gammasGen = new AtTPCGammaDummyGenerator(pdgId, multiplicity);
     gammasGen->SetThetaRange(theta1, theta2);
     gammasGen->SetCosTheta();
     gammasGen->SetPRange(momentum, momentum);
	gammasGen->SetNuclearDecayChain();

    // -- sources -- 

     if (isotopeName == "Co60") {
        gammasGen->SetDecayChainPoint(0.001173, 0.50); // 1.173 MeV, 100% probability
        gammasGen->SetDecayChainPoint(0.001332, 0.50); // 1.332 MeV, 100% probability
    } else if (isotopeName == "Cs137") {
        gammasGen->SetDecayChainPoint(0.000662, 0.85); // 0.662 MeV, 85% probability
    } else if (isotopeName == "I131") {
        gammasGen->SetDecayChainPoint(0.000364, 0.81); // 0.364 MeV, 81% probability
    } else if (isotopeName == "Te99") {
        gammasGen->SetDecayChainPoint(0.000140, 0.89); // 0.140 MeV, 89% probability
    } else if (isotopeName == "Na22") {
        gammasGen->SetDecayChainPoint(0.001275, 1.0); // 1.275 MeV, 100% probability
    } else if (isotopeName == "Am241") {
        gammasGen->SetDecayChainPoint(0.000060, 0.36); // 0.060 MeV, 36% probability
    } else if (isotopeName == "Th208") {
        gammasGen->SetDecayChainPoint(0.000583, 0.85); // 0.583 MeV, 85% probability
        gammasGen->SetDecayChainPoint(0.000860, 0.12); // 0.860 MeV, 12% probability
        gammasGen->SetDecayChainPoint(0.002614, 0.36); // 2.614 MeV, 36% probability
    } else if (isotopeName == "U238") {
        gammasGen->SetDecayChainPoint(0.000186, 0.57); // 0.186 MeV, 57% probability
        // Add more lines for other significant gamma energies of Uranium-238
    }
    // Add similar blocks for other isotopes
    else {
        std::cerr << "Isotope not recognized: " << isotopeName << std::endl;
        std::cerr << "Available isotopes: Co60, Ce137, I131, Te99, Na22, Am241, Th208, U238" << std::endl;
    }
     gammasGen->SetPhiRange(0., 360.); //(2.5,4)
    gammasGen->SetXYZ(0.0,0,0);
     gammasGen->SetLorentzBoost(0.0); // for instance beta=0.8197505718204776 for 700 A MeV
     // add the gamma generator
     primGen->AddGenerator(gammasGen);


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
   run->CreateGeometryFile("./data/geofile_gamma_full.root");
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
