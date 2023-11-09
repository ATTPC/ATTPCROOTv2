void Mg20_test_sim(Int_t nEvents = 100, TString mcEngine = "TGeant4")
{

   TString dir = getenv("VMCWORKDIR");

   // Output file name
   TString outFile = "./data/gadgetsim.root";

   // Parameter file name
   TString parFile = "./data/gadgetpar.root";

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();
   // ------------------------------------------------------------------------

   // gSystem->Load("libAtGen.so");
   //AtVertexPropagator *vertex_prop = new AtVertexPropagator();

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
   ATTPC->SetGeometryFileName("GADGET_II.root");
   // ATTPC->SetModifyGeometry(kTRUE);
   run->AddModule(ATTPC);

   // ------------------------------------------------------------------------

   // -----   Create PrimaryGenerator   --------------------------------------
   FairPrimaryGenerator *primGen = new FairPrimaryGenerator();

   AtTPC20MgDecay *decay = new AtTPC20MgDecay();
   decay->SetNuclearDecayChain();

   decay->SetDecayChainPoint(0.000984, 0.7142);   // beta1
   decay->SetDecayChainPoint(0.002645, 0.00103);  // beta2
   decay->SetDecayChainPoint(0.007440, 0.000103); // beta13
   decay->SetDecayChainPoint(0.001210, 1);        // p
   decay->SetDecayChainPoint(0.000506, 1);        // a
   decay->SetDecayChainPoint(0.003001, 0.11903);  // beta3
   decay->SetDecayChainPoint(0.000806, 1);        // p1
   decay->SetDecayChainPoint(0.004800, 0.01966);  // beta6
   decay->SetDecayChainPoint(0.001056, 0.4375);   // p2
   decay->SetDecayChainPoint(0.002256, 0.1875);   // p6
   decay->SetDecayChainPoint(0.002344, 0.25);     // p7
   decay->SetDecayChainPoint(0.002559, 0.125);    // p8
   decay->SetDecayChainPoint(0.003874, 0.0496);   // beta4
   decay->SetDecayChainPoint(0.001416, 0.0769);   // p3
   decay->SetDecayChainPoint(0.001679, 0.9231);   // p4
   decay->SetDecayChainPoint(0.004123, 0.02794);  // beta5
   decay->SetDecayChainPoint(0.001928, 1);        // p5
   decay->SetDecayChainPoint(0.005600, 0.01552);  // beta7
   decay->SetDecayChainPoint(0.003003, 1);        // p9
   decay->SetDecayChainPoint(0.005836, 0.00579);  // beta8
   decay->SetDecayChainPoint(0.003389, 1);        // p10
   decay->SetDecayChainPoint(0.006266, 0.0124);   // beta9
   decay->SetDecayChainPoint(0.003820, 1);        // p11
   decay->SetDecayChainPoint(0.006534, 0.03415);  // beta10
   decay->SetDecayChainPoint(0.004071, 0.333);    // p12
   decay->SetDecayChainPoint(0.004326, 0.666);    // p13
   decay->SetDecayChainPoint(0.006770, 0.000310); // beta11
   decay->SetDecayChainPoint(0.006920, 0.000103); // beta12
   decay->SetDecayChainPoint(0.011885, 000016);   // beta14
   decay->SetDecayChainPoint(0.007260, 1);        // a1
   decay->SetDecayChainPoint(0.011320, 0.000263); // beta15
   decay->SetDecayChainPoint(0.006561, 1);        // a2
   decay->SetDecayChainPoint(0.011262, 0.00205);  // beta16
   decay->SetDecayChainPoint(0.010884, 0.00117);  // beta17
   decay->SetDecayChainPoint(0.010843, 0.00174);  // beta18
   decay->SetDecayChainPoint(0.006106, 1);        // a3
   decay->SetDecayChainPoint(0.010584, 0.000883); // beta19
   decay->SetDecayChainPoint(0.005844, 1);        // a4
   decay->SetDecayChainPoint(0.010274, 0.002877); // beta20
   decay->SetDecayChainPoint(0.005540, 1);        // a5
   decay->SetDecayChainPoint(0.009873, 0.00028);  // beta21
   decay->SetDecayChainPoint(0.009487, 0.00241);  // beta22
   decay->SetDecayChainPoint(0.004749, 1);        // a6
   decay->SetDecayChainPoint(0.008800, 0.000625); // beta23
   decay->SetDecayChainPoint(0.004140, 1);        // a7
   decay->SetDecayChainPoint(0.007829, 0.00583);  // beta24
   decay->SetDecayChainPoint(0.003099, 1);        // a8
   decay->SetDecayChainPoint(0.007422, 0.1596);   // beta25
   decay->SetDecayChainPoint(0.002692, 1);        // a9
   decay->SetDecayChainPoint(0.001633, 0.7944);   // beta26

   decay->SetBoxXYZ(-2.0, -2.0, 15.0, 2.0, 2.0, 25.0);
   // decay->SetBoxXYZ(-4.15875,-0.15875,0.1500,0.15875,0.15875,0.1596);
   primGen->AddGenerator(decay);

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
   run->CreateGeometryFile("./data/geofile_proto_full.root");
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
