// Code to simulate fission event from a file
#include "FairLogger.h"

void runMC_sim(Int_t nEvents = 20000, TString mcEngine = "TGeant4")
{
   // fair::Logger::SetConsoleSeverity("debug");
   // Output file name
   TString dir = getenv("VMCWORKDIR");
   TString inOutDir = "/macro/e12014/simulation/eventGenerator/symFull/";
   TString parFileName = dir + inOutDir + "par_attpc.root";
   TString simFileName = dir + inOutDir + "sim_attpc.root";

   // Input file names
   TString geoFileName = "ATTPC_He1bar.root";
   TString paramFileName = "ATTPC.e12014.par";
   paramFileName = dir + "/parameters/" + paramFileName;
   TString geoFileInputName = dir + "/geometry/" + geoFileName;

   TString ionList = dir + inOutDir + "ion_list.csv";
   TString fissionDistro = dir + inOutDir + "fissionFragments.root";

   TStopwatch timer;
   timer.Start();
   cout << "TIME IS " << timer.RealTime() << endl;

   FairRunSim *run = new FairRunSim();
   run->SetName(mcEngine);                          // Transport engine
   run->SetSink(new FairRootFileSink(simFileName)); // Output file
   FairRuntimeDb *rtdb = run->GetRuntimeDb();
   cout << "TIME IS " << timer.RealTime() << endl;

   run->SetMaterials("media.geo"); // Materials

   FairModule *cave = new AtCave("CAVE");
   cave->SetGeometryFileName("caveSmall.geo");
   run->AddModule(cave);

   AtTpc *ATTPC = new AtTpc("ATTPC", kTRUE);
   ATTPC->SetGeometryFileName(geoFileInputName);

   // ATTPC->SetVerboseLevel(2);
   // ATTPC->SetModifyGeometry(kTRUE);
   run->AddModule(ATTPC);

   // ------------------------------------------------------------------------

   // -----   Create PrimaryGenerator   --------------------------------------
   // This is what everyother generator is added to
   FairPrimaryGenerator *primGen = new FairPrimaryGenerator();

   /***** Primary Beam Information *****/
   Int_t z = 82;  // Atomic number
   Int_t a = 197; // Mass number
   Int_t q = 80;  // Charge State

   // Multiplicity  NOTE: Due the limitation of the TGenPhaseSpace accepting only pointers/arrays
   // the maximum multiplicity has been set to 10 particles.
   Int_t m = 1;

   Double_t px = 0.000 / a; // X-Momentum / per nucleon!!!!!!
   Double_t py = 0.000 / a; // Y-Momentum / per nucleon!!!!!!

   // 70 MeV / nucleon
   Double_t pz = 33918.7 / a; // Z-Momentum (MeV/c)/ per nucleon!!!!!!
   pz /= 1000;                // change to GeV/c for FairSoft

   Double_t BExcEner = 0.0;

   Double_t Bmass = 196.9299; // Mass in amu

   // Nominal Energy of the beam: Only used for cross section calculation
   // (Tracking energy is determined with momentum).
   Double_t NomEnergy = 3109.5; // MeV Depricated

   // E loss until reaction occurs in MeV
   Double_t eAtEndOfTPC = 1190.4; // MeV
   Double_t eLoss = NomEnergy - eAtEndOfTPC;

   // Create the ion generator
   AtTPCIonGenerator *ionGen = new AtTPCIonGenerator("Ion", z, a, q, m, px, py, pz, BExcEner, Bmass, NomEnergy, eLoss);
   // Set the beam at enterance of TPC
   ionGen->SetSpotRadius(0, -100, 0);

   // Create the fission generator
   AtTPCFissionGeneratorV3 *fissionGen = new AtTPCFissionGeneratorV3("FissionGenerator", ionList, fissionDistro);

   // Add all of the generators
   primGen->AddGenerator(ionGen);
   primGen->AddGenerator(fissionGen);

   run->SetGenerator(primGen);

   //---Store the visualiztion info of the tracks, this make the output file very large!!
   //--- Use it only to display but not for production!
   // run->SetStoreTraj(kTRUE);

   // -----   Runtime database   ---------------------------------------------

   Bool_t kParameterMerged = kTRUE;
   FairParRootFileIo *parOut = new FairParRootFileIo(kParameterMerged);
   FairParAsciiFileIo *parIn = new FairParAsciiFileIo();
   parOut->open(parFileName);
   parIn->open(paramFileName, "in");
   rtdb->setFirstInput(parIn);
   rtdb->setOutput(parOut);

   run->Init();
   run->Run(nEvents);

   rtdb->saveOutput();
   rtdb->Print();

   // You can export your ROOT geometry ot a separate file
   run->CreateGeometryFile(geoFileName);
   // ------------------------------------------------------------------------

   parOut->close();
   // -----   Finish   -------------------------------------------------------
   timer.Stop();
   Double_t rtime = timer.RealTime();
   Double_t ctime = timer.CpuTime();
   cout << endl << endl;
   cout << "Macro finished succesfully." << endl;
   cout << "Output file is " << simFileName << endl;
   cout << "Parameter file is " << parFileName << endl;
   cout << "Real time " << rtime << " s, CPU time " << ctime << "s" << endl << endl;
   // ------------------------------------------------------------------------
}
