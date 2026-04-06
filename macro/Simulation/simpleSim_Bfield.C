/**
 * simpleSim_Bfield.C
 *
 * Standalone AT-TPC simulation using AtTestSimulation (no Geant4).
 * Fires 1 MeV protons into a 2 T solenoid field along Z.
 *
 * Expected physics:
 *   A 1 MeV proton (range ~200 mm in H gas at the table density) stops within the
 *   drift volume, producing a visible Bragg peak.  p⊥ = 0 (parallel to B) so the
 *   track is straight.  To see Larmor curvature set a non-zero theta or use a
 *   reaction generator.
 *
 * Energy loss table: HinH.txt (H in H gas, SRIM format with density header).
 *   proton_He_700torr.txt is available but lacks the SRIM density/conversion
 *   header required by AtELossTable; use HinH.txt as stand-in until a
 *   properly-formatted He table is generated.
 *
 * Output: ./data/simpleSim_Bfield.root
 *   Tree "cbmsim", branch "AtTpcPoint" (TClonesArray of AtMCPoint).
 *   Open with ROOT browser or compareSimVsGeant.C.
 *
 * Usage:
 *   source build/config.sh
 *   root -l -q 'macro/Simulation/simpleSim_Bfield.C(100)'
 */

#include <iostream>
#include <memory>

void simpleSim_Bfield(Int_t nEvents = 100)
{
   TString dir = getenv("VMCWORKDIR");
   if (dir.IsNull()) {
      std::cerr << "ERROR: VMCWORKDIR not set. Run 'source build/config.sh' first.\n";
      return;
   }

   TString geoFile = dir + "/geometry/ATTPC_He1bar_geomanager.root";
   TString outputFile = "./data/simpleSim_Bfield.root";
   TString elossFile = dir + "/resources/energy_loss/HinH.txt";

   // ---- FairRunAna (no Geant4) ------------------------------------------
   FairRunAna *run = new FairRunAna();
   run->SetOutputFile(outputFile);

   // ---- Build the simulation --------------------------------------------
   auto sim = std::make_unique<AtSimpleSimulation>(geoFile.Data());

   // Proton energy-loss in He gas (SRIM table, H in H approximation for He)
   auto eloss = std::make_shared<AtTools::AtELossTable>();
   eloss->LoadSrimTable(elossFile.Data());
   sim->AddModel(1, 1, eloss);

   // 2 T solenoidal field along Z (beam axis).
   // Particles with transverse momentum will spiral; a purely longitudinal beam
   // proton travels straight.  Use a reaction generator for curved tracks.
   sim->SetMagneticField({0., 0., 2.0}); // T

   // ---- FairTask wrapper ------------------------------------------------
   auto *simTask = new AtTestSimulation(std::move(sim));

   // ---- Generator: 1 MeV proton along Z ------------------------------------
   // FairBoxGenerator works with FairRunAna (no Geant4/FairRunSim required).
   // PDG 2212 = proton.  Momentum is given in GeV/c.
   //   KE = 1 MeV proton: E = 938.272 + 1 = 939.272 MeV
   //                       p = sqrt(939.272² - 938.272²) ≈ 43.33 MeV/c = 0.04333 GeV/c
   // Range ≈ 203 mm (from HinH.txt) — stops well within the 1000 mm drift volume.
   auto *primGen = new FairPrimaryGenerator();
   auto *boxGen = new FairBoxGenerator(2212 /*proton PDG*/, 1 /*multiplicity*/);
   boxGen->SetPRange(0.04333, 0.04333);  // fixed |p| in GeV/c
   boxGen->SetPhiRange(0., 0.);          // phi = 0  → momentum in XZ plane
   boxGen->SetThetaRange(0., 0.);        // theta = 0 → along +Z
   // drift_volume is a tube at (0, 6.079, 50) cm with r=25 cm, half-length=50 cm → z: 0–100 cm
   boxGen->SetXYZ(0., 6.079, 1.);        // start 1 cm inside the window at beam axis
   primGen->AddGenerator(boxGen);
   simTask->SetPrimaryGenerator(primGen);

   run->AddTask(simTask);

   // ---- Run ---------------------------------------------------------------
   TStopwatch timer;
   run->Init();
   timer.Start();
   run->Run(0, nEvents);
   timer.Stop();

   std::cout << "\nMacro finished. Output: " << outputFile << "\n";
   std::cout << "Real time: " << timer.RealTime() << " s,  CPU time: " << timer.CpuTime() << " s\n";
}
