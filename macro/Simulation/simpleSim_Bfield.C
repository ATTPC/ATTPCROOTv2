/**
 * simpleSim_Bfield.C
 *
 * Standalone AT-TPC simulation using AtTestSimulation (no Geant4).
 * Fires 50 MeV protons into a 2 T solenoid field along Z.
 *
 * Expected physics:
 *   Proton with p_z ≈ 310.5 MeV/c in B = 2 T along Z will spiral around Z with
 *   Larmor radius r = p⊥ / (q B).  Because the initial momentum is purely along Z
 *   (parallel to B), p⊥ = 0 and there is no Larmor bending — the proton travels in a
 *   straight line along Z while decelerating.  To see curvature, use a reaction
 *   generator (e.g. AtTPC2Body) that produces particles with transverse momentum, or
 *   tilt the beam angle via ionGen->SetBeamAngle().
 *
 * Output: ./data/simpleSim_Bfield.root
 *   Tree "cbmsim", branch "AtTpcPoint" (TClonesArray of AtMCPoint).
 *   Open with ROOT browser or compareSimVsGeant.C.
 *
 * Usage:
 *   source build/config.sh
 *   root -l -q 'macro/Simulation/simpleSim_Bfield.C(100)'
 */

#include "AtSimpleSimulation.h"
#include "AtTestSimulation.h"
#include "AtTPCIonGenerator.h"

#include <FairParAsciiFileIo.h>
#include <FairPrimaryGenerator.h>
#include <FairRunAna.h>
#include <FairRuntimeDb.h>

#include <TStopwatch.h>
#include <TString.h>

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

   // ---- Generator: 50 MeV proton along Z --------------------------------
   // Momentum components are in GeV/c PER NUCLEON.
   // p_z for KE = 50 MeV proton:
   //   E = m + KE = 938.272 + 50 = 988.272 MeV
   //   p = sqrt(E² - m²) ≈ 310.5 MeV/c  →  0.3105 GeV/c per nucleon (A=1)
   const Double_t pz_GeV = 0.3105;
   const Double_t mass_GeV = 0.938272;
   const Double_t ener_MeV = 50.0;

   auto *primGen = new FairPrimaryGenerator();
   auto *ionGen = new AtTPCIonGenerator("proton", /*z=*/1, /*a=*/1, /*q=*/1, /*mult=*/1,
                                        /*px=*/0.0, /*py=*/0.0, /*pz=*/pz_GeV,
                                        /*Ex=*/0.0, /*m=*/mass_GeV, /*ener=*/ener_MeV);
   // Start beam at upstream face of the detector (z = -50 cm in detector coords)
   ionGen->SetSpotRadius(0, -50., 0.);
   primGen->AddGenerator(ionGen);
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
