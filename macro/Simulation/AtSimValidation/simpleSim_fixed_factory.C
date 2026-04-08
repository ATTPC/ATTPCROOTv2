// SimpleSim drop-in replacement for geant4_fixed.C using factory-based energy loss.
// Compare the diff between this file and geant4_fixed.C to see what changes.

#include <AtSimpleSimulation.h>
#include <AtSimpleSimulationTask.h>

#include <AtELossFactoryCATIMA.h>

namespace {
FairPrimaryGenerator *BuildElasticGenerator(Double_t thetaMinCmsDeg, Double_t thetaMaxCmsDeg)
{
   constexpr Int_t z = 6;
   constexpr Int_t a = 16;
   constexpr Int_t q = 0;
   constexpr Int_t m = 1;
   constexpr Double_t px = 0.0;
   constexpr Double_t py = 0.0;
   constexpr Double_t pz = 2.297 / a;
   constexpr Double_t beamExcitation = 0.0;
   constexpr Double_t beamMass = 16.014701;
   constexpr Double_t nominalEnergy = 0.0;

   auto *primGen = new FairPrimaryGenerator();

   auto *ionGen =
      new AtTPCIonGenerator("Ion", z, a, q, m, px, py, pz, beamExcitation, beamMass, nominalEnergy);
   ionGen->SetSpotRadius(0, -100, 0);
   ionGen->SetDoReaction(kTRUE);
   primGen->AddGenerator(ionGen);

   std::vector<Int_t> Zp{6, 1, 6, 1};
   std::vector<Int_t> Ap{16, 1, 16, 1};
   std::vector<Int_t> Qp{0, 0, 0, 0};
   std::vector<Double_t> Pxp{px, 0.0, 0.0, 0.0};
   std::vector<Double_t> Pyp{py, 0.0, 0.0, 0.0};
   std::vector<Double_t> Pzp{pz, 0.0, 0.0, 0.0};
   std::vector<Double_t> Mass{16.014701, 1.0078250322, 16.014701, 1.0078250322};
   std::vector<Double_t> ExE{beamExcitation, 0.0, 0.0, 0.0};

   constexpr Int_t mult = 4;
   constexpr Double_t resEnergy = 40.0;
   auto *twoBody = new AtTPC2Body("Elastic", &Zp, &Ap, &Qp, mult, &Pxp, &Pyp, &Pzp, &Mass, &ExE, resEnergy,
                                  thetaMinCmsDeg, thetaMaxCmsDeg);
   primGen->AddGenerator(twoBody);

   return primGen;
}
} // namespace

void simpleSim_fixed_factory(Double_t thetaCms = 45.0, Int_t nEvents = 100, UInt_t seed = 42)
{
   TString dir = gSystem->Getenv("VMCWORKDIR");
   if (dir.IsNull()) {
      std::cerr << "VMCWORKDIR is not set. Run 'source build/config.sh' first.\n";
      return;
   }

   gSystem->mkdir("data", kTRUE);
   gSystem->Setenv("GEOMPATH", (dir + "/geometry").Data());
   gRandom->SetSeed(seed);

   TString outFile = "./data/simpleSim_fixed_factory.root";
   TString parFile = "./data/simpleSim_fixed_factory_params.root";

   TStopwatch timer;
   timer.Start();

   auto *run = new FairRunSim();
   run->SetName("TGeant3");
   run->SetSink(new FairRootFileSink(outFile));
   run->SetMaterials("media.geo");
   auto *rtdb = run->GetRuntimeDb();

   auto *cave = new AtCave("CAVE");
   cave->SetGeometryFileName("cave.geo");
   run->AddModule(cave);

   auto *tpc = new AtTpc("ATTPC", kTRUE);
   tpc->SetGeometryFileName("ATTPC_He1bar.root");
   run->AddModule(tpc);

   auto *magField = new AtConstField();
   magField->SetField(0., 0., 20.);
   magField->SetFieldRegion(-50., 50., -50., 50., -10., 110.);
   run->SetField(magField);

   // --- SimpleSim drop-in: replace Geant4 transport with factory-based SimpleSim ---
   run->SetGenerator(new FairPrimaryGenerator());

   auto sim = std::make_unique<AtSimpleSimulation>();
   sim->SetModelFactory(std::make_shared<AtTools::AtELossFactoryCATIMA>());

   auto *simTask = new AtSimpleSimulationGeneratorTask(std::move(sim));
   simTask->SetPrimaryGenerator(BuildElasticGenerator(thetaCms, thetaCms));
   simTask->SetDetector(tpc);
   run->AddTask(simTask);

   run->SetStoreTraj(kFALSE);
   run->Init();

   Bool_t parameterMerged = kTRUE;
   auto *parOut = new FairParRootFileIo(parameterMerged);
   parOut->open(parFile.Data());
   rtdb->setOutput(parOut);
   rtdb->saveOutput();

   run->Run(nEvents);

   timer.Stop();
   std::cout << "Wrote " << outFile << "\n";
   std::cout << "Real time " << timer.RealTime() << " s, CPU time " << timer.CpuTime() << " s\n";
}
