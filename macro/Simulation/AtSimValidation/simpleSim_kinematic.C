#include "AtCave.h"
#include "AtSimParticleCollector.h"
#include "AtSimpleSimulation.h"
#include "AtTPC2Body.h"
#include "AtTPCIonGenerator.h"
#include "AtTpc.h"

#include <FairGenerator.h>
#include <FairMCEventHeader.h>
#include <FairParRootFileIo.h>
#include <FairPrimaryGenerator.h>
#include <FairRootFileSink.h>
#include <FairRunSim.h>
#include <FairTask.h>

#include <TDatabasePDG.h>
#include <TParticlePDG.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace {
std::pair<int, int> GetZAFromPDG(int pdg)
{
   if (pdg > 1000000000) {
      int A = (pdg / 10) % 1000;
      int Z = (pdg / 10000) % 1000;
      return {Z, A};
   }

   auto *particle = TDatabasePDG::Instance()->GetParticle(pdg);
   if (particle) {
      int Z = static_cast<int>(std::round(particle->Charge() / 3.0));
      int A = static_cast<int>(std::round(particle->Mass() / 0.9315));
      return {Z, std::max(A, 1)};
   }

   return {0, 0};
}

FairPrimaryGenerator *BuildElasticGenerator(Double_t thetaMinCmsDeg, Double_t thetaMaxCmsDeg)
{
   constexpr Int_t z = 1;
   constexpr Int_t a = 1;
   constexpr Int_t q = 0;
   constexpr Int_t m = 1;
   constexpr Double_t px = 0.0;
   constexpr Double_t py = 0.0;
   constexpr Double_t pz = 0.04333;
   constexpr Double_t beamExcitation = 0.0;
   constexpr Double_t beamMass = 0.938272;
   constexpr Double_t nominalEnergy = 1.0;

   auto *primGen = new FairPrimaryGenerator();

   auto *ionGen =
      new AtTPCIonGenerator("Ion", z, a, q, m, px, py, pz, beamExcitation, beamMass, nominalEnergy);
   ionGen->SetSpotRadius(0, -100, 0);
   ionGen->SetDoReaction(kTRUE);
   primGen->AddGenerator(ionGen);

   std::vector<Int_t> Zp{1, 2, 1, 2};
   std::vector<Int_t> Ap{1, 4, 1, 4};
   std::vector<Int_t> Qp{0, 0, 0, 0};
   std::vector<Double_t> Pxp{px, 0.0, 0.0, 0.0};
   std::vector<Double_t> Pyp{py, 0.0, 0.0, 0.0};
   std::vector<Double_t> Pzp{pz, 0.0, 0.0, 0.0};
   std::vector<Double_t> Mass{1.007276, 4.00260, 1.007276, 4.00260};
   std::vector<Double_t> ExE{beamExcitation, 0.0, 0.0, 0.0};

   constexpr Int_t mult = 4;
   constexpr Double_t resEnergy = 1.0;
   auto *twoBody = new AtTPC2Body("Elastic", &Zp, &Ap, &Qp, mult, &Pxp, &Pyp, &Pzp, &Mass, &ExE, resEnergy,
                                  thetaMinCmsDeg, thetaMaxCmsDeg);
   primGen->AddGenerator(twoBody);

   return primGen;
}

void ConfigureHeLossModels(AtSimpleSimulation &sim)
{
   constexpr double heDensity = 1.664e-4;
   std::vector<std::tuple<int, int, int>> material{{4, 2, 1}};

   auto protonModel = std::make_shared<AtTools::AtELossCATIMA>(heDensity, material);
   protonModel->SetProjectile(1, 1, 1.007276);
   sim.AddModel(1, 1, protonModel, 1.007276);

   auto heliumModel = std::make_shared<AtTools::AtELossCATIMA>(heDensity, material);
   heliumModel->SetProjectile(4, 2, 4.002602);
   sim.AddModel(2, 4, heliumModel, 4.002602);
}

class SimpleSimTask : public FairTask {
public:
   explicit SimpleSimTask(FairPrimaryGenerator *primGen) : FairTask("SimpleSimTask"), fPrimGen(primGen) {}

   InitStatus Init() override
   {
      fSimulation = std::make_unique<AtSimpleSimulation>();
      ConfigureHeLossModels(*fSimulation);
      fSimulation->SetMagneticField(ROOT::Math::XYZVector(0., 0., 2.0));
      fSimulation->SetMaxPropagationStep(1e-3);
      fSimulation->RegisterBranch();

      if (fPrimGen) {
         fMCHeader = std::make_unique<FairMCEventHeader>();
         fPrimGen->SetEvent(fMCHeader.get());
         fPrimGen->Init();
      }

      return kSUCCESS;
   }

   void Exec(Option_t *) override
   {
      fSimulation->NewEvent();
      if (!fPrimGen)
         return;

      fCollector.Clear();
      fPrimGen->GenerateEvent(&fCollector);

      for (const auto &particle : fCollector.GetParticles()) {
         auto [Z, A] = GetZAFromPDG(particle.pdgCode);
         if (Z == 0 && A == 0)
            continue;

         ROOT::Math::XYZPoint pos(particle.vx * 10., particle.vy * 10., particle.vz * 10.);
         ROOT::Math::PxPyPzEVector mom(particle.px * 1000., particle.py * 1000., particle.pz * 1000.,
                                       particle.e * 1000.);

         try {
            fSimulation->SimulateParticle(Z, A, pos, mom);
         } catch (const std::invalid_argument &) {
         }
      }
   }

private:
   std::unique_ptr<AtSimpleSimulation> fSimulation;
   FairPrimaryGenerator *fPrimGen{};
   AtSimParticleCollector fCollector;
   std::unique_ptr<FairMCEventHeader> fMCHeader;
};
} // namespace

void simpleSim_kinematic(Int_t nEvents = 1000, UInt_t seed = 42)
{
   TString dir = gSystem->Getenv("VMCWORKDIR");
   if (dir.IsNull()) {
      std::cerr << "VMCWORKDIR is not set. Run 'source build/config.sh' first.\n";
      return;
   }

   gSystem->mkdir("data", kTRUE);
   gSystem->Setenv("GEOMPATH", (dir + "/geometry").Data());
   gRandom->SetSeed(seed);

   TString outputFile = "./data/simpleSim_kinematic.root";
   TString parFile = "./data/simpleSim_kinematic_params.root";

   TStopwatch timer;
   timer.Start();

   auto *run = new FairRunSim();
   run->SetName("TGeant3");
   run->SetSink(new FairRootFileSink(outputFile));
   run->SetMaterials("media.geo");

   auto *cave = new AtCave("CAVE");
   cave->SetGeometryFileName("caveSmall.geo");
   run->AddModule(cave);

   auto *tpc = new AtTpc("ATTPC", kTRUE);
   tpc->SetGeometryFileName((dir + "/geometry/ATTPC_He1bar.root").Data());
   run->AddModule(tpc);

   run->SetGenerator(new FairPrimaryGenerator());

   auto *simPrimGen = BuildElasticGenerator(0.0, 180.0);
   run->AddTask(new SimpleSimTask(simPrimGen));

   auto *rtdb = run->GetRuntimeDb();
   Bool_t parameterMerged = kTRUE;
   auto *parOut = new FairParRootFileIo(parameterMerged);
   parOut->open(parFile.Data());
   rtdb->setOutput(parOut);

   run->Init();
   run->Run(nEvents);
   rtdb->saveOutput();

   timer.Stop();
   std::cout << "Wrote " << outputFile << "\n";
   std::cout << "Real time " << timer.RealTime() << " s, CPU time " << timer.CpuTime() << " s\n";
}
