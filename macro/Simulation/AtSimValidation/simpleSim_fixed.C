
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

std::unique_ptr<AtSimTransport> BuildSimpleSimulation(const TString &geoFile)
{
   auto manager = std::make_shared<AtTools::AtELossManager>();

   constexpr double heDensity = 1.664e-4;
   std::vector<std::tuple<int, int, int>> material{{4, 2, 1}};

   auto carbonModel = std::make_shared<AtTools::AtELossCATIMA>(heDensity, material);
   carbonModel->SetProjectile(16, 6, 16.014701);
   manager->AddModel(6, 16, carbonModel);

   auto protonModel = std::make_shared<AtTools::AtELossCATIMA>(heDensity, material);
   protonModel->SetProjectile(1, 1, 1.0078250322);
   manager->AddModel(1, 1, protonModel);

   auto sim = std::make_unique<AtSimTransport>(geoFile.Data(), manager);
   sim->SetMagneticField(ROOT::Math::XYZVector(0., 0., 2.0));
   sim->SetMaxStep(1e-3);

   return sim;
}
} // namespace

void simpleSim_fixed(Double_t thetaCms = 45.0, Int_t nEvents = 100, UInt_t seed = 42,
                     TString geantTruthFile = "./data/geant4_fixed.root")
{
   TString dir = gSystem->Getenv("VMCWORKDIR");
   if (dir.IsNull()) {
      std::cerr << "VMCWORKDIR is not set. Run 'source build/config.sh' first.\n";
      return;
   }

   gSystem->mkdir("data", kTRUE);
   gSystem->Setenv("GEOMPATH", (dir + "/geometry").Data());
   gRandom->SetSeed(seed);

   TString outputFile = "./data/simpleSim_fixed.root";
   TString parFile = "./data/simpleSim_fixed_params.root";

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

   // FairRunSim still needs a generator object to drive the event loop, but the
   // actual physics generator for the SimpleSim path is owned by the SimpleSim task.
   auto *eventLoopDriver = new FairPrimaryGenerator();
   run->SetGenerator(eventLoopDriver);

   auto *simTask = new AtSimTransportReplayTask(BuildSimpleSimulation(dir + "/geometry/ATTPC_He1bar_geomanager.root"));
   simTask->SetPrimaryTrackSource(geantTruthFile.Data());
   simTask->SetDetector(tpc);
   run->AddTask(simTask);

   run->Init();
   auto *rtdb = run->GetRuntimeDb();
   Bool_t parameterMerged = kTRUE;
   auto *parOut = new FairParRootFileIo(parameterMerged);
   parOut->open(parFile.Data());
   rtdb->setOutput(parOut);
   run->Run(nEvents);
   rtdb->saveOutput();

   timer.Stop();
   std::cout << "Wrote " << outputFile << "\n";
   std::cout << "Real time " << timer.RealTime() << " s, CPU time " << timer.CpuTime() << " s\n";
}
