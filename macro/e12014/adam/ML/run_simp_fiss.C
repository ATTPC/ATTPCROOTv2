// Code to take MC tracks and digitize
#include "eventSim.h"

bool reduceFunc(AtRawEvent *evt);

void run_simp_fiss(int runNum = 0)
{

   delete gRandom;
   gRandom = new TRandom3;   
   gRandom->SetSeed(0);

   //************ Things to change ************//
   int Zcn = 83 + 2;
   int Acn = 200 + 4;
   int Zmin = 26;
   int Zmax = 59;

   simEvent::beamZ = 83;
   simEvent::beamA = 200;
   simEvent::beamM = 199.9332;
   simEvent::massFrac = 0.56; //Probably don't change
   simEvent::massDev = 6; //Probably don't change

   simEvent::beamE = 2.70013e+03; //Get from LISE
   simEvent::beamEsig = 1.28122e+02; //Either set to 0 or maintain the same ratio with beamE

   //************ End things to change ************//

   TString inOutDir = "./data/";
   //TString outputFile = inOutDir + "symFissionLg.root";
   TString outputFile = inOutDir + TString::Format("simFission%02d.root", runNum);
   TString geoFile = "ATTPC_v1.1_geomanager.root";
   TString scriptfile = "e12014_pad_mapping.xml";
   TString paramFile = "ATTPC.e12014.par";

   TString dir = getenv("VMCWORKDIR");

   // TString mcFile = "./data/sim_attpc.root";

   // Create the full parameter file paths
   //TString digiParFile = dir + "/parameters/" + paramFile;
   TString GeoDataPath = dir + "/geometry/" + geoFile;
   TString digiParFile = paramFile;
   TString mapParFile = dir + "/scripts/" + scriptfile;

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;

   // ------------------------------------------------------------------------

   // __ Run ____________________________________________
   FairRunAna *fRun = new FairRunAna();
   fRun->SetSink(new FairRootFileSink(outputFile));
   fRun->SetGeomFile(GeoDataPath);

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(digiParFile.Data(), "in");
   rtdb->setFirstInput(parIo1);

   // Create the detector map to pass to the simulation
   auto mapping = std::make_shared<AtTpcMap>();
   mapping->ParseXMLMap(mapParFile.Data());
   mapping->GeneratePadPlane();

   auto sim = std::make_unique<AtSimpleSimulation>(GeoDataPath.Data());
   sim->SetDistanceStep(1);

   auto scModel = std::make_shared<AtLineChargeModel>();
   //scModel->SetBeamLocation({7, -10, 0}, {0, 7, 1000});
   scModel->SetBeamLocation({0, -6, 0}, {10, 0, 1000});
   //scModel->SetBeamLocation({1, -4, 0}, {-0.1197, 13.01, 1000});
   scModel->SetBeamRadius(0);
   sim->SetSpaceChargeModel(scModel);

   // Create and load energy loss models
   std::vector<std::pair<int, int>> ions;
   for (int i = Zmin; i <= Zmax; i++)
      ions.push_back({i, std::round((double)i / Zcn * Acn)});
   for (auto [Z, A] : ions) {
      auto eloss = std::make_shared<AtTools::AtELossTable>();
      eloss->LoadLiseTable(TString::Format("./eLoss/LISE/%d_%d.txt", Z, A).Data(), A, 0);
      sim->AddModel(Z, A, eloss);
   }

      auto beamloss = std::make_shared<AtTools::AtELossTable>();
      beamloss->LoadLiseTable(TString::Format("./eLoss/LISE/%d_%d.txt", 83, 200).Data(), 200, 0);
      sim->AddModel(83, 200, beamloss);

   simEvent::moveSim(std::move(sim));
   simEvent::ions = ions;

   AtMacroTask *simTask = new AtMacroTask();
   simTask->AddInitFunction(simEvent::Init);
   simTask->AddFunction(simEvent::Exec);

   fRun->AddTask(simTask);

   //  __ Init and run ___________________________________

   //Response::Init();
   fRun->Init();


   timer.Start();
   // fRun->Run(0, 20001);
   fRun->Run(0, 10000);
   simEvent::CleanUp();
   timer.Stop();

   std::cout << std::endl << std::endl;
   std::cout << "Macro finished succesfully." << std::endl << std::endl;
   // -----   Finish   -------------------------------------------------------

   Double_t rtime = timer.RealTime();
   Double_t ctime = timer.CpuTime();
   cout << endl;
   cout << "Real time " << rtime << " s, CPU time " << ctime << " s" << endl;
   cout << endl;
   // ------------------------------------------------------------------------
}

bool reduceFunc(AtRawEvent *evt)
{
   if(evt->GetEventID() % 2 == 0)
      return false;
   return (evt->GetNumPads() > 0) && evt->IsGood();
}

