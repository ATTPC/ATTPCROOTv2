
void run_eve()
{
   std::cout << " Hi. " << std::endl;
   TString InputDataPath = "./output_digi_rcnp_13Be_p_18.0_18.0_hole_pdt3He4He12C_4mm.root";
   TString OutputDataPath = "./output.reco_display.root";
   std::cout << "Opening: " << InputDataPath << std::endl;

   TString attpcrootPath = gSystem->Getenv("VMCWORKDIR");
   TString geoFile = "ATTPC_C3D8_450torr_geomanager.root";
   TString mapFile = "Lookup20150611.xml";
   TString GeoDataPath = attpcrootPath + "/geometry/" + geoFile;
   TString mapDir = attpcrootPath + "/scripts/" + mapFile;

   FairRunAna *fRun = new FairRunAna();
   FairRootFileSink *sink = new FairRootFileSink(OutputDataPath);
   FairFileSource *source = new FairFileSource(InputDataPath);
   fRun->SetSource(source);
   fRun->SetSink(sink);
   fRun->SetGeomFile(GeoDataPath);

   FairRuntimeDb *rtdb = fRun->GetRuntimeDb();
   FairParRootFileIo *parIo = new FairParRootFileIo();
   rtdb->setFirstInput(parIo);

   auto fMap = std::make_shared<AtTpcMap>();
   fMap->ParseXMLMap(mapDir.Data());
   AtViewerManager *eveMan = new AtViewerManager(fMap);

   auto tabMain = std::make_unique<AtTabMain>();
   tabMain->SetMultiHit(100);

   auto fitter = std::make_unique<AtFITTER::At3DBraggFitter>();
   // Proton
   fitter->AddAZPair({1, 1});
   fitter->AddMassUma(1.007825031898);
   // Deuteron
   fitter->AddAZPair({2, 1});
   fitter->AddMassUma(2.014101777844);
   // Triton
   fitter->AddAZPair({3, 1});
   fitter->AddMassUma(3.01604928132);
   // 3He
   fitter->AddAZPair({3, 2});
   fitter->AddMassUma(3.01602932197);
   // 4He
   fitter->AddAZPair({4, 2});
   fitter->AddMassUma(4.00260325413);
   // 12C
   fitter->AddAZPair({12, 6});
   fitter->AddMassUma(12.);

   fitter->SetMaterialDensity(1.28e-3); // g/cm3
   fitter->AddMaterialComponent({12, 6, 3});
   fitter->AddMaterialComponent({2, 1, 8});

   fitter->SetDZ(4.); // mm
   fitter->SetMinKE(1.); // MeV
   fitter->SetMaxKE(5.); // MeV
   fitter->SetKEGuess(3.); // MeV

   fitter->Init();

   auto tab3DBraggCurve = std::make_unique<AtTab3DBraggCurve>();
   tab3DBraggCurve->SetMultiHit(100);
   tab3DBraggCurve->SetDZ(4.);
   tab3DBraggCurve->Set3DBraggCurveFitter(std::move(fitter));

   eveMan->AddTab(std::move(tabMain));
   eveMan->AddTab(std::move(tab3DBraggCurve));

   eveMan->Init();

   std::cout << " Finished init. " << std::endl;
}
