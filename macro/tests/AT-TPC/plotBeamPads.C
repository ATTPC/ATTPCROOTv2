void plotBeamPads()
{
   // Construct an AtTpcMap object, parse the mapping and parse the beam pad list.
   TString dir = getenv("VMCWORKDIR");
   //TString scriptFile = "rcnp_map_size.xml";
   TString scriptFile = "e23031_pad_map_size.xml";
   TString mapDir = dir + "/scripts/" + scriptFile;
   //TString beamPadsFile = "BeamPads_RCNP.csv";
   TString beamPadsFile = "BeamPads_e23031.csv";
   TString beamPadsDir = dir + "/scripts/" + beamPadsFile;
   AtTpcMap *map = new AtTpcMap();
   map->ParseXMLMap(mapDir.Data());
   map->InhibitBeamPads(beamPadsDir);

   // Generate padplane and get the TH2Poly from it.
   map->GeneratePadPlane();
   TH2Poly *padPlane = map->GetPadPlane();

   // Check all pads and count how many are actually inhibited.
   Int_t actuallyInhibitedPadsCount{};
   for (int i = 0; i < map->GetNumPads(); i++)
      if (map->IsInhibited(i) == AtMap::InhibitType::kXTalk) {
         auto position = map->CalcPadCenter(i);
         padPlane->Fill(position.X(), position.Y(), 1);
   }
   // Create TCanvas and draw the padplane.
   TCanvas *cPadPlane = new TCanvas();
   padPlane->Draw("COL L1");
   padPlane->SetMinimum(1.0);
   padPlane->GetXaxis()->SetTitle("x [mm]");
   padPlane->GetYaxis()->SetTitle("y [mm]");
}
