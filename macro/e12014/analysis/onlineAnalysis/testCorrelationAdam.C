void findEvents(int tpc_run_num = 118)
{
   TChain tpc_tree("cbmsim");
   tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", tpc_run_num));

   TTreeReader reader(&tpc_tree);
   TTreeReaderValue<TClonesArray> ransac(reader, "ATRansac");

   std::ofstream events_file("findEvents_" + std::to_string(tpc_run_num) + ".dat");

   TH1F *vertex_z = new TH1F("vertex_z", ";z (mm;", 10, 0, 900);
   TH2F *hAngle = new TH2F("hAngle", "Angular Distro Of Fragments", 50, 200, 1100, 60, 0, 60);

   int tpc_nevents = tpc_tree.GetEntries();
   int numFission = 0;
   int numVertex = 0;
   for (int i = 0; i < tpc_nevents; ++i) {
      reader.Next();
      if (ransac->GetEntries() > 0) {
         auto atransac = (ATRANSACN::ATRansac *)(ransac->At(0));
         auto ransac_ary = atransac->GetTrackCand();

         std::vector<int> validTrackIndex;
         for (int i = 0; i < ransac_ary.size(); ++i) {
            double angle = ransac_ary[i].GetAngleZAxis() * TMath::RadToDeg();
            if (2 < angle && ransac_ary[i].GetHitArray()->size() > 30)
               validTrackIndex.push_back(i);
         }

         if (validTrackIndex.size() > 1)
            numFission++;

         // Find the vertex Source: https://en.wikipedia.org/wiki/Skew_lines
         if (validTrackIndex.size() == 2) {
            // Get four vectors describing the lines
            TVector3 p1(ransac_ary[validTrackIndex[0]].GetFitPar()[0], ransac_ary[validTrackIndex[0]].GetFitPar()[2],
                        0);
            TVector3 d1(ransac_ary[validTrackIndex[0]].GetFitPar()[1], ransac_ary[validTrackIndex[0]].GetFitPar()[3],
                        1);
            TVector3 p2(ransac_ary[validTrackIndex[1]].GetFitPar()[0], ransac_ary[validTrackIndex[1]].GetFitPar()[2],
                        0);
            TVector3 d2(ransac_ary[validTrackIndex[1]].GetFitPar()[1], ransac_ary[validTrackIndex[1]].GetFitPar()[3],
                        1);
            TVector3 n = d1.Cross(d2);
            TVector3 n1 = d1.Cross(n);
            TVector3 n2 = d2.Cross(n);

            TVector3 c1 = p1 + (p2 - p1).Dot(n2) / d1.Dot(n2) * d1;
            TVector3 c2 = p2 + (p1 - p2).Dot(n1) / d2.Dot(n1) * d2;

            auto vertex = (c1.Z() + c2.Z()) / 2.0;
            if (vertex < 1000) {
               vertex_z->Fill(vertex);
               numVertex++;
            }

            // Get the angle between vectors
            TVector3 u1 = p1 + d1 - c1;
            TVector3 u2 = p2 + d2 - c2;
            hAngle->Fill(vertex, u1.Angle(u2) * TMath::RadToDeg());

            events_file << reader.GetCurrentEntry() << endl;
         }

      } // End loop over ranscal

   } // end loop over events

   std::cout << "Found " << numFission << " in " << tpc_nevents << std::endl;
   std::cout << "Found " << numVertex << " in " << numFission << std::endl;

   TCanvas *c = new TCanvas("c1");
   vertex_z->Draw("hist");
   TCanvas *c2 = new TCanvas("c2");
   hAngle->Draw("colz");
   //  gPad -> WaitPrimitive("CUTG", "[CUTG]");
}

void testCorrelation(int nscl_run_num = 310, int tpc_run_num = 118)
{
   TChain nscl_tree("tr");
   nscl_tree.Add(TString::Format("/mnt/analysis/e12014/HiRAEVT/Calibrated/run-%02d-aligned.root", nscl_run_num));
   TChain tpc_tree("cbmsim");
   tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", tpc_run_num));

   TTreeReader reader(&nscl_tree);
   TTreeReaderValue<TClonesArray> eventArray(reader, "ATEventH");
   TTreeReaderValue<std::vector<double>> beam_ic(reader, "IonCb_dE");

   TGraph timeVsRatio; //, hist;//
   TH2F hist("hist", ";ADC Ch 2;Aux channel max.", 100, 2000, 3000, 100, 700, 1200);
   int nscl_nevents = nscl_tree.GetEntries();
   int tpc_nevents = tpc_tree.GetEntries();
   nscl_tree.AddFriend(&tpc_tree);
   cout << nscl_nevents << " " << tpc_nevents << endl;
   for (int i = 0; i < ((nscl_nevents > tpc_nevents) ? tpc_nevents : nscl_nevents); ++i) {
      reader.Next();
      auto event = (ATEvent *)eventArray->At(0);
      auto auxPadArray = event->GetAuxPadArray();
      for (auto auxpad : *auxPadArray) {
         if (auxpad.GetAuxName().compare(std::string("IonCb_34")) == 0) {
            auto adc = auxpad.GetADC();
            auto max = std::max_element(adc, adc + 512);
            if (beam_ic->size() > 0) {
               hist.Fill(beam_ic->at(2), *max);
               // hist.SetPoint(hist.GetN(), beam_ic->at(2), *max);
               timeVsRatio.SetPoint(timeVsRatio.GetN(), i, beam_ic->at(2) / (*max));
            }
         }
      }
      std::cout << "Event " << i << "\r" << std::flush;
   }
   std::cout << std::endl;
   TCanvas c1;
   hist.Draw("colz");
   TCanvas c2;
   timeVsRatio.Draw("AP");
   gPad->WaitPrimitive("CUTG", "[CUTG]");
}

void testCorrelationTracks(int nscl_run_num = 310, int tpc_run_num = 118)
{
   TChain nscl_tree("tr");
   nscl_tree.Add(TString::Format("/mnt/analysis/e12014/HiRAEVT/Calibrated/run-%02d-aligned.root", nscl_run_num));
   TChain tpc_tree("cbmsim");
   tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", tpc_run_num));

   TTreeReader reader(&nscl_tree);
   TTreeReaderValue<TClonesArray> ransac(reader, "ATRansac");
   TTreeReaderValue<TVector3> beam_ic(reader, "BeamOnATTPC");
   // TTreeReaderValue<double> beam_ic(reader, "Truncated_dE_to_U6");

   TH2F hist("hist", "", 5000, 0, 90, 500, 0, 10);
   int nscl_nevents = nscl_tree.GetEntries();
   int tpc_nevents = tpc_tree.GetEntries();
   nscl_tree.AddFriend(&tpc_tree);
   for (int i = 0; i < ((nscl_nevents > tpc_nevents) ? tpc_nevents : nscl_nevents); ++i) {
      reader.Next();
      if (ransac->GetEntries() > 0) {
         auto ransac_ary = ((ATRANSACN::ATRansac *)(ransac->At(0)))->GetTrackCand();
         // if(ransac_ary.size() == 1)
         {
            double sum_energy = 0;
            for (int i = 0; i < ransac_ary.size(); ++i) {
               // std::cout << ransac_ary[i].GetGeoQEnergy() << std::endl;
               auto parFit = ransac_ary[i].GetFitPar();
               // calculate beam angle
               // if(ransac_ary[i].GetAngleZAxis()*TMath::RadToDeg() < 10) sum_energy += ransac_ary[i].GetGeoQEnergy();
               std::cout << beam_ic->Theta() * TMath::RadToDeg() << std::endl;
               hist.Fill(beam_ic->Theta() * TMath::RadToDeg(),
                         ransac_ary[i].GetAngleZAxis() * TMath::RadToDeg()); //, ransac_ary[i].GetGeoQEnergy());
            }
            // hist.Fill(*beam_ic, sum_energy);
         }
      }
      std::cout << "Event " << i << "\r" << std::flush;
   }
   std::cout << std::endl;
   hist.Draw("colz");
   gPad->WaitPrimitive("CUTG", "[CUTG]");
}
