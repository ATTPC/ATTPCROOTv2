void plotBeamAngle(int run_num = 118)
{
   TChain tpc_tree("cbmsim");
   tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", run_num));

   TTreeReader reader(&tpc_tree);
   TTreeReaderValue<TClonesArray> ransac(reader, "ATRansac");

   TH2F hist("hist", "", 61, 0, 60, 1000, 0, 90);
   while (reader.Next()) {
      auto ransac_ary = ((ATRANSACN::ATRansac *)(ransac->At(0)))->GetTrackCand();
      for (auto &track : ransac_ary)
         hist.Fill(track.GetHitArray()->size(), track.GetAngleZAxis() * TMath::RadToDeg());
   }
   hist.Draw("colz");
   gPad->WaitPrimitive();
}

void plotBeamEnergyLost(int nscl_run_num = 310, int tpc_run_num = 118)
{
   TChain tpc_tree("cbmsim");
   tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", tpc_run_num));
   tpc_tree.AddFriend("tr",
                      TString::Format("/mnt/analysis/e12014/HiRAEVT/Calibrated/run-%02d-aligned.root", nscl_run_num));

   TTreeReader reader(&tpc_tree);
   TTreeReaderValue<TClonesArray> raw_event(reader, "ATRawEvent");
   TTreeReaderValue<double> IC_dE(reader, "Truncated_dE_to_U6");
   TTreeReaderValue<double> DS(reader, "DS");
   TTreeReaderValue<double> Tof(reader, "DTof");

   // get cuts;
   std::vector<TCutG *> cuts;
   TFile cuts_file("IC_cuts_run_400.root");
   cuts.push_back((TCutG *)cuts_file.Get("CUTG1"));
   cuts.push_back((TCutG *)cuts_file.Get("CUTG2"));
   cuts.push_back((TCutG *)cuts_file.Get("CUTG3"));

   const int num_tb = 512;
   std::vector<TH1F> hists;
   TH2F PID("PID", ";ToF (ns);dE - Ion chamber", 101, -440, -430, 100, 8, 13);
   for (int i = 0; i < cuts.size(); ++i)
      hists.emplace_back(TString::Format("hist%d;Time bucket;ADC signal all pads", i), "", num_tb + 1, 0, num_tb);
   std::vector<bool> hist_status(cuts.size(), false);

   while (reader.Next()) {
      if (*DS == -9999)
         continue;
      // Draw PID
      PID.Fill(*Tof, *IC_dE);
      // find which cut does the event corresponds to
      int id;
      for (id = 0; id < cuts.size(); ++id)
         if (cuts[id]->IsInside(*Tof, *IC_dE)) {
            hist_status[id] = true;
            break;
         }
      if (id == cuts.size())
         continue; // if search reaches the end
      auto &hist = hists[id];

      auto pads = static_cast<ATRawEvent *>(raw_event->At(0))->GetPads();
      for (auto &pad : *pads) {
         auto adc = pad.GetRawADC();
         for (int i = 0; i < num_tb; ++i)
            hist.Fill(i, adc[i]);
      }

      bool all_true = true;
      for (auto status : hist_status)
         all_true = all_true && status;
      // if(all_true) break;
      std::cout << "Entry: " << reader.GetCurrentEntry() << "\r" << std::flush;
   }

   // draw results
   TCanvas c1, c2;
   c2.cd();
   PID.Draw("colz");
   for (int i = 0; i < hists.size(); ++i) {
      c1.cd();
      hists[i].SetLineColor(i + 2);
      // normalize the histogram
      hists[i].Scale(1. / hists[i].GetEntries()); // Integral(1, num_tb));
      hists[i].Draw((i == 0) ? "hist" : "hist same");
      c2.cd();
      cuts[i]->SetLineColor(i + 2);
      cuts[i]->Draw("l same");
   }
   gPad->WaitPrimitive("CUTG", "CUTG");
}

void plotBeamEnergyVsIC(int nscl_run_num = 310, int tpc_run_num = 118)
{
   TChain tpc_tree("cbmsim");
   tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpacked/run_%04d.root", tpc_run_num));
   tpc_tree.AddFriend("tr",
                      TString::Format("/mnt/analysis/e12014/HiRAEVT/Calibrated/run-%02d-aligned.root", nscl_run_num));

   TTreeReader reader(&tpc_tree);
   TTreeReaderValue<TClonesArray> raw_event(reader, "ATRawEvent");
   TTreeReaderValue<TClonesArray> ransac(reader, "ATRansac");
   // TTreeReaderValue<std::vector<double>> IC_dE(reader, "CalibratedE_abs_U6");
   TTreeReaderValue<double> mean_dE(reader, "Truncated_dE_to_U6");
   TTreeReaderValue<double> Tof(reader, "DTof");
   TTreeReaderValue<double> DS(reader, "DS");

   const int num_tb = 512;
   TH2F energy_correlation("energy_correlation", "", 500, 0, -20, 3000, 0, 300000);

   double last_tof = 0;
   while (reader.Next()) {
      if (*DS == -9999)
         continue;
      auto atransac = (ATRANSACN::ATRansac *)(ransac->At(0));
      auto ransac_ary = atransac->GetTrackCand();

      // int num_valid_tracks = 0;
      // for(int i = 0; i < ransac_ary.size(); ++i)
      //{
      //  double angle = ransac_ary[i].GetAngleZAxis()*TMath::RadToDeg();
      //  if(2 < angle && ransac_ary[i].GetHitArray()->size() > 30)
      //    ++num_valid_tracks;
      //}
      // if(num_valid_tracks == 2)
      {
         // Draw PID
         auto pads = static_cast<ATRawEvent *>(raw_event->At(0))->GetPads();
         double adc_sum = 0;

         for (auto &pad : *pads) {
            auto adc = pad.GetRawADC();
            for (int i = 120; i < num_tb - 50; ++i)
               adc_sum += adc[i];
         }
         /*if(IC_dE -> size() > 0)*/ energy_correlation.Fill(*mean_dE, adc_sum / 512.);

         // bool all_true = true;
         // for(auto status : hist_status) all_true = all_true && status;
         // if(all_true) break;
      }
      std::cout << "Entry: " << reader.GetCurrentEntry() << "\r" << std::flush;
   }

   // draw results
   TCanvas c1;
   energy_correlation.Draw("colz");
   gPad->WaitPrimitive("CUTG", "CUTG");
}
