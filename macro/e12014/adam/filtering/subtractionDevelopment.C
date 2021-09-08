/*
  Examine how similar the AGET ch0s in each ASAD.

  For each ASAD, it gets the average of the four ch0s. For the first 10 events, it saves a
  histogram with the four chanels and the average on a single plot.
  One plot for the baseline subtracted, the other for the non-baseline subtracted.

  For each ch0, it also saves a histogram of the difference between the average and that channel
  for each time bucket

 */
const int nAget = 4;
const int nAsad = 4;
const int nCobo = 10;
const int nTb = 512;

// const int nEventsToProcess = std::numeric_limits<int>::max();
const int nEventsToProcess = 100;
const int nEventToPrint = 10;

void filteringTest()
{
   // Get the input tree and reader
   TChain tpc_tree("cbmsim");
   tpc_tree.Add(TString::Format("/mnt/analysis/e12014/TPC/unpackedReduced/run_%04d.root", 210));
   TTreeReader reader(&tpc_tree);
   TTreeReaderValue<TClonesArray> rawEvent(reader, "AtRawEvent");

   // Create pad plane and load map
   TString mapFile = "e12014_pad_mapping.xml"; //"Lookup20150611.xml";
   // Set directories
   TString dir = gSystem->Getenv("VMCWORKDIR");
   TString mapDir = dir + "/scripts/" + mapFile;
   auto fAtMapPtr = new AtTpcMap();
   fAtMapPtr->ParseXMLMap(mapDir.Data());
   fAtMapPtr->GenerateAtTpc();
   auto fPadPlane = fAtMapPtr->GetAtTpcPlane();

   // Create the output file and required histograms
   TFile *outFile = new TFile("./backgroundCheck.root", "RECREATE");

   // Create the histograms to save
   TH1F *diffHists[nCobo][nAsad][nAget];
   TH1F *diffHistsRaw[nCobo][nAsad][nAget];

   for (int cobo = 0; cobo < nCobo; cobo++)
      for (int asad = 0; asad < nAsad; asad++)
         for (int aget = 0; aget < nAget; aget++) {
            diffHists[cobo][asad][aget] = new TH1F(TString::Format("h_%d_%d_%d", cobo, asad, aget),
                                                   TString::Format("%d %d %d", cobo, asad, aget), 200, -50, 50);
            diffHistsRaw[cobo][asad][aget] = new TH1F(TString::Format("hr_%d_%d_%d", cobo, asad, aget),
                                                      TString::Format("%d %d %d", cobo, asad, aget), 200, -50, 50);
         }

   int eventNumber = 0;
   while (reader.Next()) {
      if (eventNumber >= nEventsToProcess)
         break;

      std::cout << "Looking at event: " << eventNumber << std::endl;

      auto eventPtr = (AtRawEvent *)(rawEvent->At(0));
      auto numPads = eventPtr->GetNumPads();

      Double_t sum[nCobo][nAsad][nTb] = {0};
      Int_t sumRaw[nCobo][nAsad][nTb] = {0};

      // Create trace histograms
      TH1F *adcHists[5];
      TH1F *adcSubHists[4];

      if (eventNumber < nEventToPrint) {
         for (int i = 0; i < 4; ++i) {
            adcHists[i] = new TH1F(TString::Format("trace_%d", i), TString::Format("AGET %d", i), 512, 0, 512);
            adcSubHists[i] =
               new TH1F(TString::Format("traceSub_%d", i), TString::Format("Filtered AGET %d", i), 512, 0, 512);
         }
         adcHists[4] = new TH1F("hAvg", "ASAD average", 512, 0, 512);
      }

      for (int i = 0; i < numPads; ++i) {
         // Get the average for each ASAD
         auto padRef = fAtMapPtr->GetPadRef(eventPtr->GetPad(i)->GetPadNum()); //[cobo,asad,aget,ch]

         // If it is a channel zero, add it to the average
         if (padRef[3] == 0)
            for (int tb = 0; tb < nTb; ++tb) {
               sum[padRef[0]][padRef[1]][tb] += eventPtr->GetPad(i)->GetADC(tb);
               sumRaw[padRef[0]][padRef[1]][tb] += eventPtr->GetPad(i)->GetRawADC(tb);
            }
      } // End loop over pads to get ch0s

      // Get the difference between each AGET ch0 and its associated average
      for (int i = 0; i < numPads; ++i) {
         // Look for ch 0 pads
         auto padRef = fAtMapPtr->GetPadRef(eventPtr->GetPad(i)->GetPadNum()); //[cobo,asad,aget,ch]
         if (padRef[3] == 0) {

            // For each time bucket, save the difference between the ch0 and the average
            for (int tb = 0; tb < nTb; ++tb) {
               auto cobo = padRef[0];
               auto asad = padRef[1];
               auto aget = padRef[2];
               diffHists[cobo][asad][aget]->Fill(sum[cobo][asad][tb] / 4.0 - eventPtr->GetPad(i)->GetADC(tb));
               diffHistsRaw[cobo][asad][aget]->Fill(sumRaw[cobo][asad][tb] / 4.0 - eventPtr->GetPad(i)->GetRawADC(tb));

               // if we should record traces, do
               if (eventNumber < nEventToPrint && cobo == 0 && asad == 0) {
                  adcHists[aget]->Fill(tb, eventPtr->GetPad(i)->GetADC(tb));
                  adcSubHists[aget]->Fill(tb, sum[cobo][asad][tb] / 4.0 - eventPtr->GetPad(i)->GetADC(tb));
                  // save the average only once
                  if (aget == 0)
                     adcHists[4]->Fill(tb, sum[cobo][asad][tb] / 4.0);

               } // end if record traces

            } // end loop over timebuckets in ch0
         }    // end if ch0
      }       // End loop over pads to get difference

      // Write the traces to disk and delete the histograms
      if (eventNumber < nEventToPrint) {
         TCanvas cTraces(TString::Format("trace_%d", eventNumber), TString::Format("Traces %d", eventNumber), 1);
         TCanvas cSubTraces(TString::Format("trace_sub_%d", eventNumber),
                            TString::Format("Subtracted Traces %d", eventNumber), 1);

         cTraces.cd();
         for (int i = 0; i < 5; ++i)
            adcHists[i]->Draw("HIST SAME PLC");
         gPad->BuildLegend();
         outFile->cd();
         cTraces.Write();

         cSubTraces.cd();
         for (int i = 0; i < 4; ++i)
            adcSubHists[i]->Draw("HIST SAME PLC");
         gPad->BuildLegend();
         outFile->cd();
         cSubTraces.Write();

         // Delete old histograms
         for (int i = 0; i < 4; ++i) {
            delete adcHists[i];
            delete adcSubHists[i];
         }
         delete adcHists[4];
      }

      eventNumber++;
   } // End loop over all events

   std::cout << "Writing histograms to disk" << std::endl;
   // Write all of the histograms to disk
   for (int cobo = 0; cobo < nCobo; cobo++)
      for (int asad = 0; asad < nAsad; asad++)
         for (int aget = 0; aget < nAget; aget++) {
            diffHists[cobo][asad][aget]->Write();
            diffHistsRaw[cobo][asad][aget]->Write();
         }

   std::cout << "Closing file" << std::endl;
   outFile->Close();
}
