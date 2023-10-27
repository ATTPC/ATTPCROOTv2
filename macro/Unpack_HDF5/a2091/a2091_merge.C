void GetEnergy(Double_t M, Double_t IZ, Double_t BRO, float &E);

std::tuple<double, double>
kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject);

std::vector<int> runNumbers = {14, 15, 16, 17, 18, 19,
                               // 20,
                               // 21,
                               22, 23, 24,
                               // 26,
                               //  GET and NSCL run numbers got messed up? same with 30 and 31
                               // 27,
                               // 28,
                               //  29,
                               32, 33, 34, 35, 36, 37, 38, 39, 40, 42, 43, 44, 49, 50, 51, 52, 53, 54, 55, 56,
                               // 57,
                               58, 59, 60, 61, 62, 63, 64, 65, 98, 99};
// 115

Double_t m_p = 1.007825 * 931.49401;
Double_t m_d = 2.0135532 * 931.49401;
Double_t m_t = 3.016049281 * 931.49401;
Double_t m_He3 = 3.016029 * 931.49401;
Double_t m_Be10 = 10.013533818 * 931.49401;
Double_t m_Be11 = 11.021657749 * 931.49401;
Double_t m_Li9 = 9.026790 * 931.49401;
Double_t m_beam = m_Be10;
Float_t aMass = 4.00260325415;
Float_t O16Mass = 15.99491461956;
Double_t m_C14 = 14.003242 * 931.49401;
Double_t m_C13 = 13.00335484 * 931.49401;
Double_t m_C12 = 12.00 * 931.49401;
Double_t m_C16 = 16.0147 * 931.49401;
Double_t m_C17 = 17.0226 * 931.49401;
Double_t m_C15 = 15.0106 * 931.49401;

Double_t m_a = 4.00260325415 * 931.49401;
Double_t m_O16 = 15.99491461956 * 931.49401;

Double_t Ebeam_buff = 192.0;

double m_b = m_d;
double m_B = m_C15;

void a2091_merge(int runNum);
void a2091_merge_all()
{
   for (auto i : runNumbers) {
      // std::cout << "Unpacking " << i << std::endl;
      a2091_merge(i);
   }
}
void a2091_merge(std::string GETfile, std::string FRIBfile)
{
   TString runName = GETfile;
   TString outFileName = TString::Format("mergedData/%s_merged.root", GETfile.data());
   GETfile += ".root";
   FRIBfile += "_FRIB_sorted.root";

   std::cout << "Merging " << GETfile << " and " << FRIBfile << " into " << outFileName << std::endl;

   TFile outFile(outFileName, "RECREATE");
   outFile.cd();
   TTree *outTree = new TTree("a2091", "Merged Tree");

   // Data to fill in tree
   int eventID;
   int trackID;
   float theta;
   float bro;
   float ener;
   float zpos;
   float len;
   float eloss;
   float dedx;
   float Qval;
   int trackMult;
   std::vector<float> energyIC;

   outTree->Branch("eventID", &eventID);
   outTree->Branch("trackID", &trackID);
   outTree->Branch("theta", &theta);
   outTree->Branch("brho", &bro);
   outTree->Branch("energy", &ener);
   outTree->Branch("length", &len);
   outTree->Branch("zpos", &zpos);
   outTree->Branch("eloss", &eloss);
   outTree->Branch("dedx", &dedx);
   outTree->Branch("Qval", &Qval);
   outTree->Branch("track_mult", &trackMult);
   outTree->Branch("energyIC", &energyIC);
   outTree->Branch("run_name", &runName);

   // Open the GET file
   TFile *file = new TFile(GETfile.data(), "READ");
   TTree *tree = (TTree *)file->Get("cbmsim");
   int nEvents = tree->GetEntries();
   TTreeReader Reader1("cbmsim", file);
   TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtPatternEvent");
   TTreeReaderValue<TClonesArray> eventHArray(Reader1, "AtEventH");
   std::cout << " Number of events : " << nEvents << std::endl;

   // Open the FRIB file
   TFile *fileFRIB = new TFile(FRIBfile.data(), "READ");
   TTree *treeFRIB = (TTree *)fileFRIB->Get("FRIB_output_tree");
   int nEventsFRIB = treeFRIB->GetEntries();
   std::cout << " Number of FRIB DAQ events : " << nEventsFRIB << std::endl;
   TTreeReader Reader2("FRIB_output_tree", fileFRIB);
   TTreeReaderValue<ULong64_t> ts(Reader2, "timestamp");
   TTreeReaderValue<std::vector<Float_t>> energyICVec(Reader2, "energy");
   TTreeReaderValue<std::vector<Float_t>> timeIC(Reader2, "time");
   TTreeReaderValue<UInt_t> multIC(Reader2, "mult");
   TTreeReaderValue<std::string> fribEvName(Reader2, "eventName");

   ULong64_t fribTSRef = 0;
   ULong64_t getTSRef = 0;

   ULong64_t fribDTS = 0;
   ULong64_t getDTS = 0;

   for (Int_t i = 0; i < nEvents; i++) {

      // eventArray->Clear();
      if (i % 10000 == 0)
         std::cout << " Event Number : " << i << "\n";

      Reader1.Next();
      Reader2.Next();
      eventID = i;

      AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray->At(0);
      AtEvent *event = (AtEvent *)eventHArray->At(0);

      if (!patternEvent || !event)
         continue;
      std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
      auto eventName = event->GetEventName();
      auto getTS = event->GetTimestamp(1); // NB New merger only has the internal timestamp

      if (i == 0) {
         fribDTS = 0;
         getDTS = 0;
      } else {

         fribDTS = *ts - fribTSRef;
         getDTS = getTS - getTSRef;
      }

      if ((fribDTS > (getDTS + 5) || fribDTS < (getDTS - 5)) && i != 0) {
         std::cout << " -------------------------------- "
                   << "\n";
         std::cerr << i << "  " << fribDTS << "  " << getDTS << "\n";
         std::cerr << i << "  " << *ts << "  " << getTS << "\n";
         // std::exit(0);
      }

      getTSRef = getTS;
      fribTSRef = *ts;

      energyIC = *energyICVec;
      /*
      // Make sure this is a multiplicity 1 event
      if (*multIC != 1)
         continue;

      // Save the IC energy
      energyIC = energyICVec->at(0);
      */

      // Find track with largets angle
      auto itMax = std::max_element(patternTrackCand.begin(), patternTrackCand.end(),
                                    [](const auto &a, const auto &b) { return b.GetGeoTheta() > a.GetGeoTheta(); });

      if (itMax == patternTrackCand.end())
         continue;
      // std::cout << "Grabbing track" << std::endl;
      auto track = *itMax;
      trackID = std::distance(patternTrackCand.begin(), itMax);
      trackMult = patternTrackCand.size();

      theta = track.GetGeoTheta();
      Double_t rad = track.GetGeoRadius();
      // std::cout << "Got geometry" << std::endl;
      //  if (theta * TMath::RadToDeg() > 90.0)
      //   continue;

      Double_t B_f = 2.85;

      bro = B_f * rad / TMath::Sin(theta) / 1000.0;
      ener = 0;
      Double_t Am = 2.0;

      GetEnergy(Am, 1.0, bro, ener);

      len = 0;
      eloss = 0;
      dedx = 0;

      // Get dE/dx and total energy loss
      auto hitClusterArray = track.GetHitClusterArray();
      std::size_t cnt = 0;

      // std::cout << "Finding e loss" << std::endl;
      if (theta * TMath::RadToDeg() < 90) {

         auto firstCluster = hitClusterArray->back();
         zpos = firstCluster.GetPosition().Z();
         auto it = hitClusterArray->rbegin();
         while (it != hitClusterArray->rend()) {

            if (((Float_t)cnt / (Float_t)hitClusterArray->size()) > 0.7)
               break;
            auto dir = (*it).GetPosition() - (*std::next(it, 1)).GetPosition();
            eloss += (*it).GetCharge();
            len += std::sqrt(dir.Mag2());
            dedx += (*it).GetCharge();
            // std::cout<<(*it).GetCharge()<<"\n";
            it++;
            ++cnt;
         }
      } else if (theta * TMath::RadToDeg() > 90) {

         auto firstCluster = hitClusterArray->front();
         zpos = firstCluster.GetPosition().Z();
         eloss += hitClusterArray->at(0).GetCharge();

         cnt = 1;
         for (auto iHitClus = 1; iHitClus < hitClusterArray->size(); ++iHitClus) {

            if (((Float_t)cnt / (Float_t)hitClusterArray->size()) > 0.7)
               break;
            auto dir = hitClusterArray->at(iHitClus).GetPosition() - hitClusterArray->at(iHitClus - 1).GetPosition();
            len += std::sqrt(dir.Mag2());
            eloss += hitClusterArray->at(iHitClus).GetCharge();
            dedx += hitClusterArray->at(iHitClus).GetCharge();
            // std::cout<<len<<" - "<<eloss<<" - "<<hitClusterArray->at(iHitClus).GetCharge()<<"\n";
            ++cnt;
         }
      }
      // std::cout << "Found e loss" << std::endl;
      eloss /= cnt;
      dedx /= len;
      auto [ex_energy_exp, theta_cm] = kine_2b(m_C15, m_d, m_b, m_B, Ebeam_buff, theta, ener * Am);
      Qval = ex_energy_exp;

      outTree->Fill();
   } // End loop over events
   outFile.Write();
   outFile.Close();
}

void a2091_merge(int runNum)
{
   TString get = TString::Format("run_%04d", runNum);
   a2091_merge(get.Data(), get.Data());
}

void GetEnergy(Double_t M, Double_t IZ, Double_t BRO, float &E)
{

   // Energy per nucleon
   Float_t AM = 931.5;
   Float_t X = BRO / 0.1439 * IZ / M;
   X = pow(X, 2);
   X = 2. * AM * X;
   X = X + pow(AM, 2);
   E = TMath::Sqrt(X) - AM;
}

Double_t omega(Double_t x, Double_t y, Double_t z)
{
   return sqrt(x * x + y * y + z * z - 2 * x * y - 2 * y * z - 2 * x * z);
}

std::tuple<double, double>
kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject)
{

   // in this definition: m1(projectile); m2(target); m3(ejectile); and m4(recoil);
   double Et1 = K_proj + m1;
   double Et2 = m2;
   double Et3 = K_eject + m3;
   double Et4 = Et1 + Et2 - Et3;
   double m4_ex, Ex, theta_cm;
   double s, t, u; //---Mandelstam variables

   s = pow(m1, 2) + pow(m2, 2) + 2 * m2 * Et1;
   u = pow(m2, 2) + pow(m3, 2) - 2 * m2 * Et3;

   m4_ex = sqrt((cos(thetalab) * omega(s, pow(m1, 2), pow(m2, 2)) * omega(u, pow(m2, 2), pow(m3, 2)) -
                 (s - pow(m1, 2) - pow(m2, 2)) * (pow(m2, 2) + pow(m3, 2) - u)) /
                   (2 * pow(m2, 2)) +
                s + u - pow(m2, 2));
   Ex = m4_ex - m4;

   t = pow(m2, 2) + pow(m4_ex, 2) - 2 * m2 * Et4;

   // for inverse kinematics Note: this angle corresponds to the recoil
   theta_cm = TMath::Pi() - acos((pow(s, 2) + s * (2 * t - pow(m1, 2) - pow(m2, 2) - pow(m3, 2) - pow(m4_ex, 2)) +
                                  (pow(m1, 2) - pow(m2, 2)) * (pow(m3, 2) - pow(m4_ex, 2))) /
                                 (omega(s, pow(m1, 2), pow(m2, 2)) * omega(s, pow(m3, 2), pow(m4_ex, 2))));

   theta_cm = theta_cm * TMath::RadToDeg();
   return std::make_tuple(Ex, theta_cm);
}
