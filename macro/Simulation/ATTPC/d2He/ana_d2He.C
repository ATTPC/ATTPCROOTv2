

static Double_t proton_mass = 1.0078250322 * 931.494;
static Double_t proj_mass = 14.008596359 * 931.494;
static Double_t target_mass = 2.01410177812 * 931.494;
static Double_t recoil_mass = 14.00307400443 * 931.494;
static Double_t he2_mass = 2.0 * proton_mass;
static Double_t Ekin_proj = 105.16 * 14.008596359; // 100.0
// static Double_t decay_frag_mass = 14.00307400443*931.494/1000;//GeV/c^2
// static Double_t decay_frag_mass = 12.*931.494/1000;//GeV/c^2
static Double_t decay_frag_mass = 13.0033548352 * 931.494 / 1000; // GeV/c^2 13C
// static Double_t decay_frag_mass = 13.005738609*931.494/1000;//GeV/c^2 13N
// static Double_t decay_frag_mass = 10.012936862*931.494/1000;//GeV/c^2

TSpline3 *splineEloss;
using XYZVector = ROOT::Math::XYZVector;

std::pair<Double_t, Double_t> GetThetaPhi(AtTrack track, XYZPoint vertex, XYZPoint maxPos, Int_t zdir)
{
   std::pair<Double_t, Double_t> thetaPhi;
   std::vector<Double_t> par;
   par = track.GetPattern()->GetPatternPar();
   XYZVector vp(TMath::Sign(1, maxPos.X()) * fabs(par[3]), TMath::Sign(1, maxPos.Y()) * fabs(par[4]),
                zdir * TMath::Sign(1, (maxPos.Z() - vertex.Z())) * fabs(par[5])); // works with simu
   thetaPhi.first = vp.Theta();
   thetaPhi.second = vp.Phi();
   return thetaPhi;
}

Double_t FindAngleBetweenTracks(XYZVector vec1, XYZVector vec2)
{
   Double_t ang = acos(vec1.Dot(vec2) / (sqrt(vec1.Mag2()) * sqrt(vec2.Mag2())));
   return ang;
}

void SetERtable()
{ // fit of the GEANT4 E vs R obtained from the simulation with the function model given by LISE++
   ifstream fER("eLossTables/p_in_d_530torr_SRIM.txt"); // from SRIM++,
   Double_t l1 = 0, l2 = 0;
   vector<vector<Double_t>> Energy_Range;

   for (string line; getline(fER, line);) {
      stringstream parse_die(line);
      vector<Double_t> iRE;
      parse_die >> l1 >> l2;
      iRE.push_back(l1); // E in MeV
      iRE.push_back(l2); // mm
      Energy_Range.push_back(iRE);
   }
   fER.close();
   Int_t v_size = Energy_Range.size();
   Double_t X[v_size];
   Double_t Y[v_size];
   for (Int_t i = 0; i < v_size; i++) {
      X[i] = Energy_Range.at(i).at(0) * 1.; // 0.98
      Y[i] = Energy_Range.at(i).at(1) * 1.;
      // cout<<X[i]<<" "<<Y[i]<<endl;
   }
   // splineEloss = new TGraph(v_size,Y,X);
   splineEloss = new TSpline3("ElossRange", Y, X, v_size);
}

void ana_d2He()
{

   SetERtable();

   FairRunAna *run = new FairRunAna(); // Forcing a dummy run
   // ATd2HeAnalysis *d2heana = new ATd2HeAnalysis ();

   TString digiFileName =
      "/mnt/analysis/e18008/rootAna/giraud/simulation/digi/attpcdigi_d2He_1000_run0_Ex10_testUpdates.root";
   TFile *file = new TFile(digiFileName, "READ");
   TTree *tree = (TTree *)file->Get("cbmsim");
   Int_t nEvents = tree->GetEntries();
   std::cout << " Number of events : " << nEvents << std::endl;

   TTreeReader reader("cbmsim", file);
   TTreeReaderValue<TClonesArray> eventArray(reader, "AtPatternEvent");
   TTreeReaderValue<TClonesArray> SimPointArray(reader, "AtTpcPoint");

   TFile *outfile;
   TString outFileNameHead = "ana_d2He_test.root";
   outfile = TFile::Open(outFileNameHead, "recreate");

   //-----
   Int_t ivt = 0;
   Float_t range_p1 = 0., range_p2 = 0.;
   Float_t eLoss_p1_reco = 0.0, eLoss_p2_reco = 0.0;
   Float_t epsilon_pp = -999;
   Float_t theta1 = 0., theta2 = 0., phi1 = 0., phi2 = 0., angle12 = 0.;
   Float_t mom1_norm_reco = 0., mom2_norm_reco = 0.;
   Float_t E_tot_he2 = 0., he2_mass_ex = 0.;
   Float_t kin_He2 = 0., theta_He2 = 0., kin_He2_same = 0., theta_He2_same = 0., phi_He2 = 0.;
   Float_t theta_cm = 0., Ex4 = 0., Ex_reco_same = 0.;
   Float_t lastX1 = 0., lastX2 = 0., lastY1 = 0., lastY2 = 0., lastZ1 = 0., lastZ2 = 0., vertexX = 0., vertexY = 0.,
           vertexZ = 0.;
   Float_t MaxR1, MaxR2, MaxZ1, MaxZ2;
   Double_t theta_lab = 0;
   Double_t Eje_ata = -999, Eje_bta = -999, Eje_dta = -999;
   Double_t brho = -999;
   Double_t holeAcc = 0, dtaAcc = 0;

   XYZVector mom_proton1_reco, mom_proton2_reco, mom_He2_reco;

   XYZVector beamDir(-0.00915252, -0.00221017, 0.9999557);
   // XYZVector beamDir(0, 0, 1);

   vector<TString> fcutPID1File;
   vector<TCutG *> fcutPID1;

   fcutPID1File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/cutAcc.root");

   for (Int_t w = 0; w < fcutPID1File.size(); w++) {
      TFile f(fcutPID1File[w]);
      TIter next(f.GetListOfKeys());
      TKey *key;
      while ((key = (TKey *)next())) {
         cout << "PID1 Loading Cut file:  " << key->GetName() << endl;
         fcutPID1.push_back((TCutG *)f.Get(key->GetName()));
      }
   }

   TTree *anatree = new TTree("anatree", "new TTree");

   anatree->Branch("ivt", &ivt);
   anatree->Branch("range_p1", &range_p1);
   anatree->Branch("range_p2", &range_p2);
   anatree->Branch("theta1", &theta1);
   anatree->Branch("theta2", &theta2);
   anatree->Branch("phi1", &phi1);
   anatree->Branch("phi2", &phi2);
   anatree->Branch("lastX1", &lastX1);
   anatree->Branch("lastY1", &lastY1);
   anatree->Branch("lastZ1", &lastZ1);
   anatree->Branch("lastX2", &lastX2);
   anatree->Branch("lastY2", &lastY2);
   anatree->Branch("lastZ2", &lastZ2);
   anatree->Branch("vertexX", &vertexX);
   anatree->Branch("vertexY", &vertexY);
   anatree->Branch("vertexZ", &vertexZ);
   anatree->Branch("angle12", &angle12);
   anatree->Branch("eLoss_p1_reco", &eLoss_p1_reco);
   anatree->Branch("eLoss_p2_reco", &eLoss_p2_reco);
   anatree->Branch("kin_He2", &kin_He2);
   anatree->Branch("theta_He2", &theta_He2);
   anatree->Branch("E_tot_he2", &E_tot_he2);
   anatree->Branch("he2_mass_ex", &he2_mass_ex);
   anatree->Branch("theta_cm", &theta_cm);
   anatree->Branch("theta_lab", &theta_lab);
   anatree->Branch("Ex4", &Ex4);
   anatree->Branch("epsilon_pp", &epsilon_pp);
   anatree->Branch("mom1_norm_reco", &mom1_norm_reco);
   anatree->Branch("mom2_norm_reco", &mom2_norm_reco);
   anatree->Branch("Eje_ata", &Eje_ata);
   anatree->Branch("Eje_bta", &Eje_bta);
   anatree->Branch("Eje_dta", &Eje_dta);
   anatree->Branch("brho", &brho);
   anatree->Branch("holeAcc", &holeAcc);
   anatree->Branch("dtaAcc", &dtaAcc);

   /// --------------------- Event loop -------------------------------------------

   for (Int_t i = 0; i < nEvents; i++) {

      std::cout << " Event Number : " << i << "\n";

      reader.Next();
      if (i % 2 != 0)
         continue;

      // reaction products quantities from GEANT4--------------------------------------

      TVector3 Pejec;
      TVector3 pfirst;
      TVector3 plast;
      TVector3 Pproj(0, 0, 1);
      Double_t Anejec;
      Double_t Anp1;
      Double_t Anp2;
      Double_t Eeje = 0;
      Double_t Peje = 0;
      Double_t Ep1 = 0;
      Double_t Ep2 = 0;
      Eje_ata = -999;
      Eje_bta = -999;
      Eje_dta = -999;
      Bool_t fl1 = kTRUE;
      Bool_t fl2 = kTRUE;
      TVector3 momEje;
      Int_t Npoints = SimPointArray->GetEntries();
      // get information from geant4
      for (Int_t ipoint = 0; ipoint < Npoints; ipoint++) {
         auto point = dynamic_cast<AtMCPoint *>(SimPointArray->At(ipoint));
         TString VolName = point->GetVolName();
         Int_t trackID = point->GetTrackID();

         // std::cout<<"trackID "<<trackID<<" "<<point ->GetMassNum()<<" "<<point ->GetAtomicNum()<<" X
         // "<<point->GetX()<<" Y "<<point->GetY()<<" Z "<<point->GetZ()<<std::endl;

         if (trackID == 4 && VolName == "drift_volume") { // beam-like 1; decay like 4
            // Anejec = point->GetAIni();
            // Eeje = point->GetEIni();

            if (point->GetZ() >= 100) {
               Peje = sqrt(pow(point->GetPx(), 2) + pow(point->GetPy(), 2) + pow(point->GetPz(), 2));   // GeV/c
               Eeje = (sqrt(Peje * Peje + decay_frag_mass * decay_frag_mass) - decay_frag_mass) * 1000; // MeV
               // Eje_dta = (Eeje-2.18)/1493.7515- 1.0;//1493.75 MeV is the TKE of central 14N at rigidity=3.05824
               // Tm//2.18 for matching data (eloss in windows, obj scint )
               // Eje_dta = (Eeje-2.18)/1280.6107- 1.0;//12C
               Eje_dta = (Eeje - 2.18) / 1359.27 - 1.0; // 13C
               // Eje_dta = (Eeje-2.18)/1385.54- 1.0;//13N
               // Eje_dta = (Eeje-2.18)/1065.94- 1.0;//10B
               momEje.SetXYZ(point->GetPx(), point->GetPy(), point->GetPz());
               Eje_ata = atan(momEje.X() / momEje.Z()) -
                         2 * 0.0019; // offset between angle of the beam in tpc frame and S800 frame
               Eje_bta = atan(momEje.Y() / momEje.Z());
               Double_t xBell = point->GetX() + 40. * tan(Eje_ata);
               Double_t yBell = point->GetY() + 40. * tan(Eje_bta);
               if (sqrt(pow(xBell, 2) + pow(yBell, 2)) < 2.5)
                  holeAcc = 1; // 2. cm radius exit hole at 100cm but bellows of 2.5cm radius at 140cm so 1.786cm
                               // effective radius at 100cm
               // if( sqrt(pow(point->GetXIn(),2) + pow(point->GetYIn(),2)) <1.786) holeAcc = 1;//2. cm radius exit hole
               // at 100cm but bellows of 2.5cm radius at 140cm so 1.786cm effective radius at 100cm
            }
            if (fl1) {
               fl1 = kFALSE;
               pfirst.SetXYZ(point->GetX(), point->GetY(), point->GetZ());
               // Peje = sqrt(pow(point->GetPxOut(),2)+pow(point->GetPyOut(),2)+pow(point->GetPzOut(),2));
               // std::cout<<i<<" fl1  "<<trackID<<"  "<<ipoint<<"   "<<point->GetZIn()<<"  "<<point->GetZOut()<<"
               // "<<point->GetPzOut()<<std::endl;
               // std::cout<<" Elast first "<<Eeje<<" "<<point->GetPzOut()<<" "<<point->GetZIn()<<"
               // "<<Anejec<<std::endl;
            }
            if (fl2 && point->GetZ() > 99) {
               fl2 = kFALSE;
               plast.SetXYZ(point->GetX(), point->GetY(), point->GetZ());
               // Pejec.SetXYZ(point->GetPxOut(),point->GetPyOut(),point->GetPzOut());
               // std::cout<<i<<" fl2 "<<trackID<<"  "<<ipoint<<"   "<<point->GetZIn()<<"  "<<point->GetZOut()<<"
               // "<<point->GetPzOut()<<std::endl;
            }
         }
         if (trackID == 2 && VolName == "drift_volume") { // p1
            Anp1 = point->GetAIni();
            Ep1 = point->GetEIni();
         }
         if (trackID == 3 && VolName == "drift_volume") { // p2
            Anp2 = point->GetAIni();
            Ep2 = point->GetEIni();
         }
      } // simulated points

      for (Int_t w = 0; w < fcutPID1.size(); w++)
         if (fcutPID1[w]->IsInside(Eje_dta, Eje_ata))
            dtaAcc += 1;
      // std::cout<<" Elast1 "<<Anejec<<" "<<Eeje<<" "<<Peje<<" "<<atan(momEje.X()/momEje.Z())<<"
      // "<<atan(momEje.Y()/momEje.Z())<<" "<<Eje_dta<<std::endl;

      TLorentzVector ejec4v;
      ejec4v.SetXYZM(momEje.X(), momEje.Y(), momEje.Z(), decay_frag_mass);
      Double_t gamma = ejec4v.Gamma();
      Double_t beta = ejec4v.Beta();
      //--check eq., same results with root functions---------------------------------
      //	Double_t gamma_bis = 1.+Eeje/(decay_frag_mass*1e3);
      //	Double_t beta_bis = sqrt(gamma_bis*gamma_bis-1.)/gamma_bis;
      //	Double_t brho_bis = 3.1*beta_bis*gamma_bis*12./6.;
      //------------------------------------------------------------------------------
      brho = 3.1 * beta * gamma * 13. / 6.;
      std::cout << " brho " << beta << " " << brho << std::endl;
      // std::cout<<"check analysis ata, bta "<<Eje_ata<<" "<<Eje_bta<<std::endl;
      // std::cout<<"check analysis fPx, fPy, fPz "<<momEje.X()<<" "<<fPy<<" "<<fPz<<std::endl;

      //	std::cout<<" bis beta "<<beta<<" "<<beta_bis<<" gamma "<<gamma<<" "<<gamma_bis<<" brho "<<brho<<"
      //"<<brho_bis<<std::endl;

      //------------------------------------------------------------------------------

      AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray->At(0);

      if (patternEvent) {
         std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
         std::vector<std::vector<Float_t>> lines;
         /*std::cout << " Number of pattern tracks " << patternTrackCand.size() << "\n";
         for (auto track : patternTrackCand) {

           auto ransacLine = dynamic_cast<const AtPatterns::AtPatternLine *>(track.GetPattern());
           std::vector<Float_t> patternPar;
           patternPar = ransacLine->GetPatternPar();

           for (auto par : patternPar) {std::cout << " pattern par. : " << par << "\n";}

           lines.push_back(patternPar);

         }*/
         AtFindVertex findVtx(12);
         findVtx.FindVertex(patternTrackCand, 2);
         std::vector<tracksFromVertex> tv;
         tv = findVtx.GetTracksVertex();
         for (size_t itv = 0; itv < tv.size(); itv++) {
            std::cout << itv << " " << tv.at(itv).vertex.X() << " " << tv.at(itv).vertex.Y() << " "
                      << tv.at(itv).vertex.Z() << " " << tv.at(itv).tracks.at(0).GetGeoQEnergy() << std::endl;

            theta1 = 0.;
            theta2 = 0.;
            phi1 = 0.;
            phi2 = 0.;
            range_p1 = 0.;
            range_p2 = 0.;
            eLoss_p1_reco = 0.;
            eLoss_p2_reco = 0.;
            mom1_norm_reco = 0.;
            mom2_norm_reco = 0.; // reset variables
            E_tot_he2 = 0.;
            he2_mass_ex = 0.;
            kin_He2 = 0.;
            theta_He2 = 0.;
            phi_He2 = 0.;
            theta_cm = 0.;
            Ex4 = 0.;
            MaxR1 = 0.;
            MaxR2 = 0.;
            MaxZ1 = 0.;
            MaxZ2 = 0.;
            theta_lab = 0;

            XYZPoint vertexMean = (XYZPoint)tv.at(itv).vertex;
            XYZPoint lastPoint1 = tv.at(itv).tracks.at(0).GetLastPoint();
            XYZPoint lastPoint2 = tv.at(itv).tracks.at(1).GetLastPoint();
            MaxR1 = lastPoint1.Rho();
            MaxR2 = lastPoint2.Rho();
            MaxZ1 = lastPoint1.Z();
            MaxZ2 = lastPoint2.Z();

            if (MaxR1 > 245. || MaxR2 > 245. || MaxR1 < 35. || MaxR2 < 35. || MaxZ1 > 975. || MaxZ2 > 975. ||
                MaxZ1 < 25. || MaxZ2 < 25. || vertexMean.Z() < 25. || vertexMean.Z() > 975.)
               continue;

            lastX1 = lastPoint1.X();
            lastY1 = lastPoint1.Y();
            lastZ1 = lastPoint1.Z();
            lastX2 = lastPoint2.X();
            lastY2 = lastPoint2.Y();
            lastZ2 = lastPoint2.Z();
            vertexX = vertexMean.X();
            vertexY = vertexMean.Y();
            vertexZ = vertexMean.Z();

            theta1 = GetThetaPhi(tv.at(itv).tracks.at(0), vertexMean, lastPoint1, -1)
                        .first; // GetThetaPhi(..,..,-1) for simu;
            theta2 = GetThetaPhi(tv.at(itv).tracks.at(1), vertexMean, lastPoint2, -1).first;
            phi1 = GetThetaPhi(tv.at(itv).tracks.at(0), vertexMean, lastPoint1, -1).second;
            phi2 = GetThetaPhi(tv.at(itv).tracks.at(1), vertexMean, lastPoint2, -1).second;

            std::vector<Double_t> fitPar1 = tv.at(itv).tracks.at(0).GetPattern()->GetPatternPar();
            std::vector<Double_t> fitPar2 = tv.at(itv).tracks.at(1).GetPattern()->GetPatternPar();

            XYZVector vp1(TMath::Sign(1, lastX1) * fabs(fitPar1[3]), TMath::Sign(1, lastY1) * fabs(fitPar1[4]),
                          -TMath::Sign(1, (lastZ1 - vertexZ)) * fabs(fitPar1[5]));
            XYZVector vp2(TMath::Sign(1, lastX2) * fabs(fitPar2[3]), TMath::Sign(1, lastY2) * fabs(fitPar2[4]),
                          -TMath::Sign(1, (lastZ2 - vertexZ)) * fabs(fitPar2[5]));
            angle12 = FindAngleBetweenTracks(vp1, vp2);

            // std::cout<<i<<" "<<" protons 1 2 theta : "<<theta1*TMath::RadToDeg()<<"
            // "<<theta2*TMath::RadToDeg()<<"\n"; std::cout<<i<<" protons 1 2 phi : "<<phi1*TMath::RadToDeg()<<"
            // "<<phi2*TMath::RadToDeg()<<"\n";

            range_p1 = tv.at(itv).tracks.at(0).GetLinearRange(vertexMean, lastPoint1);
            range_p2 = tv.at(itv).tracks.at(0).GetLinearRange(vertexMean, lastPoint2);

            //==============================================================================
            // methods to get the proton eloss

            // eLoss_p1_reco = eloss_approx(range_p1);
            // eLoss_p2_reco = eloss_approx(range_p2);

            eLoss_p1_reco = splineEloss->Eval(range_p1);
            eLoss_p2_reco = splineEloss->Eval(range_p2);

            // std::cout<<i<<" vertex : "<<vertexZ<<"\n";
            // std::cout<<i<<" range p1 p2 : "<<range_p1<<" "<<range_p2<<"\n";
            // std::cout<<i<<" eloss reco : "<<eLoss_p1_reco<<" "<<eLoss_p2_reco<<" "<<"\n";

            //==============================================================================

            epsilon_pp =
               0.5 * (eLoss_p1_reco + eLoss_p2_reco - 2 * sqrt(eLoss_p1_reco * eLoss_p2_reco) * TMath::Cos(angle12));

            // reconstruction of 2He
            mom1_norm_reco = TMath::Sqrt(eLoss_p1_reco * eLoss_p1_reco + 2.0 * eLoss_p1_reco * proton_mass);
            mom_proton1_reco.SetX(mom1_norm_reco * TMath::Sin(theta1) * TMath::Cos(phi1));
            mom_proton1_reco.SetY(mom1_norm_reco * TMath::Sin(theta1) * TMath::Sin(phi1));
            mom_proton1_reco.SetZ(mom1_norm_reco * TMath::Cos(theta1));

            mom2_norm_reco = TMath::Sqrt(eLoss_p2_reco * eLoss_p2_reco + 2.0 * eLoss_p2_reco * proton_mass);
            mom_proton2_reco.SetX(mom2_norm_reco * TMath::Sin(theta2) * TMath::Cos(phi2));
            mom_proton2_reco.SetY(mom2_norm_reco * TMath::Sin(theta2) * TMath::Sin(phi2));
            mom_proton2_reco.SetZ(mom2_norm_reco * TMath::Cos(theta2));
            // std::cout<<i<<" mom1 : "<<mom_proton1_reco.Mag()<<"\n";
            // std::cout<<i<<" mom2 : "<<mom_proton2_reco.Mag()<<"\n";

            mom_He2_reco = mom_proton1_reco + mom_proton2_reco;

            //------- rotation of track vectors so that Theta and Phi are in the beam frame
            TVector3 momBuff; // dirty trick to use Rotate functions (not available with XYZvector)
            momBuff.SetXYZ(mom_He2_reco.X(), mom_He2_reco.Y(), mom_He2_reco.Z());
            Double_t aRX = TMath::ATan2(beamDir.Y(), beamDir.Z());
            Double_t aRY = TMath::ATan2(-beamDir.X(), beamDir.Z());
            momBuff.RotateX(aRX); // rotate in trigo sens, y to z
            momBuff.RotateY(aRY); // rotate in trigo sens, z to x
            mom_He2_reco.SetXYZ(momBuff.X(), momBuff.Y(), momBuff.Z());

            E_tot_he2 = (proton_mass + eLoss_p1_reco) + (proton_mass + eLoss_p2_reco);
            he2_mass_ex = TMath::Sqrt(E_tot_he2 * E_tot_he2 - mom_He2_reco.Mag2());
            // ex_he2_reco->Fill (he2_mass_ex - he2_mass);
            kin_He2 = TMath::Sqrt(mom_He2_reco.Mag2() + he2_mass_ex * he2_mass_ex) - he2_mass_ex;
            theta_He2 = mom_He2_reco.Theta() * TMath::RadToDeg();

            Double_t mom_beam = sqrt(pow(Ekin_proj + proj_mass, 2) - pow(proj_mass, 2));
            Double_t missing_mom;
            missing_mom = sqrt((mom_beam * beamDir - mom_He2_reco).Mag2());
            Double_t missing_energy = (Ekin_proj + proj_mass + target_mass - (kin_He2 + epsilon_pp + he2_mass));
            Double_t missing_mass = sqrt(pow(missing_energy, 2) - pow(missing_mom, 2));

            phi_He2 = mom_He2_reco.Phi() * TMath::RadToDeg();
            // theta_r_he2_reco->Fill (theta_He2);
            // phi_r_he2_reco->Fill (phi_He2);
            // kin_r_he2_reco->Fill (kin_He2);
            // theta_kin_he2_reco->Fill (theta_He2, kin_He2);

            Ex4 = missing_mass - recoil_mass;

            Double_t sInv = pow(target_mass + proj_mass, 2) + 2. * target_mass * Ekin_proj;
            Double_t momCMScat = sqrt((pow(sInv - pow(he2_mass + epsilon_pp, 2) - pow(Ex4 + recoil_mass, 2), 2) -
                                       4. * pow(he2_mass + epsilon_pp, 2) * pow(Ex4 + recoil_mass, 2)) /
                                      (4. * sInv));
            theta_He2 = mom_He2_reco.Theta() * TMath::RadToDeg();
            Double_t thetaCMScat = asin(sqrt(mom_He2_reco.Mag2()) / momCMScat * sin(theta_He2 * TMath::DegToRad()));
            theta_lab = atan(sin(theta_cm * TMath::DegToRad()) /
                             (cos(theta_cm * TMath::DegToRad()) + recoil_mass / target_mass));
            theta_cm = thetaCMScat * TMath::RadToDeg();

            std::cout << " check values, Ex  " << Ex4 << " theta_cm " << theta_cm << " epsilon_pp " << epsilon_pp << " "
                      << std::endl;

            // thetacm_he2_reco->Fill (theta_cm);
            // Ex_reco->Fill (Ex4);
            // thetacm_Ex_he2_reco->Fill (theta_cm, Ex4);
            // thetacm_epsilon_pp_reco->Fill(theta_cm,epsilon_pp);

            ivt = i;
            anatree->Fill();
         } // for tv size (itv)
      }    // RANSAC null pointer
   }       // Event loop

   /// --------------------- End event loop ---------------------------------------
   outfile->cd();
   anatree->Write();

   //	tracks_z_r->Write ();
   //	tracks_x_y->Write ();
   // theta_r_he2_reco->Write ();
   // phi_r_he2_reco->Write ();
   // kin_r_he2_reco->Write ();
   // theta_kin_he2_reco->Write ();
   // thetacm_he2_reco->Write ();
   // Ex_reco->Write ();
   // ex_he2_reco->Write ();
   // thetacm_Ex_he2_reco->Write ();
   // thetacm_epsilon_pp_reco->Write();
   // epsilon_pp_reco->Write();

   outfile->Close();

} // end main
