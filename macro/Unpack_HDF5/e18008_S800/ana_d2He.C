
//#include "AtPattern.h"

static Double_t proton_mass = 1.0078250322 * 931.494 - 0.511;
static Double_t proj_mass = 14.008596359 * 931.494 - 0.511 * 8.;
static Double_t target_mass = 2.01410177812 * 931.494;
static Double_t recoil_mass = 14.00307400443 * 931.494 - 0.511 * 7.;
static Double_t he2_mass = 2.0 * proton_mass;
static Double_t Ekin_proj = 105.16 * 14.008596359; // 100.0
// static Double_t decay_frag_mass = 14.00307400443*931.494/1000;//GeV/c^2
// static Double_t decay_frag_mass = 12.*931.494/1000;//GeV/c^2
static Double_t decay_frag_mass = 13.0033548352 * 931.494 / 1000; // GeV/c^2 13C
// static Double_t decay_frag_mass = 13.005738609*931.494/1000;//GeV/c^2 13N
// static Double_t decay_frag_mass = 10.012936862*931.494/1000;//GeV/c^2

static Int_t nbTracksPerVtx = 2;

TSpline3 *splineEloss;
using XYZVector = ROOT::Math::XYZVector;
using XYZPoint = ROOT::Math::XYZPoint;
Double_t aDVel[300];

std::pair<Double_t, Double_t> GetThetaPhi(AtTrack track, XYZVector vertex, XYZVector maxPos, Int_t zdir)
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

std::pair<Double_t, Double_t> GetThetaPhi(XYZVector vertex, XYZVector maxPos)
{
   std::pair<Double_t, Double_t> thetaPhi;
   XYZVector vp = maxPos - vertex;
   thetaPhi.first = vp.Theta();
   thetaPhi.second = vp.Phi();
   return thetaPhi;
}

Double_t FindAngleBetweenTracks(XYZVector vec1, XYZVector vec2)
{
   Double_t ang = acos(vec1.Dot(vec2) / (sqrt(vec1.Mag2()) * sqrt(vec2.Mag2())));
   return ang;
}

// returns the projection of a point on a parametric line
// dir is the direction of the parametric line, posOn is a point of the line, posOut is the point that will be projected
XYZVector ptOnLine(std::vector<Double_t> par, XYZVector posOut)
{
   XYZVector result(-999, -999, -999);
   XYZVector posOn(par[0], par[1], par[2]);
   XYZVector dir(par[3], par[4], par[5]);
   XYZVector vop1 = ((dir.Cross(posOut - posOn)).Cross(dir)).Unit();
   Double_t paraVar1 = posOut.Dot(dir.Unit()) - posOn.Dot(dir.Unit());
   Double_t paraVar2 = posOn.Dot(vop1) - posOut.Dot(vop1);
   XYZVector vInter1 = posOn + dir.Unit() * paraVar1;
   XYZVector vInter2 = posOut + vop1 * paraVar2;
   if ((vInter1 - vInter2).Mag2() < 1e-10)
      result = vInter1;
   return result;
}

void SetDVelArray()
{
   TString fileName = "utils/drift_vel_cal_vtxZ_FermiFit.txt";
   ifstream fDVel(fileName);
   Int_t l1 = 0;
   Double_t l2 = 0;
   for (string line; getline(fDVel, line);) {
      stringstream parse_die(line);
      parse_die >> l1 >> l2;
      aDVel[l1] = l2;
   }
   fDVel.close();
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

XYZVector ClosestPoint2Lines(std::vector<Double_t> par1, std::vector<Double_t> par2, Int_t nHits1, Int_t nHits2)
{
   XYZVector p1(par1[0], par1[1], par1[2]); // p1
   XYZVector e1(par1[3], par1[4], par1[5]); // d1
   XYZVector p2(par2[0], par2[1], par2[2]); // p2
   XYZVector e2(par2[3], par2[4], par2[5]); // d2
   XYZVector n1 = e1.Cross(e2.Cross(e1));
   XYZVector n2 = e2.Cross(e1.Cross(e2));
   double t1 = (p2 - p1).Dot(n2) / (e1.Dot(n2));
   double t2 = (p1 - p2).Dot(n1) / (e2.Dot(n1));
   XYZVector c1 = p1 + t1 * e1;
   XYZVector c2 = p2 + t2 * e2;
   Double_t w1 = (Double_t)nHits1 / (nHits1 + nHits2);
   Double_t w2 = (Double_t)nHits2 / (nHits1 + nHits2);
   XYZVector meanpoint;
   XYZVector meanpoint1 = w1 * c1 + w2 * c2;
   XYZVector meanpoint2 = 0.5 * (c1 + c2);
   if ((nHits1 > 8 && nHits2 > 8) && (nHits1 < 50 || nHits2 < 50))
      meanpoint = meanpoint1; // if sufficient number of hits use the not weighted average
   else
      meanpoint = meanpoint2;
   return meanpoint;
}

void ana_d2He(Int_t runNumber)
{

   SetERtable();
   SetDVelArray();

   FairRunAna *run = new FairRunAna(); // Forcing a dummy run
   // ATd2HeAnalysis *d2heana = new ATd2HeAnalysis ();

   // TString digiFileName = "/mnt/analysis/e18008/rootMerg/giraud/run_2271_0271_test15.root";
   TString digiFileName =
      TString::Format("/mnt/analysis/e18008/rootMerg/giraud/run_2%03d_%04d_test15.root", runNumber, runNumber);
   TFile *file = new TFile(digiFileName, "READ");
   TTree *tree = (TTree *)file->Get("cbmsim");
   Int_t nEvents = tree->GetEntries();
   std::cout << " Number of events : " << nEvents << std::endl;

   S800Calc *s800cal = new S800Calc();
   TBranch *bS800cal = tree->GetBranch("s800cal");
   bS800cal->SetAddress(&s800cal);

   TTreeReader reader("cbmsim", file);
   TTreeReaderValue<TClonesArray> patternArray(reader, "AtPatternEvent");
   TTreeReaderValue<TClonesArray> eventArray(reader, "AtEventH");

   TFile *outfile;
   // TString  outFileNameHead = "ana_d2He_test_271.root";
   TString outFileNameHead = TString::Format("ana_d2He_test_%04d_14N_nbptsW.root", runNumber);
   outfile = TFile::Open(outFileNameHead, "recreate");

   S800Ana s800Ana;

   //-----
   Int_t ivt = 0, irun = runNumber, NVtxEvt = 0, NTracksVtx = 0;
   Float_t range_p1 = 0., range_p2 = 0., charge1 = 0., charge2 = 0.;
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

   ULong64_t S800_timeStamp = 0;
   Double_t S800_timeRf = 0., S800_x0 = 0., S800_x1 = 0., S800_y0 = 0., S800_y1 = 0., S800_E1up = 0., S800_E1down = 0.,
            S800_hodoSum = 0., S800_afp = 0., S800_bfp = 0., S800_ata = 0., S800_bta = 0., S800_yta = 0., S800_dta = 0.,
            S800_thetaLab = 0., S800_phi = 0., S800_E1up_ToF = 0., S800_E1down_ToF = 0., S800_E1_ToF = 0.,
            S800_XfObj_ToF = 0., S800_ObjCorr_ToF = 0., S800_Obj_ToF = 0., S800_XfObjCorr_ToF = 0., S800_ICSum_dE = 0.;

   XYZVector beamDir(-0.00915252, -0.00221017, 0.9999557);

   //---- Set S800Ana -------------------------------------------------------------
   vector<TString> fcutPID1File;
   vector<TString> fcutPID2File;
   vector<TString> fcutPID3File;

   // fcutPID1File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/XfObjObj_run115.root");
   // fcutPID2File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/ICSumObj_run115.root");
   fcutPID1File.push_back(
      "/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/XfObj14O.root");
   fcutPID2File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/afpx.root");
   // fcutPID3File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/13N.root");
   // fcutPID3File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/13C.root");
   fcutPID3File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/14N.root");
   // fcutPID3File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/12C.root");
   // fcutPID3File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/10B.root");
   std::vector<Double_t> S800MTDCObjCorr;
   S800MTDCObjCorr.push_back(70.);
   S800MTDCObjCorr.push_back(0.0085);
   std::vector<Double_t> S800MTDCObjRange;
   S800MTDCObjRange.push_back(-120);
   S800MTDCObjRange.push_back(-20);
   std::vector<Double_t> S800MTDCXfRange;
   S800MTDCXfRange.push_back(160);
   S800MTDCXfRange.push_back(240);

   s800Ana.SetPID1cut(fcutPID1File);
   s800Ana.SetPID2cut(fcutPID2File);
   s800Ana.SetPID3cut(fcutPID3File);
   s800Ana.SetMTDCXfRange(S800MTDCXfRange);
   s800Ana.SetMTDCObjRange(S800MTDCObjRange);
   s800Ana.SetTofObjCorr(S800MTDCObjCorr);
   //----------------------------------------------------------------------------

   TTree *anatree = new TTree("anatree", "new TTree");

   anatree->Branch("ivt", &ivt);
   anatree->Branch("irun", &irun);
   anatree->Branch("NVtxEvt", &NVtxEvt);
   anatree->Branch("NTracksVtx", &NTracksVtx);
   anatree->Branch("range_p1", &range_p1);
   anatree->Branch("range_p2", &range_p2);
   anatree->Branch("charge1", &charge1);
   anatree->Branch("charge2", &charge2);
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

   anatree->Branch("S800_XfObj_ToF", &S800_XfObj_ToF);
   anatree->Branch("S800_ObjCorr_ToF", &S800_ObjCorr_ToF);
   anatree->Branch("S800_ICSum_dE", &S800_ICSum_dE);

   ///============================= Event loop ====================================
   std::cout << " nEvents : " << nEvents << "\n";
   for (Int_t i = 0; i < nEvents; i++) {
      s800cal->Clear();
      bS800cal->GetEntry(i);
      reader.Next();

      Bool_t isInPIDGates = kFALSE;
      isInPIDGates = s800Ana.isInPID(s800cal);

      if (!isInPIDGates)
         continue;

      std::cout << " Event Number : " << i << "\n";

      //---- Get S800 data -----------------------------------------------------------
      S800_XfObj_ToF = s800Ana.GetXfObj_ToF();
      S800_ObjCorr_ToF = s800Ana.GetObjCorr_ToF();
      S800_ICSum_dE = s800Ana.GetICSum_E();
      std::vector<Double_t> S800_fpVar; //[0]=fX0 | [1]=fX1 | [2]=fY0 | [3]=fY1 | [4]=fAfp | [5]=fBfp
      S800_fpVar = s800Ana.GetFpVariables();

      //------------------------------------------------------------------------------

      AtPatternEvent *patternEvent = (AtPatternEvent *)patternArray->At(0);

      if (patternEvent) {

         std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
         // std::vector<std::vector<Float_t>> lines;
         /*
      std::cout << " Number of pattern tracks " << patternTrackCand.size() << "\n";
      for (auto track : patternTrackCand) {
        auto ransacLine = dynamic_cast<const AtPatterns::AtPatternLine *>(track.GetPattern());
        std::vector<Float_t> patternPar;
        patternPar = ransacLine->GetPatternPar();
        for (auto par : patternPar) {std::cout << " ini pattern par. : " << par << "\n";}
        //lines.push_back(patternPar);
      }
         */

         AtFindVertex findVtx(9);
         findVtx.FindVertex(patternTrackCand, nbTracksPerVtx);
         std::vector<tracksFromVertex> tv;
         tv = findVtx.GetTracksVertex();
         NVtxEvt =
            tv.size();   // number of clusters of tracks forming a vertex (could have ex: 2 vertexes with 2 tracks each)
         NTracksVtx = 0; // number of tracks for each vertex

         for (size_t ive = 0; ive < NVtxEvt; ive++) {
            std::cout << "ive " << ive << " " << tv.at(ive).vertex.X() << " " << tv.at(ive).vertex.Y() << " "
                      << tv.at(ive).vertex.Z() << " " << tv.at(ive).tracks.at(0).GetGeoQEnergy() << " "
                      << tv.at(ive).tracks.at(1).GetGeoQEnergy() << std::endl;

            NTracksVtx = tv.at(ive).tracks.size();
            if (NTracksVtx != 2)
               continue; // don't analyze event with other than 2 tracks per vertex

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
            charge1 = 0;
            charge2 = 0;

            XYZVector vertexMean = tv.at(ive).vertex;
            XYZVector lastPoint1 = (XYZVector)tv.at(ive).tracks.at(0).GetLastPoint();
            XYZVector lastPoint2 = (XYZVector)tv.at(ive).tracks.at(1).GetLastPoint();
            MaxR1 = lastPoint1.Rho();
            MaxR2 = lastPoint2.Rho();
            MaxZ1 = lastPoint1.Z();
            MaxZ2 = lastPoint2.Z();

            charge1 = tv.at(ive).tracks.at(0).GetGeoQEnergy();
            charge2 = tv.at(ive).tracks.at(1).GetGeoQEnergy();

            // std::cout<<charge1<<" "<<charge2<<std::endl;

            // tracks must stop within the chamber
            if (charge1 < 5e3 || charge2 < 5e3 || MaxR1 > 245. || MaxR2 > 245. || MaxR1 < 35. || MaxR2 < 35. ||
                MaxZ1 > 975. || MaxZ2 > 975. || MaxZ1 < 25. || MaxZ2 < 25. || vertexMean.Z() < 25. ||
                vertexMean.Z() > 975.)
               continue;

            //------------------------------------------------------------------------------
            // refit only the last part of the tracks and do small adjustement for the drift vel.
            std::vector<AtTrack> patternTrackCandReFit;
            for (size_t itv = 0; itv < NTracksVtx; itv++) {
               AtTrack trackToReFit;
               std::vector<AtHit> hitArray = tv.at(ive).tracks.at(itv).GetHitArray();
               for (Int_t iHit = 0; iHit < hitArray.size(); iHit++) {
                  AtHit hit = hitArray.at(iHit);
                  XYZPoint position = hit.GetPosition();
                  position = position.SetXYZ(position.X(), position.Y(), (1000. / aDVel[irun]) * (position.Z()));
                  hit.SetPosition(position);
                  if (sqrt(pow(position.X(), 2) + pow(position.Y(), 2)) > 35.) {
                     trackToReFit.AddHit(hit);
                  }
               }

               auto patternType = AtPatterns::PatternType::kLine;
               auto pattern = AtPatterns::CreatePattern(patternType);
               pattern->FitPattern(trackToReFit.GetHitArray(),
                                   100); // 100 is qThreshold, defined in the unpack macro as well
               trackToReFit.SetPattern(pattern->Clone());
               patternTrackCandReFit.push_back(trackToReFit);
            }

            // find new vertex
            std::vector<tracksFromVertex> tvReFit;
            if (patternTrackCandReFit.size() >= nbTracksPerVtx) {
               AtFindVertex findVtxReFit(9);
               findVtxReFit.FindVertex(patternTrackCandReFit, nbTracksPerVtx);
               tvReFit = findVtxReFit.GetTracksVertex();
            }
            std::cout << "tvReFit.size() "
                      << " " << tvReFit.size() << " " << nbTracksPerVtx << " " << patternTrackCandReFit.size()
                      << std::endl;
            if (tvReFit.size() < 1)
               continue; // tvReFit size should be 1

            vertexMean = (XYZPoint)tvReFit.at(0).vertex;
            // test ----
            auto ransacLine1 = dynamic_cast<const AtPatterns::AtPatternLine *>(tvReFit.at(0).tracks.at(0).GetPattern());
            auto ransacLine2 = dynamic_cast<const AtPatterns::AtPatternLine *>(tvReFit.at(0).tracks.at(1).GetPattern());
            // XYZVector newVertexMean =
            // ClosestPoint2Lines(ransacLine1->GetPatternPar(),ransacLine2->GetPatternPar(),tvReFit.at(0).tracks.at(0).GetHitArray().size(),tvReFit.at(0).tracks.at(1).GetHitArray().size());//weighted
            // vertex vertexMean = newVertexMean;
            //--- test
            lastPoint1 = tvReFit.at(0).tracks.at(0).GetLastPoint();
            lastPoint2 = tvReFit.at(0).tracks.at(1).GetLastPoint();
            XYZVector lastPoint1proj =
               ptOnLine(ransacLine1->GetPatternPar(),
                        lastPoint1); // projection of the last point of the track on the parametric line
            XYZVector lastPoint2proj = ptOnLine(ransacLine2->GetPatternPar(), lastPoint2);
            MaxR1 = lastPoint1proj.Rho();
            MaxR2 = lastPoint2proj.Rho();
            MaxZ1 = lastPoint1proj.Z();
            MaxZ2 = lastPoint2proj.Z();

            // charge1 = tvReFit.at(0).tracks.at(0).GetGeoQEnergy();
            // charge2 = tvReFit.at(0).tracks.at(1).GetGeoQEnergy();

            // for (size_t itvReFit = 0; itvReFit < tvReFit.size(); itvReFit++)
            // std::cout<<"itvReFit "<<itvReFit<<" "<<tvReFit.at(itvReFit).vertex.X()<<"
            // "<<tvReFit.at(itvReFit).vertex.Y()<<" "<<tvReFit.at(itvReFit).vertex.Z()<<" "<<std::endl;
            //	XYZPoint vertexMeanReFit = (XYZPoint)tvReFit.at(0).vertex;

            std::cout << "itvReFit "
                      << " " << vertexMean.X() << " " << vertexMean.Y() << " " << vertexMean.Z() << " " << MaxZ1 << " "
                      << MaxZ2 << " " << std::endl;

            //------------------------------------------------------------------------------

            lastX1 = lastPoint1proj.X();
            lastY1 = lastPoint1proj.Y();
            lastZ1 = lastPoint1proj.Z();
            lastX2 = lastPoint2proj.X();
            lastY2 = lastPoint2proj.Y();
            lastZ2 = lastPoint2proj.Z();
            vertexX = vertexMean.X();
            vertexY = vertexMean.Y();
            vertexZ = vertexMean.Z();

            // theta1 = GetThetaPhi(tvReFit.at(0).tracks.at(0), vertexMean, lastPoint1,1).first;//GetThetaPhi(..,..,-1)
            // for simu;
            // theta2 = GetThetaPhi(tvReFit.at(0).tracks.at(1), vertexMean, lastPoint2,1).first;
            // phi1 = GetThetaPhi(tvReFit.at(0).tracks.at(0), vertexMean, lastPoint1,1).second;
            // phi2 = GetThetaPhi(tvReFit.at(0).tracks.at(1), vertexMean, lastPoint2,1).second;
            theta1 = GetThetaPhi(vertexMean, lastPoint1proj).first; // GetThetaPhi(..,..,-1) for simu;
            theta2 = GetThetaPhi(vertexMean, lastPoint2proj).first;
            phi1 = GetThetaPhi(vertexMean, lastPoint1proj).second;
            phi2 = GetThetaPhi(vertexMean, lastPoint2proj).second;

            // std::vector<Double_t> fitPar1 = tvReFit.at(0).tracks.at(0).GetPattern()->GetPatternPar();
            // std::vector<Double_t> fitPar2 = tvReFit.at(0).tracks.at(1).GetPattern()->GetPatternPar();

            // XYZVector
            // vp1(TMath::Sign(1,lastX1)*fabs(fitPar1[3]),TMath::Sign(1,lastY1)*fabs(fitPar1[4]),TMath::Sign(1,(lastZ1-vertexZ))*fabs(fitPar1[5]));
            // XYZVector
            // vp2(TMath::Sign(1,lastX2)*fabs(fitPar2[3]),TMath::Sign(1,lastY2)*fabs(fitPar2[4]),TMath::Sign(1,(lastZ2-vertexZ))*fabs(fitPar2[5]));
            XYZVector vp1 = lastPoint1proj - vertexMean;
            XYZVector vp2 = lastPoint2proj - vertexMean;
            angle12 = FindAngleBetweenTracks(vp1, vp2);

            // std::cout<<i<<" "<<" protons 1 2 theta : "<<theta1*TMath::RadToDeg()<<"
            // "<<theta2*TMath::RadToDeg()<<"\n"; std::cout<<i<<" protons 1 2 phi : "<<phi1*TMath::RadToDeg()<<"
            // "<<phi2*TMath::RadToDeg()<<"\n";

            range_p1 = tvReFit.at(0).tracks.at(0).GetLinearRange((XYZPoint)vertexMean, (XYZPoint)lastPoint1proj);
            range_p2 = tvReFit.at(0).tracks.at(1).GetLinearRange((XYZPoint)vertexMean, (XYZPoint)lastPoint2proj);

            if (charge1 < 5e3 || charge2 < 5e3 || MaxR1 > 245. || MaxR2 > 245. || MaxR1 < 35. || MaxR2 < 35. ||
                MaxZ1 > 975. || MaxZ2 > 975. || MaxZ1 < 25. || MaxZ2 < 25. || vertexMean.Z() < 25. ||
                vertexMean.Z() > 975.)
               continue;

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

            E_tot_he2 = (proton_mass + eLoss_p1_reco) + (proton_mass + eLoss_p2_reco);
            he2_mass_ex = TMath::Sqrt(E_tot_he2 * E_tot_he2 - mom_He2_reco.Mag2());
            // ex_he2_reco->Fill (he2_mass_ex - he2_mass);
            kin_He2 = TMath::Sqrt(mom_He2_reco.Mag2() + he2_mass_ex * he2_mass_ex) - he2_mass_ex;
            theta_He2 = mom_He2_reco.Theta() * TMath::RadToDeg();

            Double_t mom_beam = sqrt(pow(Ekin_proj + proj_mass, 2) - pow(proj_mass, 2));
            Double_t missing_mom;
            missing_mom = sqrt((mom_beam * beamDir - mom_He2_reco).Mag2());
            Double_t missing_energy = (Ekin_proj + proj_mass + target_mass - (kin_He2 + epsilon_pp + he2_mass + 0.511));
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

            // //------- rotation of track vectors so that Theta and Phi are in the beam frame
            TVector3 momBuff; // dirty trick to use Rotate functions (not available with XYZvector)
            momBuff.SetXYZ(mom_He2_reco.X(), mom_He2_reco.Y(), mom_He2_reco.Z());
            Double_t aRX = TMath::ATan2(beamDir.Y(), beamDir.Z());
            Double_t aRY = TMath::ATan2(-beamDir.X(), beamDir.Z());
            momBuff.RotateX(aRX); // rotate in trigo sens, y to z
            momBuff.RotateY(aRY); // rotate in trigo sens, z to x
            mom_He2_reco.SetXYZ(momBuff.X(), momBuff.Y(), momBuff.Z());

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
         } // for tv size (ive)
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

//====================================================================================
//====================================================================================
//====================================================================================
//====================================================================================
//====================================================================================
//====================================================================================
//====================================================================================
//====================================================================================
//====================================================================================
//====================================================================================

/*



//#include "spline.h"  //cubic interpolation
//#include "TInverseMap.hh"
#include <unistd.h>

TGraph *graphtable;
TSpline3 *splineEloss;
TGraph2D *g2BraggMesh;
Double_t aDVel[300];

//change MTDC gate S800_timeMTDCXf.at(k)>215 && etc..
//change the ObjCorr1C1 ObjCorr1C2
//change the Ekin_proj to the correct value,
//check the energy loss function
//check calibration/correction coef on the dE, and ToF S800.
//If plots make no sens first check that the angles are well calculated (e.g. for real data GetThetaPhi(..,..,1)

static Double_t proton_mass = 1.0078250322 * 931.494 - 0.511;
static Double_t proj_mass = 14.008596359 * 931.494 - 0.511*8.;
static Double_t target_mass = 2.01410177812 * 931.494;
static Double_t recoil_mass = 14.00307400443 * 931.494 - 0.511*7.;
static Double_t he2_mass = 2.0 * proton_mass;
static Double_t Ekin_proj = 105.16* 14.008596359;//115//105.16//107 prop

static TVector3 vfrom(0,0,1);

static Double_t corrGainE1up = 1.;
static Double_t corrGainE1down = 1.;
static double x0_corr_tof = 0.0;
static double afp_corr_tof = 0.0;
static double rf_offset = 19320.0;
static double afp_corr_dE = 0.0;
static double x0_corr_dE = 0.0;
static double bta_corr = 1.0;
static double coeff_hodo[32] = {100000000, 1164, 887, 100000000, 824, 1728, 826,
   100000000, 941, 962, 875, 915, 875, 835, 1079, 842, 855, 870, 891, 1786, 960,
   940, 825, 625, 990, 2195, 805, 870, 575, 1004, 1164, 748 };

std::vector <double> get_invmap_vars(TInverseMap *inv_map, double x0, double y0, double afp, double bfp)
   {
      int order;
      double sinb_sina;
      vector <double> outPuts;

      order = 5;

      outPuts.push_back(inv_map->Ata(order, x0, afp, y0, bfp));//ata
      outPuts.push_back(inv_map->Bta(order, x0, afp, y0, bfp) * bta_corr);//bta
      outPuts.push_back(inv_map->Yta(order, x0, afp, y0, bfp) * 1000.);//yta
      outPuts.push_back(inv_map->Dta(order, x0, afp, y0, bfp));//dta
      outPuts.push_back(atan(sqrt(pow(tan(outPuts.at(0)), 2) + pow(tan(outPuts.at(1)), 2))));//theta_lab
      outPuts.push_back(atan(tan(outPuts.at(1))/tan(outPuts.at(0))));//phi
      if (outPuts.at(0) < 0)
      outPuts.at(5) = 3.141592653 + outPuts.at(5);
      else if (outPuts.at(1) < 0)
      outPuts.at(5) = 2*3.141592653 + outPuts.at(5);

      std::cout<<"inv map out "<<outPuts.at(0)<<" "<<outPuts.at(1)<<" "<<outPuts.at(2)<<" "<<outPuts.at(3)<<"
"<<std::endl; return outPuts;
   }

std::vector <double> get_invmap_vars(TInverseMap *inv_map, double x0, double y0, double afp, double bfp, double z)
   {
      int order;
      double sinb_sina;
      vector <double> outPuts;

      order = 5;

      //std::cout<<"inv map inpt "<<x0<<" "<<afp<<" "<<y0<<" "<<bfp<<" "<<std::endl;

      outPuts.push_back(inv_map->Ata(order, x0, afp, y0, bfp, z));//ata
      outPuts.push_back(inv_map->Bta(order, x0, afp, y0, bfp, z) * bta_corr);//bta
      outPuts.push_back(inv_map->Yta(order, x0, afp, y0, bfp, z) * 1000.);//yta
      outPuts.push_back(inv_map->Dta(order, x0, afp, y0, bfp, z));//dta
      outPuts.push_back(atan(sqrt(pow(tan(outPuts.at(0)), 2) + pow(tan(outPuts.at(1)), 2))));//theta_lab
      outPuts.push_back(atan(tan(outPuts.at(1))/tan(outPuts.at(0))));//phi
      if (outPuts.at(0) < 0)
      outPuts.at(5) = 3.141592653 + outPuts.at(5);
      else if (outPuts.at(1) < 0)
      outPuts.at(5) = 2*3.141592653 + outPuts.at(5);

      //std::cout<<"inv map out "<<outPuts.at(0)<<" "<<outPuts.at(1)<<" "<<outPuts.at(2)<<" "<<outPuts.at(3)<<"
"<<std::endl;

      return outPuts;
   }


Double_t FindAngleBetweenTracks(const TVector3 &vec1,const TVector3 &vec2)
    {
        Double_t ang = vec1.Angle(vec2);
         return ang;
    }


std::pair<Double_t,Double_t> GetNewThetaPhi(std::vector<Double_t> fParFit, const TVector3 &vertex, const TVector3
&maxPos, int zdir)//zdir -1 for simu // +1 for data
{
        std::pair<Double_t,Double_t> thetaPhi;
        if(fParFit.size()>0){
      TVector3 maxPosRot=maxPos;
      //maxPosRot.RotateY(+0.018);//last point coordinates in the beam frame
      //maxPosRot.SetXYZ(maxPosRot.X()+2.5,maxPosRot.Y(),maxPosRot.Z());//+2.5 beam offset
      maxPosRot.SetXYZ(maxPosRot.X()+2.24859,maxPosRot.Y()-0.988841,maxPosRot.Z());//new beam offset
      //TVector3 parFitRot(fParFit[1],fParFit[3],fParFit[5]);
      //parFitRot.RotateY(+0.018);
      //TVector3 beamDir(-0.017997222,0,0.99983804);
      //TVector3 beamDir(-0.00915252,-0.00221017,0.9999557);
      //TVector3 beamDir(-0.0158,0.0,0.99987517);
      TVector3 beamDir(-0.014,0.,0.999902);
      //TVector3 beamDir(-0.02,0.,0.99979998);
      //TVector3 beamDir(0.0,0.,1.);
      //TVector3 beamDir(-0.025,0.,0.99968745);

                TVector3
vp(TMath::Sign(1,maxPos.X())*fabs(fParFit[1]),TMath::Sign(1,maxPos.Y())*fabs(fParFit[3]),zdir*TMath::Sign(1,(maxPos.Z()-vertex.Z()))*fabs(fParFit[5]));//works
with simu TVector3
vpRot(TMath::Sign(1,maxPosRot.X())*fabs(fParFit[1]),TMath::Sign(1,maxPosRot.Y())*fabs(fParFit[3]),zdir*TMath::Sign(1,(maxPosRot.Z()-vertex.Z()))*fabs(fParFit[5]));//works
with simu
                //thetaPhi.first = vp.Theta();//TMath::ATan2(Perp(),fZ)
                //thetaPhi.second = vp.Phi();//TMath::ATan2(fY,fX);
                //thetaPhi.first = beamDir.Angle(vpRot);
      //vpRot.RotateY(+0.018);
                thetaPhi.first = vpRot.Theta();//theta in the beam frame
                thetaPhi.second = TMath::ATan2(vpRot.Y(),vpRot.X());
                //thetaPhi.second = TMath::ATan2(vpRot.X(),vpRot.Y());// phi in the beam frame
        }
        return thetaPhi;
}



void SetERtable(){//fit of the GEANT4 E vs R obtained from the simulation with the function model given by LISE++
                ifstream fER("eLossTables/p_in_d_530torr_SRIM.txt");//from SRIM++,
                Double_t l1=0, l2=0;
                vector <vector<Double_t>> Energy_Range;

                for (string line; getline(fER, line);) {
                        stringstream parse_die(line);
                        vector<Double_t> iRE;
                        parse_die >> l1 >> l2 ;
                        iRE.push_back(l1);//E in MeV
                        iRE.push_back(l2);//mm
                        Energy_Range.push_back(iRE);
                }
                fER.close();
                Int_t v_size = Energy_Range.size();
                Double_t X[v_size];
                Double_t Y[v_size];
                for(Int_t i=0; i<v_size; i++){
                        X[i]=Energy_Range.at(i).at(0)*1.;//0.98
                        Y[i]=Energy_Range.at(i).at(1)*1.;
                        //cout<<X[i]<<" "<<Y[i]<<endl;
                }
                //splineEloss = new TGraph(v_size,Y,X);
      splineEloss = new TSpline3("ElossRange", Y, X, v_size);
        }



// void SetDVelArray(){
// 	TString fileName = "utils/drift_vel_cal_vtxZ_FermiFit.txt";
// 	ifstream fDVel(fileName);
// 	Int_t l1=0;
// 	Double_t l2=0;
// 	for (string line; getline(fDVel, line);) {
// 	  stringstream parse_die(line);
// 	  parse_die >> l1 >> l2;
// 	  aDVel[l1] = l2;
// 	}
// 	fDVel.close();
// }




TVector3 linePt(std::vector<Double_t> par, Double_t paraVar)
{
        TVector3 result(-999,-999,-999);
        if(par.size()!=6) return result;
        Double_t x = par[0] + paraVar*par[1];
        Double_t y = par[2] + paraVar*par[3];
        Double_t z = par[4] + paraVar*par[5];
        result.SetXYZ(x,y,z);
        return result;
}

//returns the projection of a point on the parametric line
//dir is the direction of the parametric line, posOn is a point of the line, posOut is the point that will be projected
TVector3 ptOnLine(TVector3 dir, TVector3 posOn, TVector3 posOut)
{
        TVector3 result(-999,-999,-999);
        TVector3 vop1 = ((dir.Cross(posOut-posOn)).Cross(dir)).Unit();
        Double_t paraVar1 = posOut.Dot(dir.Unit())-posOn.Dot(dir.Unit());
        Double_t paraVar2 = posOn.Dot(vop1)-posOut.Dot(vop1);
        TVector3 vInter1 = posOn + dir.Unit()*paraVar1;
        TVector3 vInter2 = posOut + vop1*paraVar2;
        if((vInter1-vInter2).Mag()<1e-6) result=vInter1;
        return result;
}

//returns the informations for the paramtetric loop needed to scan the range of a track
//starting from firstPt to lastPt and given a certain range step deltaR (in mm)
std::vector<Double_t> lineParaNiter(std::vector<Double_t> par, Double_t deltaR, TVector3 firstPt, TVector3 lastPt)
{
        std::vector<Double_t> result;
        Double_t tmin=0,tmax=0,deltaPar=0.01;
        if(par.size()!=6) return result;
   deltaPar = deltaR/sqrt(pow(par[1],2) + pow(par[3],2) + pow(par[5],2));
        tmin=(firstPt.X()-par[0])/par[1];
   tmax=(lastPt.X()-par[0])/par[1];
   if(tmin>tmax){
      Double_t tBuff=tmax;
      tmax=tmin;
      tmin=tBuff;
   }
   result.push_back((tmax-tmin)/deltaPar);
   result.push_back(deltaPar);
   result.push_back(tmin);
   result.push_back(tmax);
   //std::cout<<" in func "<<deltaPar<<" "<<tmin<<" "<<tmax<<" "<<result.at(0)<<std::endl;
        return result;
}

Double_t lineIntersecHole(std::vector<Double_t> par, std::vector<Double_t> parNiter)
{
        Double_t result;
        Double_t a=0,b=0,c=0,rHole=15.,sol1=0,sol2=0;
        if(par.size()!=6) return result;
   a = pow(par[1],2) + pow(par[3],2);
   b = 2*(par[0]*par[1] + par[2]*par[3]);
   c = pow(par[0],2) + pow(par[2],2) - pow(rHole,2);
   if(b*b-4*a*c>=0){
      sol1=(-b+sqrt(b*b-4*a*c))/(2*a);
      sol2=(-b-sqrt(b*b-4*a*c))/(2*a);
      //std::cout<<"sol "<<sol1<<" "<<sol2<<" "<<parNiter.at(2)<<" "<<parNiter.at(3)<<std::endl;
      if(sol1>parNiter.at(2) && sol1<parNiter.at(3))result=sol1;
      else if(sol2>parNiter.at(2) && sol2<parNiter.at(3))result=sol2;
   }
        // if(tmin>tmax){
        //         Double_t tBuff=tmax;
        //         tmax=tmin;
        //         tmin=tBuff;
        // }
        //result.push_back(tmax);
        //std::cout<<" in func "<<deltaPar<<" "<<tmin<<" "<<tmax<<" "<<result.at(0)<<std::endl;
        return result;
}


TVector3 lineIntersecYPlane(std::vector<Double_t> par, Double_t yPlane)
{
         TVector3 result(-999,-999,-999);
            if(par.size()!=6) return result;
            Double_t paraVar = (yPlane-par[2])/par[3];
            Double_t x = par[0] + paraVar*par[1];
            Double_t z = par[4] + paraVar*par[5];
            result.SetXYZ(x,yPlane,z);
            return result;
}

TVector3 ClosestPoint2Lines(std::vector<Double_t> par1, std::vector<Double_t> par2)
{
   TVector3 p1(par1[0], par1[2], par1[4] );//p1
   TVector3 e1(par1[1], par1[3], par1[5] );//d1
   TVector3 p2(par2[0], par2[2], par2[4] );//p2
   TVector3 e2(par2[1], par2[3], par2[5] );//d2
  TVector3 n1 = e1.Cross(e2.Cross(e1));
  TVector3 n2 = e2.Cross(e1.Cross(e2));
  double t1 = (p2-p1).Dot(n2)/(e1.Dot(n2));
  double t2 = (p1-p2).Dot(n1)/(e2.Dot(n1));
  TVector3 c1 = p1 + t1*e1;
  TVector3 c2 = p2 + t2*e2;
  TVector3 meanpoint = 0.5*(c1+c2);
  return meanpoint;

}

//weighted vertex
TVector3 ClosestPoint2Lines(std::vector<Double_t> par1, std::vector<Double_t> par2, Int_t nHits1, Int_t nHits2)
{
   TVector3 p1(par1[0], par1[2], par1[4] );//p1
   TVector3 e1(par1[1], par1[3], par1[5] );//d1
   TVector3 p2(par2[0], par2[2], par2[4] );//p2
   TVector3 e2(par2[1], par2[3], par2[5] );//d2
  TVector3 n1 = e1.Cross(e2.Cross(e1));
  TVector3 n2 = e2.Cross(e1.Cross(e2));
  double t1 = (p2-p1).Dot(n2)/(e1.Dot(n2));
  double t2 = (p1-p2).Dot(n1)/(e2.Dot(n1));
  TVector3 c1 = p1 + t1*e1;
  TVector3 c2 = p2 + t2*e2;
   Double_t w1 = (Double_t)nHits1/(nHits1+nHits2);
   Double_t w2 = (Double_t)nHits2/(nHits1+nHits2);
  TVector3 meanpoint;
   TVector3 meanpoint1 = w1*c1+w2*c2;
   TVector3 meanpoint2 = 0.5*(c1+c2);
   if((nHits1>8 && nHits2>8) && (nHits1<50 || nHits2<50)) meanpoint = meanpoint1;//if sufficient number of hits use the
not weighted average else meanpoint = meanpoint2; return meanpoint;

}

//closest distance btw 2 parametric lines
Double_t linesDist(std::vector<Double_t> par1, std::vector<Double_t> par2)
{
   Double_t sdist=-1;
        TVector3 p1(par1[0], par1[2], par1[4] );//p1
        TVector3 e1(par1[1], par1[3], par1[5] );//d1
            TVector3 p2(par2[0], par2[2], par2[4] );//p2
        TVector3 e2(par2[1], par2[3], par2[5] );//d2
        TVector3 n = e1.Cross(e2);
        sdist = fabs( n.Dot(p1-p2)/n.Mag() );
   return sdist;
}


// Rotation of a 3D vector around an arbitrary axis
// Rodriges Formula
TVector3 TRANSF(TVector3 from, TVector3 to, TVector3 vin){

        TVector3 vout;
        double n[3];
        double normn, normf, normt;
        double alpha, a, b;


        n[0] = (from.Y()) * (to.Z()) - (from.Z()) * (to.Y());
        n[1] = (from.Z()) * (to.X()) - (from.X()) * (to.Z());
        n[2] = (from.X()) * (to.Y()) - (from.Y()) * (to.X());
        normn = sqrt(pow(n[0],2) + pow(n[1],2) + pow(n[2],2) );
        n[0] = n[0]/normn;
        n[1] = n[1]/normn;
        n[2] = n[2]/normn;

        normf = from.Mag();
        normt = to.Mag();

        alpha = acos((  (from.X()) * (to.X())  + (from.Y()) * (to.Y()) + (from.Z()) * (to.Z())   )/(normf*normt));
        a = sin(alpha);
        b = 1.0 - cos(alpha);

        vout.SetX( (1 - b*(pow(n[2],2) + pow(n[1],2))) * ((vin.X()))   + (-a*n[2] + b*n[0]*n[1]) *((vin.Y()))  + (a*n[1]
+ b*n[0]*n[2] )* ((vin.Z())) ); vout.SetY( (a*n[2] + b*n[0]*n[1]) * ((vin.X()))   + ( 1 - b*(pow(n[2],2) + pow(n[0],2)
)) * ((vin.Y())) +  (-a*n[0] + b*n[1]*n[2]  )   * ((vin.Z())) ); vout.SetZ( ( -a*n[1] + b*n[0]*n[2]) * ((vin.X()))   +
(a*n[0] + b*n[1]*n[2]) * ((vin.Y()))  + (1 -b*(pow(n[1],2) + pow(n[0],2) )  )   * ((vin.Z())) );


        return vout;

}

//distance between a point and a parametric line
Double_t distPtLine(std::vector<Double_t> par, TVector3 pt)
{
        Double_t result=-1;
        if(par.size()!=6) return result;
        TVector3 dir(par[1],par[3],par[5]);
        TVector3 ptLine(par[0],par[2],par[4]);
        result = (ptLine-pt).Cross(dir).Mag()/dir.Mag();
        return result;
}

void ana_d2He(int runNumberS800, int runNumberATTPC)
   {


      //SetERtable_ori();
      SetERtable();
      SetDVelArray();
      //TFile *fileMesh = new TFile("braggMesh_data.root","READ");
      TFile *fileMesh = new TFile("braggMesh.root","READ");
      g2BraggMesh = (TGraph2D*)fileMesh->Get("braggMesh");
                //g2BraggMeshAlpha = (TGraph2D*)fileMeshAlpha->Get("braggMesh");


      FairRunAna* run = new FairRunAna(); //Forcing a dummy run
      ATd2HeAnalysis *d2heana = new ATd2HeAnalysis ();
      ATRansacModExt *ransacExt = new ATRansacModExt ();


      // std::string digiFileName = "run_2021_0030.root";//merged.root
      TString digiFileName = TString::Format("/mnt/analysis/e18008/rootMerg/run_%04d_%04d_exactDVel_new.root",
runNumberS800, runNumberATTPC);
      // TString digiFileName = TString::Format("/mnt/analysis/e18008/rootMerg/run_%04d_%04d.root", runNumberS800,
runNumberATTPC);
      //TString digiFileName = TString::Format("/mnt/analysis/e18008/rootMerg/run_%04d_%04d_exactDVel_new.root",
runNumberS800, runNumberATTPC);
      //TString digiFileName =
TString::Format("/mnt/analysis/e18008/rootMerg/run_%04d_%04d_padsSignalForGatedEvts.root", runNumberS800,
runNumberATTPC);
      //TString digiFileName = TString::Format("/mnt/analysis/e18008/rootMerg/run_%04d_%04d.root", runNumberS800,
runNumberATTPC);
      //TString digiFileName = TString::Format("run_%04d_%04d.root", runNumberS800, runNumberATTPC);

      TFile* file = new TFile(digiFileName,"READ");

      TTree* tree = (TTree*) file -> Get("cbmsim");
      Int_t nEvents = tree -> GetEntries();

      S800Calc *s800cal = new S800Calc();
      TBranch *bS800cal = tree->GetBranch("s800cal");
     bS800cal->SetAddress(&s800cal);

      TTreeReader reader("cbmsim", file);
      TTreeReaderValue<TClonesArray> ransacArray(reader, "ATRansac");
      TTreeReaderValue<TClonesArray> eventArray(reader, "ATEventH");


      TFile* outfile;
      // TString  outFileNameHead = "attpcana_merg_newRansac.root";
      //TString outFileNameHead = TString::Format("/mnt/analysis/e18008/rootAna/runAnalyzed_%04d_%04d.root",
runNumberS800, runNumberATTPC);
      //TString outFileNameHead = TString::Format("/mnt/analysis/e18008/rootAna/runAnalyzed_%04d_%04d_noPIDcut.root",
runNumberS800, runNumberATTPC);
      //TString outFileNameHead = TString::Format("runAnalyzed_%04d_%04d_angleCorr_padSignal.root", runNumberS800,
runNumberATTPC);
      //TString outFileNameHead = TString::Format("runAnalyzed_%04d_%04d_ZEdgesCorr_14Nimap_forBrho.root",
runNumberS800, runNumberATTPC); TString outFileNameHead =
TString::Format("/mnt/analysis/e18008/rootAna/giraud/13Nsetting/RbR/runAnalyzed_%04d_%04d_ZEdgesCorr_corrthetacm.root",
runNumberS800, runNumberATTPC);

      outfile   = TFile::Open(outFileNameHead.Data(),"recreate");
      outfile->cd();//New Bragg
      outfile->mkdir("Bragg");//New Bragg

      TH1F* scatteringAngle = new TH1F("scatteringAngle","scatteringAngle",1000,0,200);
      TH1F* energy = new TH1F("enegy","energy",100,0,40);
      TH2F* ang_vs_energy = new TH2F("ang_vs_energy","ang_vs_energy,",100,0,200,100,0,40);
      //TH2D *tracks_z_r = new TH2D ("tracks_z_r", "ZvsR", 500, -100, 1000, 500, 0, 300);
      //TH2D *tracks_x_y = new TH2D ("tracks_x_y", "XvsY", 500, -300, 300, 500, -300, 300);
      TH1D *theta_r_he2_reco = new TH1D ("theta_r_he2_reco", "theta 2He", 1800, 0, 180);
      TH1D *kin_r_he2_reco = new TH1D ("kin_r_he2_reco", "Energy 2He", 100, 0, 5);
      TH1D *phi_r_he2_reco = new TH1D ("phi_r_he2_reco", "phi 2He", 3600, -180, 180);
      TH2D *theta_kin_he2_reco = new TH2D ("theta_kin_he2_reco", "Kin vs Theta 2He", 1800, 0, 180, 100, 0, 5);
      TH1D *thetacm_he2_reco = new TH1D ("thetacm_he2_reco", "thetacm_he2", 200, 0, 20);
      TH1D *Ex_reco = new TH1D ("Ex_reco", "Ex_reco", 350, -5, 30);
      TH2D *thetacm_Ex_he2_reco = new TH2D ("thetacm_Ex_he2_reco", "thetacm_Ex_he2", 200, 0, 20, 350, -5, 30);
      TH1D *ex_he2_reco = new TH1D ("ex_he2_reco", "ex_he2", 100, 0, 10);
      TH1D *epsilon_pp_reco = new TH1D ("epsilon_pp_reco", "#epsilon_{pp}", 100, 0, 10);
      TH2D *thetacm_epsilon_pp_reco = new TH2D ("thetacm_epsilon_pp_reco", "#theta_{cm} #epsilon_{pp} #^{2}He", 200, 0,
20, 100, 0, 10); TH1D *bragg_mean = new TH1D ("bragg_mean", "bragg_mean", 200, 0, 599); TH1D *bragg_norm = new TH1D
("bragg_norm", "bragg_norm", 301, 0, 300); TH1D *trackProfile = new TH1D ("trackProfile", "trackProfile", 100, 0, 40);
      TH2D *pad_z_signalWidth = new TH2D ("pad_z_signalWidth", "pad_z_signalWidth", 100, 0, 1000, 100, 0, 5);
      TH2D *pad_z_signalHeight = new TH2D ("pad_z_signalHeight", "pad_z_signalHeight", 100, 0, 1000, 100, 0, 2000);

      //----- S800

      // TH2D *tof_dE = new TH2D ("tof_dE", "tof_dE", 250, 1000, 1500, 250, 0, 500);//PID
      TH2D *XfpObj_tof = new TH2D ("XfpObj_tof", "XfpObj_tof", 500,-70,-20,600,250,280);//PID1
      TH2D *ICSum_Obj = new TH2D ("ICSum_Obj", "ICSum_Obj",500,-70,-20,1000,50,750);//PID2
      TH2D *dta_ata = new TH2D ("dta_ata", "dta_ata", 250, -0.25, 0.25, 100, -10, 10);//acceptance
      TH2D *x_y_crdc1 = new TH2D ("x_y_crdc1", "x_dy_crdc1", 300, -300, 300, 150, -150, 150);//positions crdc1
      TH2D *x_y_crdc2 = new TH2D ("x_y_crdc2", "x_dy_crdc2", 300, -300, 300, 150, -150, 150);//positions crdc2

      TH1D *hBr0;
      TH1D *hBrBubble0;
      TH1D *hBrBubble0Err;
      TH1D *hBr1;
      TH1D *hBrBubble1;
      TH1D *hBrBubble1Err;
      TH2D *tracks_z_r;
      TH2D *tracks_x_y;
      TH2D *linePt_z_r;
    TH2D *linePt_x_y;
      TH2D *hDistWithLine1;
      TH2D *hPhiWithLine1;

      //-----
      Int_t ivt = 0,ivt_same=0, irun=runNumberATTPC;
      Double_t range_p1 = 0.,range_p2 =0.;
      Double_t eLoss_p1_reco = 0.0, eLoss_p2_reco = 0.0;
      Double_t epsilon_pp = -999;
      Double_t theta1=0., theta2=0., phi1=0., phi2=0., angle12=0.;
      Double_t thetaNew1=0., thetaNew2=0., phiNew1=0., phiNew2=0.;
      Double_t mom1_norm_reco=0., mom2_norm_reco=0.;
      Double_t E_tot_he2=0., he2_mass_ex=0.;
      Double_t kin_He2=0., theta_He2=0.,kin_He2_same=0., theta_He2_same=0., phi_He2=0.;
      Double_t theta_cm=0., Ex4=0., Ex_reco_same=0.;
      Double_t lastX1=0.,lastX2=0.,lastY1=0.,lastY2=0.,lastZ1=0.,lastZ2=0., vertexX=0., vertexY=0., vertexZ=0.;
      Double_t ata=0., dta=0.;
      ULong64_t S800_timeStamp=0;
      Double_t S800_timeRf=0.,S800_x0=0.,S800_x1=0.,S800_y0=0.,S800_y1=0.,S800_E1up=0.,S800_E1down=0.,S800_tof=0.,
      S800_tofCorr=0.,S800_dE=0.,S800_dECorr=0.,S800_hodoSum=0.,S800_afp=0.,S800_bfp=0.,S800_ata=0.,S800_bta=0.,
      S800_yta=0.,S800_dta=0.,S800_thetaLab=0.,S800_phi=0.,S800_timeE1up=0.,S800_timeE1down=0.,S800_timeE1=0.,S800_timeXf=0.,S800_timeObj=0.;
      Double_t S800_XfObj_tof=0.,S800_ObjCorr=0., S800_Obj=0., S800_XfObjCorr_tof=0.;
      Double_t S800_ICSum=0.;
      Double_t MaxR1, MaxR2, MaxZ1, MaxZ2;
    Int_t CondMTDCObj = 0,CondMTDCXfObj = 0;
      Double_t beam_theta=0., beam_phi=0.;
      Double_t chargeTot1, chargeTot2, chargeAvg1, chargeAvg2, chargeTotBubble1, chargeTotBubble2;
      Int_t Norm=0,normEloss=1;
      Double_t rangeHalfQmax1=0,rangeF1Qmax1=0,rangeF2Qmax1=0,rangeF3Qmax1=0,rangeF4Qmax1=0,rangeF5Qmax1=0;
      Double_t rangeHalfQmax2=0,rangeF1Qmax2=0,rangeF2Qmax2=0,rangeF3Qmax2=0,rangeF4Qmax2=0,rangeF5Qmax2=0;

      Double_t eLoss_p1_reco_bragg=0, eLoss_p2_reco_bragg=0;
      Double_t AscatEje=0,theta_lab=0;
      Double_t momP10=0,momP20=0;
      Double_t Ep1Tot=0,Ep2Tot=0;
      Double_t Etrans=0,invMassPro=0,invMassEje=0;
      Double_t Ex4_new=0,Ex4_new_2=0;
      Double_t check_2HeX=0;
      Double_t range_p1_charge=0,range_p2_charge=0, range_p1_intersec=0, range_p2_intersec=0, range_p1_beamZone=0,
range_p2_beamZone=0; Double_t chargeAvgBubble1=0, chargeAvgBubble2=0; Double_t distLines=-1,distLinesNew=-1; Int_t
pPairSize=0; Int_t hitsBubble1=0, hitsBubble2=0; Double_t avgDistWline1=0,avgDistWline2=0,avgDistWPts1=0,avgDistWPts2=0;
    Int_t nDistWline1=0,nDistWline2=0;
      Double_t best_Ex,best_ix,best_iz,best_iy;
      Double_t z_intersec1=-999,z_intersec2=-999,distVtx1=-999,distVtx2=-999,distVtxMean=-999;
      Double_t ratioDistCoG1_p1=0,ratioDistCoG3_p1=0,ratioDistCoG1_p2=0,ratioDistCoG3_p2=0;
      Int_t nCoG1_1=0, nCoG1_2=0, nCoG1_3=0, nCoG2_1=0, nCoG2_2=0, nCoG2_3=0;
      Double_t range_p1_new=-1,range_p2_new=-1,theta1_new=-999,theta2_new=-999,phi1_new=-999,phi2_new=-999;
      Double_t eLoss_p1_reco_new=-1,eLoss_p2_reco_new=-1;
      Int_t nNullTrackEvt=0, nTwoTrackEvt=0;
      Double_t distVtxFromBeam=-1;
      Double_t avgDistHitFromLine=-1;
      Double_t chi2Fit1=-1, chi2Fit2=-1;
      Double_t chargeRange1=-1,chargeRange2=-1,chargeTotNew1=-1,chargeTotNew2=-1;
      Double_t vertexT=-1;
      Double_t brho=-1;
      Int_t InCondition14N = 0, InCondition12C = 0, InCondition13C = 0, InCondition13N = 0, InCondition10B = 0;

      TVector3 mom_proton1_reco, mom_proton2_reco,mom_He2_reco;
      TVector3 mom_proton1_reco_new, mom_proton2_reco_new;
      TVector3 momProj0, momEje0;
      TVector3 momTrans;
      TVector3 vtxOri, vtxNew, vtxNewPt1, vtxNewPt2;


      vector<TString> fcutPID1File;
      vector<TString> fcutPID2File;
      vector<TString> fcutPID3File;
      vector<TCutG *> fcutPID1;
      vector<TCutG *> fcutPID2;
      vector<TCutG *> fcutPID3;
      Bool_t is=kFALSE;

      //
fcutPID1File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/XfObjObj_run115.root");
      //
fcutPID2File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/ICSumObj_run115.root");
      //fcutPID1File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/XfObj14O.root");
      fcutPID2File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/afpx.root");
      fcutPID3File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/13N.root");
      //
fcutPID3File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/13C.root");
       //
fcutPID3File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/14N.root");
       //
fcutPID3File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/12C.root");
       //
fcutPID3File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/10B.root");

for(Int_t w=0; w<fcutPID1File.size();w++){
TFile f(fcutPID1File[w]);
TIter next(f.GetListOfKeys());
TKey *key;

while ((key=(TKey*)next())) {
      cout<<"PID1 Loading Cut file:  "<<key->GetName()<<endl;
      fcutPID1.push_back( (TCutG*)f.Get(key->GetName()) );
}
}

for(Int_t w=0; w<fcutPID2File.size();w++){
TFile f(fcutPID2File[w]);
TIter next(f.GetListOfKeys());
TKey *key;

while ((key=(TKey*)next())) {
      cout<<"PID2 Loading Cut file:  "<<key->GetName()<<endl;
      fcutPID2.push_back( (TCutG*)f.Get(key->GetName()) );
}
}

for(Int_t w=0; w<fcutPID3File.size();w++){
TFile f(fcutPID3File[w]);
TIter next(f.GetListOfKeys());
TKey *key;

while ((key=(TKey*)next())) {
      cout<<"PID3 Loading Cut file:  "<<key->GetName()<<endl;
      fcutPID3.push_back( (TCutG*)f.Get(key->GetName()) );
}
}





      TTree *anatree = new TTree("anatree","new TTree");

      anatree->Branch("ivt",&ivt);
      anatree->Branch("irun",&irun);
      anatree->Branch("pPairSize",&pPairSize);
      anatree->Branch("range_p1",&range_p1);
      anatree->Branch("rangeHalfQmax1",&rangeHalfQmax1);
      anatree->Branch("range_p2",&range_p2);
      anatree->Branch("theta1",&theta1);
      anatree->Branch("theta2",&theta2);
      anatree->Branch("phi1",&phi1);
      anatree->Branch("phi2",&phi2);
      anatree->Branch("thetaNew1",&thetaNew1);
      anatree->Branch("thetaNew2",&thetaNew2);
      anatree->Branch("phiNew1",&phiNew1);
      anatree->Branch("phiNew2",&phiNew2);
      anatree->Branch("lastX1",&lastX1);
      anatree->Branch("lastY1",&lastY1);
      anatree->Branch("lastZ1",&lastZ1);
      anatree->Branch("lastX2",&lastX2);
      anatree->Branch("lastY2",&lastY2);
      anatree->Branch("lastZ2",&lastZ2);
      anatree->Branch("chargeTot1",&chargeTot1);
      anatree->Branch("chargeTot2",&chargeTot2);
      anatree->Branch("chargeTotBubble1",&chargeTotBubble1);
      anatree->Branch("chargeTotBubble2",&chargeTotBubble2);
      anatree->Branch("chargeAvgBubble1",&chargeAvgBubble1);
      anatree->Branch("chargeAvgBubble2",&chargeAvgBubble2);
      anatree->Branch("chargeAvg1",&chargeAvg1);
      anatree->Branch("chargeAvg2",&chargeAvg2);
      anatree->Branch("mom_proton1_reco",&mom_proton1_reco);
      anatree->Branch("mom_proton1_reco_new",&mom_proton1_reco_new);
      anatree->Branch("mom_proton2_reco",&mom_proton2_reco);
      anatree->Branch("mom_proton2_reco_new",&mom_proton2_reco_new);
      anatree->Branch("vertexX",&vertexX);
      anatree->Branch("vertexY",&vertexY);
      anatree->Branch("vertexZ",&vertexZ);
      anatree->Branch("beam_theta",&beam_theta);
      anatree->Branch("beam_phi",&beam_phi);
      anatree->Branch("angle12",&angle12);
      anatree->Branch("eLoss_p1_reco",&eLoss_p1_reco);
      anatree->Branch("eLoss_p1_reco_bragg",&eLoss_p1_reco_bragg);
      anatree->Branch("eLoss_p2_reco",&eLoss_p2_reco);
      anatree->Branch("eLoss_p2_reco_bragg",&eLoss_p2_reco_bragg);
      anatree->Branch("kin_He2",&kin_He2);
      anatree->Branch("theta_He2",&theta_He2);
      anatree->Branch("phi_He2",&phi_He2);
      anatree->Branch("E_tot_he2",&E_tot_he2);
      anatree->Branch("mom_He2_reco",&mom_He2_reco);
      anatree->Branch("he2_mass_ex",&he2_mass_ex);
      anatree->Branch("theta_cm",&theta_cm);
      anatree->Branch("Ex4",&Ex4);
      anatree->Branch("epsilon_pp",&epsilon_pp);
      anatree->Branch("mom1_norm_reco",&mom1_norm_reco);
      anatree->Branch("mom2_norm_reco",&mom2_norm_reco);
      anatree->Branch("normEloss",&normEloss);
      anatree->Branch("momP10",&momP10);
      anatree->Branch("momP20",&momP20);
      anatree->Branch("momProj0",&momProj0);
      anatree->Branch("momEje0",&momEje0);
      anatree->Branch("Etrans",&Etrans);
      anatree->Branch("momTrans",&momTrans);
      anatree->Branch("invMassPro",&invMassPro);
      anatree->Branch("invMassEje",&invMassEje);
      anatree->Branch("Ex4_new",&Ex4_new);
      anatree->Branch("Ex4_new_2",&Ex4_new_2);
      anatree->Branch("check_2HeX",&check_2HeX);
      anatree->Branch("range_p1_intersec",&range_p1_intersec);
      anatree->Branch("range_p2_intersec",&range_p2_intersec);
      anatree->Branch("range_p1_beamZone",&range_p1_beamZone);
      anatree->Branch("range_p2_beamZone",&range_p2_beamZone);
      anatree->Branch("range_p1_charge",&range_p1_charge);
      anatree->Branch("range_p2_charge",&range_p2_charge);
      anatree->Branch("distLines",&distLines);
      anatree->Branch("hitsBubble1",&hitsBubble1);
      anatree->Branch("hitsBubble2",&hitsBubble2);
      anatree->Branch("best_Ex",&best_Ex);
      anatree->Branch("best_ix",&best_ix);
      anatree->Branch("best_iy",&best_iy);
      anatree->Branch("best_iz",&best_iz);
      anatree->Branch("z_intersec1",&z_intersec1);
      anatree->Branch("z_intersec2",&z_intersec2);
      anatree->Branch("distVtx1",&distVtx1);
      anatree->Branch("distVtx2",&distVtx2);
      anatree->Branch("distVtxMean",&distVtxMean);
      anatree->Branch("vtxOri",&vtxOri);
      anatree->Branch("vtxNew",&vtxNew);
      anatree->Branch("range_p1_new",&range_p1_new);
      anatree->Branch("range_p2_new",&range_p2_new);
      anatree->Branch("theta1_new",&theta1_new);
      anatree->Branch("theta2_new",&theta2_new);
      anatree->Branch("phi1_new",&phi1_new);
      anatree->Branch("phi2_new",&phi2_new);
      anatree->Branch("distLinesNew",&distLinesNew);
      anatree->Branch("distVtxFromBeam",&distVtxFromBeam);
      anatree->Branch("avgDistHitFromLine",&avgDistHitFromLine);
      anatree->Branch("chi2Fit1",&chi2Fit1);
      anatree->Branch("chi2Fit2",&chi2Fit2);
      anatree->Branch("chargeRange1",&chargeRange1);
      anatree->Branch("chargeRange2",&chargeRange2);
      anatree->Branch("chargeTotNew1",&chargeTotNew1);
      anatree->Branch("chargeTotNew2",&chargeTotNew2);
      anatree->Branch("vertexT",&vertexT);
      anatree->Branch("MaxR1",&MaxR1);
      anatree->Branch("MaxR2",&MaxR2);
      anatree->Branch("MaxZ1",&MaxZ1);
      anatree->Branch("MaxZ2",&MaxZ2);



      //----- S800
      anatree->Branch("S800_timeStamp",&S800_timeStamp,"S800_timeStamp/l");
      //anatree->Branch("S800_timeRf",&S800_timeRf);
      //anatree->Branch("S800_timeE1up",&S800_timeE1up);
      //anatree->Branch("S800_timeE1down",&S800_timeE1down);
      //anatree->Branch("S800_timeE1",&S800_timeE1);
      anatree->Branch("S800_timeXf",&S800_timeXf);
      anatree->Branch("S800_timeObj",&S800_timeObj);
      anatree->Branch("S800_tof",&S800_tof);
      anatree->Branch("S800_XfObj_tof",&S800_XfObj_tof);
      anatree->Branch("S800_ObjCorr",&S800_ObjCorr);
      anatree->Branch("S800_Obj",&S800_Obj);
      anatree->Branch("CondMTDCObj",&CondMTDCObj);
      anatree->Branch("CondMTDCXfObj",&CondMTDCXfObj);
      anatree->Branch("S800_ICSum",&S800_ICSum);

      anatree->Branch("S800_x0",&S800_x0);
      anatree->Branch("S800_x1",&S800_x1);
      anatree->Branch("S800_y0",&S800_y0);
      anatree->Branch("S800_y1",&S800_y1);
      anatree->Branch("S800_E1up",&S800_E1up);
      anatree->Branch("S800_E1down",&S800_E1down);
      anatree->Branch("S800_tof",&S800_tof);
      anatree->Branch("S800_tofCorr",&S800_tofCorr);
      anatree->Branch("S800_dE",&S800_dE);
      anatree->Branch("S800_dECorr",&S800_dECorr);
      anatree->Branch("S800_hodoSum",&S800_hodoSum);
      anatree->Branch("S800_afp",&S800_afp);
      anatree->Branch("S800_bfp",&S800_bfp);
      anatree->Branch("S800_ata",&S800_ata);
      anatree->Branch("S800_bta",&S800_bta);
      anatree->Branch("S800_yta",&S800_yta);
      anatree->Branch("S800_dta",&S800_dta);
      anatree->Branch("S800_thetaLab",&S800_thetaLab);
      anatree->Branch("S800_phi",&S800_phi);
      anatree->Branch("AscatEje",&AscatEje);
      anatree->Branch("theta_lab",&theta_lab);
      anatree->Branch("brho",&brho);
      anatree->Branch("InCondition14N",&InCondition14N);
      anatree->Branch("InCondition13N",&InCondition13N);
      anatree->Branch("InCondition13C",&InCondition13C);
      anatree->Branch("InCondition12C",&InCondition12C);
      anatree->Branch("InCondition10B",&InCondition10B);

      std::vector<ATTrack> trackCand;


      //----------------------- S800 -------------------------------------------------
      std::vector< std::string > mapList;

                // mapList.push_back("invMap/invmap_14N/invmap_-05.inv");
                // mapList.push_back("invMap/invmap_14N/invmap_-04.inv");
                // mapList.push_back("invMap/invmap_14N/invmap_-03.inv");
                // mapList.push_back("invMap/invmap_14N/invmap_-02.inv");
                // mapList.push_back("invMap/invmap_14N/invmap_-01.inv");
                // mapList.push_back("invMap/invmap_14N/invmap_00.inv");
                // mapList.push_back("invMap/invmap_14N/invmap_01.inv");
                // mapList.push_back("invMap/invmap_14N/invmap_02.inv");
                // mapList.push_back("invMap/invmap_14N/invmap_03.inv");
                // mapList.push_back("invMap/invmap_14N/invmap_04.inv");
                // mapList.push_back("invMap/invmap_14N/invmap_05.inv");
                // mapList.push_back("invMap/invmap_14N/invmap_06.inv");
                // mapList.push_back("invMap/invmap_14N/invmap_07.inv");
                // mapList.push_back("invMap/invmap_14N/invmap_08.inv");
                // mapList.push_back("invMap/invmap_14N/invmap_09.inv");
                // mapList.push_back("invMap/invmap_14N/invmap_10.inv");


      mapList.push_back("invMap/invmap_14N/invmap_-05.inv");
                mapList.push_back("invMap/invmap_14N/invmap_-04.inv");
                mapList.push_back("invMap/invmap_14N/invmap_-03.inv");
                mapList.push_back("invMap/invmap_14N/invmap_-02.inv");
                mapList.push_back("invMap/invmap_14N/invmap_-01.inv");
                mapList.push_back("invMap/invmap_14N/invmap_00.inv");
                mapList.push_back("invMap/invmap_14N/invmap_01.inv");
                mapList.push_back("invMap/invmap_14N/invmap_02.inv");
                mapList.push_back("invMap/invmap_14N/invmap_03.inv");
                mapList.push_back("invMap/invmap_14N/invmap_04.inv");
                mapList.push_back("invMap/invmap_14N/invmap_05.inv");
                mapList.push_back("invMap/invmap_14N/invmap_06.inv");
                mapList.push_back("invMap/invmap_14N/invmap_07.inv");
                mapList.push_back("invMap/invmap_14N/invmap_08.inv");
                mapList.push_back("invMap/invmap_14N/invmap_09.inv");
                mapList.push_back("invMap/invmap_14N/invmap_10.inv");


      std::vector<Double_t> mapDist;
      for(int i=0;i<mapList.size();i++){
         mapDist.push_back(-0.5+0.1*(i));
      }

      //inv_map->SetDistPivotTarget(mapDist);
      //std::string mapFile="inv_map.inv";
      //TInverseMap *inv_map = new TInverseMap(mapFile.c_str());
      //TInverseMap *inv_map = new TInverseMap(mapList);
TInverseMap *inv_map = new TInverseMap();
//TInverseMap *inv_map = new TInverseMap("invMap/invmap_14N/invmap_00.inv");
   //TInverseMap *inv_map = new TInverseMap("invMap/invmap_14N/invmap_10.inv");
inv_map->SetDistPivotTarget(mapDist);
inv_map->ReadMultiMapFile(mapList);
      std::cout<<" mapDist "<<mapDist.size()<<" "<<mapDist.at(2)<<std::endl;


      /// --------------------- Event loop -------------------------------------------

      for(Int_t i=0;i<nEvents;i++){
      //for(Int_t i=0;i<1000;i++){

         s800cal->Clear();
         trackCand.clear();
         bS800cal->GetEntry(i);
         reader.Next();

         S800_timeStamp=0;
         S800_timeRf=0.;S800_x0=0.;S800_x1=0.;S800_y0=0.;S800_y1=0.;S800_E1up=0.;S800_E1down=0.;S800_tof=0.;
         S800_tofCorr=0.;S800_dE=0.;S800_dECorr=0.;S800_hodoSum=0.;S800_afp=0.;S800_bfp=0.;S800_ata=0.;
         S800_bta=0.;S800_yta=0.;S800_dta=0.;S800_thetaLab=0.;S800_phi=0.;
         S800_timeE1up=0.;S800_timeE1down=0.;S800_timeE1=0.;S800_timeXf=0.;S800_timeObj=0.;
         S800_tof=0.;



         TVector3 Pejec;
         TVector3 pfirst;
         TVector3 plast;
         TVector3 Pproj(0,0,1);
         Double_t Anejec;
         Double_t Anp1;
         Double_t Anp2;
         Double_t Eeje=0;
         Double_t Ep1=0;
         Double_t Ep2=0;
         Bool_t fl1 = kTRUE;
         Bool_t fl2 = kTRUE;



         // ATRANSACN::ATRansac* fATRansac  = dynamic_cast<ATRANSACN::ATRansac*> (ransacArray->At(0));
         ATRansacMod* fATRansac  = dynamic_cast<ATRansacMod*> (ransacArray->At(0));
        // ATMlesacMod* fATRansac  = dynamic_cast<ATMlesacMod*> (ransacArray->At(0));
        //ATLmedsMod* fATRansac  = dynamic_cast<ATLmedsMod*> (ransacArray->At(0));
          if(fATRansac==nullptr){
           std::cout<<" Null pointer fATRansac "<<"\n";
            continue;
            }

         trackCand = fATRansac->GetTrackCand();


         ATEvent* event = (ATEvent*) eventArray->At(0);
                        Int_t nHitsEvent = event->GetNumHits();

         //if(trackCand.size()>1 && s800cal->GetIsInCut()==kTRUE){
         if(trackCand.size()>1){


            //----------------------- S800 -------------------------------------------------

    CondMTDCObj = 0;
    CondMTDCXfObj = 0;
      Int_t InCondition1 = 0;
      Int_t InCondition2 = 0;
      Int_t InCondition3 = 0;
      InCondition14N = 0; InCondition12C = 0; InCondition13C = 0; InCondition13N = 0; InCondition10B = 0;
    vector<Float_t> S800_timeMTDCObj = s800cal->GetMultiHitTOF()->GetMTDCObj();
    vector<Float_t> S800_timeMTDCXf = s800cal->GetMultiHitTOF()->GetMTDCXf();
    Float_t S800_timeObjSelect=-999;
    Float_t S800_timeXfSelect=-999;
    S800_XfObj_tof = -999.;
      S800_XfObjCorr_tof = -999.;
    S800_ObjCorr = -999;
    S800_Obj = -999;
    Double_t ObjCorr1C1 = 70.; //70
    Double_t ObjCorr1C2 = 0.0085; //0.0085

    S800_ICSum = s800cal->GetIC()->GetSum();
    S800_x0 = s800cal->GetCRDC(0)->GetX();
    S800_x1 = s800cal->GetCRDC(1)->GetX();
    S800_y0 = s800cal->GetCRDC(0)->GetY();//-4
    S800_y1 = s800cal->GetCRDC(1)->GetY();//-7
    //S800_afp = atan( (S800_x1-S800_x0)/1073. )+0.009;
    //S800_bfp = atan( (S800_y1-S800_y0)/1073. )+0.0022;
    S800_afp = atan( (S800_x1-S800_x0)/1073. );
    S800_bfp = atan( (S800_y1-S800_y0)/1073. );

    for(int k=0; k<S800_timeMTDCXf.size(); k++){
      if(S800_timeMTDCXf.at(k)>160 && S800_timeMTDCXf.at(k)<240) S800_timeXfSelect=S800_timeMTDCXf.at(k);
    }
    for(int k=0; k<S800_timeMTDCObj.size(); k++){
      if(S800_timeMTDCObj.at(k)>-120 && S800_timeMTDCObj.at(k)<-20) S800_timeObjSelect=S800_timeMTDCObj.at(k);
    }

      S800_XfObj_tof = -999;
    S800_XfObjCorr_tof = -999;
    S800_ObjCorr = -999;
    if(S800_timeObjSelect!=-999) {
        S800_ObjCorr = S800_timeObjSelect + ObjCorr1C1*S800_afp + ObjCorr1C2*S800_x0;
        CondMTDCObj=1;
    }
    if(S800_timeXfSelect!=-999 && S800_timeObjSelect!=-999) {
        S800_XfObj_tof=S800_timeXfSelect-S800_timeObjSelect;
        S800_XfObjCorr_tof=S800_timeXfSelect-S800_ObjCorr;
        CondMTDCXfObj=1;
    }

      if(std::isnan(S800_ObjCorr)==1 || std::isnan(S800_XfObj_tof)==1 || std::isnan(S800_XfObjCorr_tof)==1 ||
std::isnan(S800_ICSum)==1) continue;
      //for(Int_t w=0; w<fcutPID1.size();w++) if(CondMTDCXfObj==1 && fcutPID1[w]->IsInside(S800_ObjCorr,S800_XfObj_tof))
InCondition1 += 1; //or of PID1 if(CondMTDCXfObj==1 && S800_XfObj_tof>268 && S800_XfObj_tof<275) InCondition1 += 1;
//14O beam for(Int_t w=0; w<fcutPID2.size();w++) if(CondMTDCXfObj==1 && fcutPID2[w]->IsInside(S800_x0,S800_afp) )
InCondition2 += 1; //or of PID2
      // for(Int_t w=0; w<fcutPID3.size();w++) if(CondMTDCXfObj==1 && fcutPID3[w]->IsInside(S800_ObjCorr,S800_ICSum) )
InCondition3 += 1; //or of PID3 for(Int_t w=0; w<fcutPID3.size();w++) if(CondMTDCXfObj==1 &&
fcutPID3[w]->IsInside(S800_ObjCorr,S800_ICSum) ){ InCondition3 += 1; //or of PID3
         // if(w==0) InCondition13C =1;
         if(w==0) InCondition13N =1;
         // if(w==0) InCondition14N =1;
         // else if(w==1) InCondition12C =1;
         // else if(w==2) InCondition10B =1;
      }

       // std::cout <<" Number of TCutG files  "<<CondMTDCXfObj<<" "<<S800_XfObj_tof<<" "<<S800_ObjCorr<<"
"<<InCondition1<<" "<<InCondition2<<" "<<S800_timeObjSelect<<" "<<S800_ICSum<< '\n';

       Int_t AndofCon = InCondition1*InCondition2*InCondition3;
       //Int_t AndofCon = InCondition1*InCondition2;
         is=kFALSE;

        if(std::isnan(S800_ObjCorr)==0)
        if(InCondition1){
         // std::cout<<"TRUE !"<<std::endl;
          //atree->Fill();
        }
        if(std::isnan(S800_ObjCorr)==0)
        if(AndofCon){
          is=kTRUE;
        }


         if(!is) continue;

            S800_timeStamp = s800cal->GetTS();
           S800_E1up = s800cal->GetSCINT(0)->GetDEup();
           S800_E1down = s800cal->GetSCINT(0)->GetDEdown();
            S800_dE = sqrt( (corrGainE1up*S800_E1up) * (corrGainE1down* S800_E1down ) );
            S800_dECorr = S800_dE + afp_corr_dE*S800_afp + x0_corr_dE*fabs(S800_x0);
            for (Int_t j=0; j<32; j++) if (s800cal->GetHODOSCOPE(j)->GetEnergy()>=10 &&
s800cal->GetHODOSCOPE(j)->GetEnergy()<=4000) S800_hodoSum += s800cal->GetHODOSCOPE(j)->GetEnergy()*3000./coeff_hodo[j];

            ICSum_Obj->Fill(S800_ObjCorr,S800_ICSum);
               XfpObj_tof->Fill(S800_XfObj_tof,S800_ObjCorr);

            x_y_crdc1->Fill(S800_x0,S800_y0);
            x_y_crdc2->Fill(S800_x1,S800_y1);

             std::cout <<"trackSize  "<<trackCand.size()<< '\n';


            std::vector< std::pair <int,int> >  two_p_pair;
            two_p_pair.clear();

            //multi track analysis
            for(Int_t w=0;w<trackCand.size()-1;w++){
                  TVector3 vertex0 = trackCand.at(w).GetTrackVertex();
                  for(Int_t z=w+1; z<trackCand.size();z++){
                           TVector3 vertex1 = trackCand.at(z).GetTrackVertex();
                           if(vertex0==vertex1) two_p_pair.push_back(std::make_pair(w,z));
                     }
               }



            // //only 2 protons per event
            // two_p_pair.clear();
            // two_p_pair.push_back(std::make_pair(0,1));
            //
            chargeTot1=0;
            chargeTot2=0;
            chargeTotBubble1=0;
            chargeTotBubble2=0;
            chargeAvgBubble1=0;
            chargeAvgBubble2=0;
            chargeAvg1=0;
            chargeAvg2=0;
            pPairSize=two_p_pair.size();

            if(pPairSize<1) continue;

            for(Int_t w=0;w<pPairSize;w++){


               theta1=0.; theta2=0.; phi1=0.; phi2=0.; range_p1=0.; range_p2=0.; eLoss_p1_reco=0.; eLoss_p2_reco=0.;
mom1_norm_reco=0.; mom2_norm_reco=0.; //reset variables E_tot_he2=0.; he2_mass_ex=0.; kin_He2=0.; theta_He2=0.;
phi_He2=0.;theta_cm=0.; Ex4=0.; MaxR1=0.; MaxR2=0.; MaxZ1=0.; MaxZ2=0.; beam_theta=0.; beam_phi=0.; chargeTotNew1=-1;
chargeTotNew2=-1; chargeRange1=-1; chargeRange2=-1;
                                //rangeHalfQmax1=0;
                                rangeF1Qmax1=0;rangeF2Qmax1=0;rangeF3Qmax1=0;rangeF4Qmax1=0;rangeF5Qmax1=0;
            eLoss_p1_reco_bragg=0; eLoss_p2_reco_bragg=0;
            thetaNew1=0.; thetaNew2=0.; phiNew1=0.; phiNew2=0.;
            theta_lab=0; AscatEje=0;
            momP10=0;momP20=0;
            Ep1Tot=0;Ep2Tot=0;
            Etrans=0;invMassPro=0;invMassEje=0;
            Ex4_new=0;Ex4_new_2=0;
            check_2HeX=0;
            hitsBubble1=0; hitsBubble2=0;
            avgDistWline1=0;avgDistWPts1=0;nDistWline1=0;avgDistWline2=0;;avgDistWPts2=0;nDistWline2=0;
            ratioDistCoG1_p1=0.;ratioDistCoG3_p1=0.;
            ratioDistCoG1_p2=0.;ratioDistCoG3_p2=0.;
            distVtxFromBeam=0;avgDistHitFromLine=-1;
            chi2Fit1=-1;chi2Fit2=-1;
            vertexT=0.;
            //chargeTot1=0;chargeTot2=0;chargeAvg1=0;chargeAvg2=0;

               TVector3 vertexMean = trackCand.at(two_p_pair[w].first).GetTrackVertex();
            TVector3 lastPoint1 = trackCand.at(two_p_pair[w].first).GetLastPoint();
            TVector3 lastPoint2 = trackCand.at(two_p_pair[w].second).GetLastPoint();
            TVector3 firstPoint1 = trackCand.at(two_p_pair[w].first).GetFirstPoint();
            TVector3 firstPoint2 = trackCand.at(two_p_pair[w].second).GetFirstPoint();
            MaxR1=sqrt(pow(lastPoint1.X(),2)+pow(lastPoint1.Y(),2));
            MaxR2=sqrt(pow(lastPoint2.X(),2)+pow(lastPoint2.Y(),2));
            MaxZ1=lastPoint1.Z();
            MaxZ2=lastPoint2.Z();

      lastX1 = lastPoint1.X();
                lastY1 = lastPoint1.Y();
                lastZ1 = lastPoint1.Z();
                lastX2 = lastPoint2.X();
                lastY2 = lastPoint2.Y();
                lastZ2 = lastPoint2.Z();
                vertexX = vertexMean.X();
                vertexY = vertexMean.Y();
                vertexZ = vertexMean.Z();

      std::vector<Double_t> fitPar1 = trackCand.at(two_p_pair[w].first).GetFitPar();
                std::vector<Double_t> fitPar2 = trackCand.at(two_p_pair[w].second).GetFitPar();

      TVector3
vp1(TMath::Sign(1,lastX1)*fabs(fitPar1[1]),TMath::Sign(1,lastY1)*fabs(fitPar1[3]),TMath::Sign(1,(lastZ1-vertexZ))*fabs(fitPar1[5]));//direction
of the parametric line TVector3
vp2(TMath::Sign(1,lastX2)*fabs(fitPar2[1]),TMath::Sign(1,lastY2)*fabs(fitPar2[3]),TMath::Sign(1,(lastZ2-vertexZ))*fabs(fitPar2[5]));
                TVector3 v11(fitPar1[0],fitPar1[2],fitPar1[4]);//point on the parametric line
                TVector3 v12(fitPar2[0],fitPar2[2],fitPar2[4]);

      TVector3 vtxPoint1proj=ptOnLine(vp1,v11,vertexMean);//projection of the vertex on the parametric line
      TVector3 lastPoint1proj=ptOnLine(vp1,v11,lastPoint1);//projection of the last point of the track on the parametric
line TVector3 vtxPoint2proj=ptOnLine(vp2,v12,vertexMean); TVector3 lastPoint2proj=ptOnLine(vp2,v12,lastPoint2);

         std::vector<Double_t> loopPar1 = lineParaNiter(fitPar1,5,vtxPoint1proj,lastPoint1proj);//return the loop
parameters to scan the range of track given dR (second argument in mm) std::vector<Double_t> loopPar2 =
lineParaNiter(fitPar2,5,vtxPoint2proj,lastPoint2proj);

      TVector3 vTrack1=lastPoint1proj-vertexMean;
      TVector3 vTrack2=lastPoint2proj-vertexMean;

      Double_t tIntersec1 =lineIntersecHole(fitPar1, loopPar1);
      Double_t tIntersec2 =lineIntersecHole(fitPar2, loopPar2);

                TVector3 posIntersec1 = linePt(fitPar1,tIntersec1);
                TVector3 posIntersec2 = linePt(fitPar2,tIntersec2);
      range_p1_intersec = (posIntersec1-lastPoint1).Mag();
      range_p2_intersec = (posIntersec2-lastPoint2).Mag();
      range_p1_beamZone = (posIntersec1-vertexMean).Mag();
      range_p2_beamZone = (posIntersec2-vertexMean).Mag();
      range_p1_charge = (firstPoint1-lastPoint1).Mag();
      range_p2_charge = (firstPoint2-lastPoint2).Mag();

         range_p1 = trackCand.at(two_p_pair[w].first).GetLinearRange(vertexMean,lastPoint1proj);
         range_p2 = trackCand.at(two_p_pair[w].second).GetLinearRange(vertexMean,lastPoint2proj);

      distLines = linesDist(fitPar1,fitPar2);

                        // theta1 = trackCand.at(two_p_pair[w].first).GetThetaPhi(vertexMean,
lastPoint1proj,1).first;//GetThetaPhi(..,..,1) for data
                // theta2 = trackCand.at(two_p_pair[w].second).GetThetaPhi(vertexMean,
lastPoint2proj,1).first;//GetThetaPhi(..,..,1) for data
                // phi1 = trackCand.at(two_p_pair[w].first).GetThetaPhi(vertexMean,
lastPoint1proj,1).second;//GetThetaPhi(..,..,1) for data
                // phi2 = trackCand.at(two_p_pair[w].second).GetThetaPhi(vertexMean,
lastPoint2proj,1).second;//GetThetaPhi(..,..,1) for data



                        std::vector<ATHit>* hitArray1 = trackCand.at(two_p_pair[w].first).GetHitArray();
                        Int_t nHits1 = hitArray1->size();
                        Double_t Tcharge1=0;
                        for(Int_t iHit=0; iHit<nHits1; iHit++){
                        ATHit hit = hitArray1->at(iHit);
                        TVector3 position = hit.GetPosition();
                        Double_t tq = hit.GetCharge();
                           TVector3 rangePoint =  position - vertexMean;
                           Double_t range = rangePoint.Mag();
                           Tcharge1 += tq;
                      }
                        chargeTot1 = Tcharge1;
                        //chargeAvg1 = Tcharge1/range_p1;
                        chargeAvg1 = Tcharge1/nHits1;

                        std::vector<ATHit>* hitArray2 = trackCand.at(two_p_pair[w].second).GetHitArray();
                        Int_t nHits2 = hitArray2->size();
                        Double_t Tcharge2=0;
                        for(Int_t iHit=0; iHit<nHits2; iHit++){
                           ATHit hit = hitArray2->at(iHit);
                           TVector3 position = hit.GetPosition();
                           Double_t tq = hit.GetCharge();
                           TVector3 rangePoint =  position - vertexMean;
                           Double_t range = rangePoint.Mag();
                           Tcharge2 += tq;
                        }
                        chargeTot2 = Tcharge2;
                        //chargeAvg2 = Tcharge2/range_p2;
                        chargeAvg2 = Tcharge2/nHits2;

//------------------------------------------

//TVector3 beamDir(-0.014,0.,0.999902);
//TVector3 beamDir(-0.00915252,-0.00221017,0.9999557);
TVector3 beamDir(-0.00915252,-0.00221017,0.9999557);//60-114
//TVector3 beamDir(-0.008,-0.0025,0.99996487);//27-1-9
//TVector3 beamDir(-0.01,0.,0.99995);
//TVector3 beamDir(-0.012,0.,0.999928);//already nice but E=0 not quite at 0
//TVector3 beamDir(-0.011,0.,0.9999395);//already nice but E=0 not quite at 0
//TVector3 beamDir(-0.005,-0.00,0.9999875);//already nice but E=0 not quite at 0
std::vector<Double_t> beamPar;
beamPar.push_back(-3.);
beamPar.push_back(-0.00915252);
beamPar.push_back(2.);
beamPar.push_back(-0.00221017);
beamPar.push_back(0.);
beamPar.push_back(0.9999557);



//-----------------------------------------------------------------------------

std::vector<ATHit> hitArrayRansac;
//std::vector<ATHit> hitArrayRansac2;
for(Int_t iHit=0; iHit<nHitsEvent; iHit++){
                                                         ATHit hit = event->GetHit(iHit);
                                                         TVector3 position = hit.GetPosition();
                                                         hit.SetPosition(position.X(),position.Y(),(1000./aDVel[irun])*(position.Z()));//
                                                         //hit.SetPosition(position.X(),position.Y(),(1000./800.)*(position.Z()));//
                                                         Double_t distWline1Pt = distPtLine(fitPar1,position);
                                                         Double_t distWline2Pt = distPtLine(fitPar2,position);
                                                         TVector3 point1Proj=ptOnLine(vp1,v11,position);
                                                         TVector3 point2Proj=ptOnLine(vp2,v12,position);
                                                         Double_t tq = hit.GetCharge();
                                                         if(distWline1Pt<12.0){
                                                                  //if((point1Proj-vertexMean).Mag()>0.25*range_p1){
                                                               //
if(sqrt(pow(position.X(),2)+pow(position.Y(),2))>35.)chargeTot1+=tq;
                                                                  if(sqrt(pow(point1Proj.X(),2)+pow(point1Proj.Y(),2))>35.){
                                                                        hitArrayRansac.push_back(hit);
                                                                  }
                                                         }
                                                         if(distWline2Pt<12.0){
                                                               //
if(sqrt(pow(position.X(),2)+pow(position.Y(),2))>35.)chargeTot2=+tq;
                                                               //if((point2Proj-vertexMean).Mag()>0.25*range_p2){
                                                                  if(sqrt(pow(point2Proj.X(),2)+pow(point2Proj.Y(),2))>35.){
                                                                        hitArrayRansac.push_back(hit);
                                                                  }
                                                                  //	}
                                                         }
                                             //      if(distWlinePt<15.0 &&
TMath::Sign(lastX1,positionLinePt.X())!=lastX1 && TMath::Sign(lastY1,positionLinePt.Y())!=lastY1 ) cout<<i<<"diff sign
"<<std::endl;
                                             }
std::cout<<"chargeTot 1,2 "<<hitArrayRansac.size()<<" "<<chargeTot1<<" "<<chargeTot2<<std::endl;
                                             //------------- Ransac of last part of the track
----------------------------------- ransacExt->SetInit(hitArrayRansac); ransacExt->SetVertexMod(1);
            ransacExt->SetRanSamMode(3);
            ransacExt->CalcRANSACMod();
            std::vector<ATTrack> newTrackCand;
            newTrackCand = ransacExt->GetTrackCand();
            TVector3 newVertexMean;
            TVector3 vTrackNew1,vTrackNew2;
            eLoss_p1_reco_new=-1;
            eLoss_p2_reco_new=-1;
            range_p1_new=-999;range_p2_new=-999;theta1_new=-999;phi1_new=-999;theta2_new=-999;phi2_new=-999;
            if(newTrackCand.size()>1){
                  std::vector<Double_t> fitParNew1 = newTrackCand.at(0).GetFitPar();
                  std::vector<Double_t> fitParNew2 = newTrackCand.at(1).GetFitPar();
                  Int_t nHitsNewTrackCand1 = newTrackCand.at(0).GetHitArray()->size();
                  Int_t nHitsNewTrackCand2 = newTrackCand.at(1).GetHitArray()->size();
                  std::cout<<"hits points "<<nHitsNewTrackCand1<<" "<<nHitsNewTrackCand2<<std::endl;

                  newVertexMean =
ClosestPoint2Lines(fitParNew1,fitParNew2,nHitsNewTrackCand1,nHitsNewTrackCand2);//weighted vertex
                  //newVertexMean = ClosestPoint2Lines(fitParNew1,fitParNew2);//weighted vertex
                  vertexX = newVertexMean.X();
                  vertexY = newVertexMean.Y();
                  vertexZ = newVertexMean.Z();
                  //newVertexMean = newTrackCand.at(0).GetTrackVertex();//do the pair stuff?
                  // std::cout<<"ORI VTX "<<vertexMean.X()<<" "<<vertexMean.Y()<<" "<<vertexMean.Z()<<std::endl;
                  // std::cout<<"newtrackCand3 VTX "<<vertexMean3.X()<<" "<<vertexMean3.Y()<<"
"<<vertexMean3.Z()<<std::endl;

                  TVector3 lastPointNew1 = newTrackCand.at(0).GetLastPoint();
                  TVector3 firstPointNew1 = newTrackCand.at(0).GetFirstPoint();
                  Double_t lastXNew1 = lastPointNew1.X();
                 Double_t lastYNew1 = lastPointNew1.Y();
                Double_t lastZNew1 = lastPointNew1.Z();
                  TVector3
vpNew1(TMath::Sign(1,lastXNew1)*fabs(fitParNew1[1]),TMath::Sign(1,lastYNew1)*fabs(fitParNew1[3]),TMath::Sign(1,(lastZNew1-newVertexMean.Z()))*fabs(fitParNew1[5]));//direction
of the parametric line TVector3 v1New1(fitParNew1[0],fitParNew1[2],fitParNew1[4]);//point on the parametric line

                  TVector3 lastPointNew1proj=ptOnLine(vpNew1,v1New1,lastPointNew1);//projection of the last point of the
track on the parametric line TVector3 firstPointNew1proj=ptOnLine(vpNew1,v1New1,firstPointNew1);//projection of the last
point of the track on the parametric line vTrackNew1=lastPointNew1proj-newVertexMean; theta1_new =
vTrackNew1.Angle(vfrom); phi1_new = TMath::ATan2(vTrackNew1.Y(),vTrackNew1.X());

                  range_p1_new = newTrackCand.at(0).GetLinearRange(newVertexMean,lastPointNew1proj);
                  eLoss_p1_reco_new = splineEloss->Eval(range_p1_new);
                  std::cout<<"theta & range new 1 "<<theta1*TMath::RadToDeg()<<" "<<theta1_new*TMath::RadToDeg()<<"
"<<range_p1<<" "<<range_p1_new<<std::endl;


                  TVector3 lastPointNew2 = newTrackCand.at(1).GetLastPoint();
                  TVector3 firstPointNew2 = newTrackCand.at(1).GetFirstPoint();
                  Double_t lastXNew2 = lastPointNew2.X();
                  Double_t lastYNew2 = lastPointNew2.Y();
                  Double_t lastZNew2 = lastPointNew2.Z();
                  TVector3
vpNew2(TMath::Sign(1,lastXNew2)*fabs(fitParNew2[1]),TMath::Sign(1,lastYNew2)*fabs(fitParNew2[3]),TMath::Sign(1,(lastZNew2-newVertexMean.Z()))*fabs(fitParNew2[5]));//direction
of the parametric line TVector3 v1New2(fitParNew2[0],fitParNew2[2],fitParNew2[4]);//point on the parametric line

                  TVector3 lastPointNew2proj=ptOnLine(vpNew2,v1New2,lastPointNew2);//projection of the last point of the
track on the parametric line TVector3 firstPointNew2proj=ptOnLine(vpNew2,v1New2,firstPointNew2);//projection of the last
point of the track on the parametric line vTrackNew2=lastPointNew2proj-newVertexMean; theta2_new =
vTrackNew2.Angle(vfrom); phi2_new = TMath::ATan2(vTrackNew2.Y(),vTrackNew2.X());

                  range_p2_new = newTrackCand.at(1).GetLinearRange(newVertexMean,lastPointNew2proj);
                  eLoss_p2_reco_new = splineEloss->Eval(range_p2_new);
                  std::cout<<"theta & range new 2 "<<theta2*TMath::RadToDeg()<<" "<<theta2_new*TMath::RadToDeg()<<"
"<<range_p2<<" "<<range_p2_new<<std::endl; TVector3 calcVtx; calcVtx = ClosestPoint2Lines(fitParNew1,fitParNew2);
                  std::cout<<"vertex ori new X "<<vertexMean.X()<<" "<<newVertexMean.X()<<" "<<calcVtx.X()<<std::endl;
                  std::cout<<"vertex ori new Y "<<vertexMean.Y()<<" "<<newVertexMean.Y()<<" "<<calcVtx.Y()<<std::endl;
                  std::cout<<"vertex ori new Z "<<vertexMean.Z()<<" "<<newVertexMean.Z()<<" "<<calcVtx.Z()<<std::endl;
                  std::cout<<"range comp 1&2 "<<range_p1<<" "<<range_p1_new<<" "<<range_p2<<"
"<<range_p2_new<<std::endl;

                  distLinesNew = linesDist(fitParNew1,fitParNew2);
                  distVtxFromBeam = distPtLine(beamPar,newVertexMean);
                  chi2Fit1 = newTrackCand.at(0).GetMinimum();
                  chi2Fit2 = newTrackCand.at(1).GetMinimum();

                  std::vector<ATHit>* hitArrayNew1 = newTrackCand.at(0).GetHitArray();
                  //chargeRange1 = ( trackCand.at(0).GetLastPoint()-trackCand.at(0).GetFirstPoint() ).Mag();
                  chargeRange1 = (lastPointNew1proj-firstPointNew1proj).Mag();
                  //chargeTotNew1 = nHitsNewTrackCand1;
                  for(Int_t iHit=0; iHit<nHitsNewTrackCand1; iHit++){
                     ATHit hit = hitArrayNew1->at(iHit);
                     TVector3 position = hit.GetPosition();
                     avgDistHitFromLine+= distPtLine(fitParNew1,position);
                     chargeTotNew1 += hit.GetCharge();
                     //pad_z_signalWidth->Fill(position.Z(),hit.GetSignalWidth());
                     //pad_z_signalHeight->Fill(position.Z(),hit.GetSignalHeight());
                  }

                  std::vector<ATHit>* hitArrayNew2 = newTrackCand.at(1).GetHitArray();
                  chargeRange2 = (lastPointNew2proj-firstPointNew2proj).Mag();
                  //chargeTotNew2 = nHitsNewTrackCand2;
                  for(Int_t iHit=0; iHit<nHitsNewTrackCand2; iHit++){
                     ATHit hit = hitArrayNew2->at(iHit);
                     TVector3 position = hit.GetPosition();
                     avgDistHitFromLine+= distPtLine(fitParNew2,position);
                     chargeTotNew2 += hit.GetCharge();
                     //pad_z_signalWidth->Fill(position.Z(),hit.GetSignalWidth());
                     //pad_z_signalHeight->Fill(position.Z(),hit.GetSignalHeight());
                  }
                  avgDistHitFromLine = avgDistHitFromLine/(nHitsNewTrackCand1+nHitsNewTrackCand2);

                  MaxR1=sqrt(pow(lastPointNew1.X(),2)+pow(lastPointNew1.Y(),2));
                  MaxR2=sqrt(pow(lastPointNew2.X(),2)+pow(lastPointNew2.Y(),2));
                  MaxZ1=lastPointNew1.Z();
                  MaxZ2=lastPointNew2.Z();
            }

         vertexX = newVertexMean.X();
         vertexY = newVertexMean.Y();
         vertexZ = newVertexMean.Z();

      //	vertexT = newVertexMean.Z()/(10.*aDVel[irun]);
//-----------------------------------------------------------------------------------

//------------------------------------------


               //if(MaxR1>245. || MaxR2>245. || MaxR1<25. || MaxR2<25. || MaxZ1>975. || MaxZ2>975. || MaxZ1<25. ||
MaxZ2<25. || vertexZ<25. || vertexZ>975. || chargeTot1<5e3 || chargeTot2<5e3) continue; //if do no stop in chamber or in
the beam hole if(MaxR1>245. || MaxR2>245. || MaxR1<35. || MaxR2<35. || MaxZ1>975. || MaxZ2>975. || MaxZ1<25. ||
MaxZ2<25. || vertexZ<25. || vertexZ>975. || chargeTot1<5e3 || chargeTot2<5e3) continue; //if do no stop in chamber or in
the beam hole

               //if(MaxR1>245. || MaxR2>245. || MaxR1<40. || MaxR2<40. || MaxZ1>975. || MaxZ2>975. || MaxZ1<25. ||
MaxZ2<25. || vertexZ<25. || vertexZ>975. || chargeTot1<5e3 || chargeTot2<5e3) continue; //if do no stop in chamber or in
the beam hole

               //if(vertexZ<25. || vertexZ>975. || chargeTot1<5e3 || chargeTot2<5e3) continue; //if do no stop in
chamber or in the beam hole



               // if(vertexZ<25. || vertexZ>975. || chargeTot1<5e3 || chargeTot2<5e3) continue; //if do no stop in
chamber or in the beam hole
               // if( (MaxR1>245. ||  MaxR1<40. || MaxZ1>975. || MaxZ1<25.) && (MaxR2>245. ||  MaxR2<40. || MaxZ2>975.
|| MaxZ2<25.) ) continue;

               //if(MaxR1>245. || MaxR2>245. || MaxR1<40. || MaxR2<40. || MaxZ1>975. || MaxZ2>975. || MaxZ1<25. ||
MaxZ2<25. || vertexZ<25. || vertexZ>975.) continue; //if do no stop in chamber or in the beam hole

            //	if(MaxR1<40. || MaxR2<40. || vertexZ<25. || vertexZ>975. || chargeTot1<5e3 || chargeTot2<5e3) continue;
//if do no stop in chamber or in the beam hole

double diff=999.;
double idiff=-1;

            std::vector <double> S800_invMapOut =
get_invmap_vars(inv_map,S800_x0,S800_y0,S800_afp,S800_bfp,1.066-vertexMean.Z()/1000); S800_ata = S800_invMapOut.at(0);
            S800_bta = S800_invMapOut.at(1);
            S800_yta = S800_invMapOut.at(2);
            S800_dta = S800_invMapOut.at(3);
            S800_thetaLab= S800_invMapOut.at(4);
            S800_phi= S800_invMapOut.at(5);

//-----------Brho calc----------------------------------------------------------
            Double_t kinE0Decay = -999;
            Double_t decay_frag_mass = -999;
            Double_t aDecay = 1;
            Double_t qDecay = 1;
            if(InCondition14N) {//14N
               kinE0Decay = 1493.7515;
               decay_frag_mass = 14.00307400443 * 931.494 - 0.511*7.;
               aDecay = 14.;
               qDecay = 7.;
            }
            else if(InCondition12C) {//12C
               kinE0Decay = 1280.6107;
               decay_frag_mass = 12. * 931.494 - 0.511*6.;
               aDecay = 12.;
               qDecay = 6.;
            }
            else if(InCondition10B) {//12C
               kinE0Decay = 1065.94;
               decay_frag_mass = 10. * 931.494 - 0.511*5.;
               aDecay = 10.;
               qDecay = 5.;
            }
            else if(InCondition13C) {//13C
               kinE0Decay = 1359.2672;
               decay_frag_mass = 13.0033548352 * 931.494 - 0.511*6.;
               aDecay = 13.;
               qDecay = 6.;
            }
            else if(InCondition13N) {//13N
               kinE0Decay = 1385.5404;
               decay_frag_mass = 13.005738609 * 931.494 - 0.511*6.;
               aDecay = 13.;
               qDecay = 7.;
            }
            Double_t kinEDecay = (1.0+S800_dta)*kinE0Decay;
            Double_t momDecay = sqrt(pow(kinEDecay+decay_frag_mass,2) - pow(decay_frag_mass,2));
            TLorentzVector ejec4v;
            ejec4v.SetXYZM(momDecay*sin(S800_ata),momDecay*sin(S800_bta),momDecay*cos(S800_thetaLab),decay_frag_mass);
            Double_t gamma = ejec4v.Gamma();
            Double_t beta = ejec4v.Beta();

            std::cout<<" beta: "<<beta<<" gamma: "<<gamma<<" Brho: "<<3.1*beta*gamma*aDecay/qDecay<<std::endl;
            //std::cout<<" Elast "<<Anejec<<" "<<Eeje<<" "<<Peje<<" "<<beta<<" "<<gamma<<"
"<<3.1*beta*gamma*32./11.<<std::endl; brho = 3.1*beta*gamma*aDecay/qDecay;
//------------------------------------------------------------------------------

            dta_ata->Fill(S800_dta,S800_ata*180./TMath::Pi());//ata in deg
            diff = fabs(S800_yta-vertexMean.Y());



         //angle12=FindAngleBetweenTracks(vTrack1,vTrack2);
               //theta1 = vTrack1.Theta();
               //phi1 = TMath::ATan2(vTrack1.Y(),vTrack1.X());
               //theta2 = vTrack2.Theta();
               //phi2 = TMath::ATan2(vTrack2.Y(),vTrack2.X());

               angle12=FindAngleBetweenTracks(vTrackNew1,vTrackNew2);
               theta1 = theta1_new;
               phi1 = phi1_new;
               theta2 = theta2_new;
               phi2 = phi2_new;



         //==============================================================================
         // methods to get the proton eloss

         //eLoss_p1_reco = splineEloss->Eval(range_p1);
         //eLoss_p2_reco = splineEloss->Eval(range_p2);

               eLoss_p1_reco = eLoss_p1_reco_new;
         eLoss_p2_reco = eLoss_p2_reco_new;


         //==============================================================================

         epsilon_pp = 0.5*(eLoss_p1_reco + eLoss_p2_reco - 2 * sqrt(eLoss_p1_reco * eLoss_p2_reco) * TMath::Cos
(angle12)); epsilon_pp_reco->Fill(epsilon_pp);

          // reconstruction of 2He
         mom1_norm_reco = TMath::Sqrt(eLoss_p1_reco * eLoss_p1_reco + 2.0 * eLoss_p1_reco * proton_mass);
         mom_proton1_reco.SetX (mom1_norm_reco * TMath::Sin(theta1) * TMath::Cos(phi1));
         mom_proton1_reco.SetY (mom1_norm_reco * TMath::Sin(theta1) * TMath::Sin(phi1));
         mom_proton1_reco.SetZ (mom1_norm_reco * TMath::Cos(theta1));

         mom2_norm_reco = TMath::Sqrt (eLoss_p2_reco * eLoss_p2_reco + 2.0 * eLoss_p2_reco * proton_mass);
         mom_proton2_reco.SetX (mom2_norm_reco * TMath::Sin(theta2) * TMath::Cos(phi2));
         mom_proton2_reco.SetY (mom2_norm_reco * TMath::Sin(theta2) * TMath::Sin(phi2));
         mom_proton2_reco.SetZ (mom2_norm_reco * TMath::Cos(theta2));

         mom_He2_reco = mom_proton1_reco + mom_proton2_reco;
         E_tot_he2 = (proton_mass + eLoss_p1_reco) + (proton_mass + eLoss_p2_reco);
         he2_mass_ex = TMath::Sqrt (E_tot_he2 * E_tot_he2 - mom_He2_reco.Mag2 ());
         ex_he2_reco->Fill (he2_mass_ex - he2_mass);
         kin_He2 = TMath::Sqrt (mom_He2_reco.Mag2 () + he2_mass_ex * he2_mass_ex) - he2_mass_ex;
         theta_He2 = mom_He2_reco.Theta ()* TMath::RadToDeg();

               // TVector3 beamDir(-0.00915252,-0.00221017,0.9999557);
               //TVector3 beamDir(0,0,1.);//tracks already rotated in beam frame
               //TVector3 beamDir(-0.00915252,-0.00221017,0.9999557);
               //TVector3 beamDir(-0.014,0.,0.999902);
               Double_t mom_beam = sqrt( pow(Ekin_proj+proj_mass,2) - pow(proj_mass,2) );
               Double_t missing_mom;
               missing_mom = (mom_beam*beamDir-mom_He2_reco).Mag();
               Double_t missing_energy = (Ekin_proj+proj_mass+target_mass-(kin_He2+epsilon_pp+he2_mass+0.511));
               Double_t missing_mass = sqrt( pow(missing_energy,2)-pow(missing_mom,2) );

         phi_He2 = mom_He2_reco.Phi ()* TMath::RadToDeg();
         theta_r_he2_reco->Fill (theta_He2);
         phi_r_he2_reco->Fill (phi_He2);
         kin_r_he2_reco->Fill (kin_He2);
         theta_kin_he2_reco->Fill (theta_He2, kin_He2);


         d2heana->kine_2b (proj_mass, target_mass, he2_mass_ex, recoil_mass, Ekin_proj, theta_He2 * TMath::DegToRad (),
kin_He2);
         //d2heana->kine_2b (proj_mass, target_mass, he2_mass_ex, recoil_mass, Ekin_proj, (theta1+theta2)/2., kin_He2);

         theta_cm = d2heana->GetThetaCM ();
         //Ex4 = d2heana->GetMissingMass ();
               Ex4 = missing_mass - recoil_mass;

               Double_t sInv = pow(target_mass+proj_mass,2) + 2.*target_mass*Ekin_proj;
            Double_t momCMScat = sqrt( ( pow(sInv-pow(he2_mass+epsilon_pp,2)-pow(Ex4+recoil_mass,2),2)
- 4.*pow(he2_mass+epsilon_pp,2)*pow(Ex4+recoil_mass,2)) / (4.*sInv) );
               //------- rotation of track vectors so that Theta and Phi are in the beam frame
                     Double_t aRX = TMath::ATan2(beamDir.Y(),beamDir.Z());
                     Double_t aRY = TMath::ATan2(-beamDir.X(),beamDir.Z());
                     mom_He2_reco.RotateX(aRX);//rotate in trigo sens, y to z
                     mom_He2_reco.RotateY(aRY);//rotate in trigo sens, z to x
                     theta_He2 = mom_He2_reco.Theta ()* TMath::RadToDeg();
               Double_t thetaCMScat = asin(mom_He2_reco.Mag()/momCMScat*sin(theta_He2*TMath::DegToRad()));
               theta_lab = atan( sin(theta_cm*TMath::DegToRad())/(cos(theta_cm*TMath::DegToRad()) +
recoil_mass/target_mass));

std::cout<<" Ex  "<<Ex4 <<" "<<theta_cm<<" "<<epsilon_pp<<" "<<thetaCMScat*TMath::RadToDeg()<<std::endl;
               theta_cm = thetaCMScat*TMath::RadToDeg();

         thetacm_he2_reco->Fill (theta_cm);
         Ex_reco->Fill (Ex4);
         thetacm_Ex_he2_reco->Fill (theta_cm, Ex4);
         thetacm_epsilon_pp_reco->Fill(theta_cm,epsilon_pp);


         ivt=i;
         anatree->Fill();

            }//two proton pairs
      } //gated and  tracks.size()>1
   }// Event loop


   /// --------------------- End event loop ---------------------------------------
   outfile->cd();
   anatree->Write();
   splineEloss->Write();

   theta_r_he2_reco->Write ();
   phi_r_he2_reco->Write ();
   kin_r_he2_reco->Write ();
   theta_kin_he2_reco->Write ();
   thetacm_he2_reco->Write ();
   Ex_reco->Write ();
   ex_he2_reco->Write ();
   thetacm_Ex_he2_reco->Write ();
   thetacm_epsilon_pp_reco->Write();
   epsilon_pp_reco->Write();

   XfpObj_tof->Write();
   ICSum_Obj->Write();
   dta_ata->Write();
   x_y_crdc1->Write();
   x_y_crdc2->Write();
   trackProfile->Write();

   pad_z_signalWidth->Write();
   pad_z_signalHeight->Write();

   outfile->Close();


} //end main
*/
