



//#include "AtPattern.h"

// static Double_t proton_mass = 1.0078250322 * 931.494 - 0.511;
static Double_t proj_mass = 11.043723754 * 931.494 - 3*0.511;
static Double_t target_mass = 1.0078250322  * 931.494;
static Double_t recoil_mass = 1.0078250322 * 931.494 - 0.511;
static Double_t ejectile_mass = 11.043723754  * 931.494 - 3*0.511;
static Double_t Ekin_proj = 53.4* 11.043723754 ;

static Int_t nbTracksPerVtx = 1;

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

// void SetDVelArray()
// {
//    TString fileName = "utils/drift_vel_cal_vtxZ_FermiFit.txt";
//    ifstream fDVel(fileName);
//    Int_t l1 = 0;
//    Double_t l2 = 0;
//    for (string line; getline(fDVel, line);) {
//       stringstream parse_die(line);
//       parse_die >> l1 >> l2;
//       aDVel[l1] = l2;
//    }
//    fDVel.close();
// }

void SetERtable(TString eLossTable)
{ // fit of the GEANT4 E vs R obtained from the simulation with the function model given by LISE++
   //ifstream fER("eLossTables/p_in_HCF4_750torr_SRIM.txt");
   //ifstream fER("eLossTables/p_in_p_750torr_SRIM.txt");
   //ifstream fER("eLossTables/p_in_p_600torr_SRIM.txt");
   //ifstream fER("eLossTables/p_in_HCF4_600torr_SRIM.txt");
   //ifstream fER("eLossTables/p_in_HCF4_699torr_SRIM.txt");
   ifstream fER(eLossTable.Data());
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
      cout<<X[i]<<" "<<Y[i]<<endl;
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

void ana_1track(Int_t runNumber, TString eLossTable)
{

   SetERtable(eLossTable);
//    SetDVelArray();

   FairRunAna *run = new FairRunAna(); // Forcing a dummy run
   // ATd2HeAnalysis *d2heana = new ATd2HeAnalysis ();

   TString digiFileName = TString::Format("/media/aurio/Cris/11Li/unpack/run_%04d_e23031.root", runNumber);
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
   TString outFileNameHead = TString::Format("temp/run_%04d_e23031_analyzed_1track.root", runNumber);
   outfile = TFile::Open(outFileNameHead, "recreate");

   S800Ana s800Ana;

   //-----
   Int_t ivt = 0, irun = runNumber, NVtxEvt = 0, NTracksVtx = 0;
   Float_t range_p1 = 0., charge1 = 0.;
   Float_t eLoss_p1_reco = 0.0;
   Float_t theta1 = 0., phi1 = 0., theta1_corr= 0.;
   Float_t mom1_norm_reco = 0.;
   Float_t theta_cm = 0., Ex4 = 0.;
   Float_t lastX1 = 0., lastY1 = 0.,  lastZ1 = 0., vertexX = 0., vertexY = 0., vertexZ = 0.;
   Float_t MaxR1, MaxZ1;
   Double_t theta_lab = 0;
   Double_t Eje_ata = -999, Eje_bta = -999, Eje_dta = -999;
   Double_t brho = -999;
   Double_t holeAcc = 0, dtaAcc = 0;

   XYZVector mom_proton1_reco, mom_proton2_reco, mom_He2_reco;

   ULong64_t S800_timeStamp = 0;
   Double_t S800_timeRf = 0., S800_x0 = 0., S800_x1 = 0., S800_y0 = 0., S800_y1 = 0., S800_E1up = 0., S800_E1down = 0.,
            S800_hodoSum = 0., S800_afp = 0., S800_bfp = 0., S800_ata = 0., S800_bta = 0., S800_yta = 0., S800_dta = 0.,
            S800_thetaLab = 0., S800_phi = 0., S800_E1up_ToF = 0., S800_E1down_ToF = 0., S800_E1_ToF = 0.,
            S800_XfObj_ToF = 0., S800_ObjCorr_ToF = 0., S800_Obj_ToF = 0., S800_XfObjCorr_ToF = 0., S800_ICSum_dE = 0.,
	    S800_E1_dE = 0.;

   XYZVector beamDir(0,0,1.0);

   //---- Set S800Ana -------------------------------------------------------------
   //TString mapPath = "/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/invMap/invmap_14N";
   //s800Ana.SetInverseMap(mapPath, -0.5, 1, 0.1);
   
   vector<TString> fcutPID1File;
   vector<TString> fcutPID2File;
   vector<TString> fcutPID3File;

   fcutPID1File.push_back("/mnt/analysis/e21018/codes/ATTPCROOTv2-1/macro/Unpack_HDF5/e21018_S800/rootPID/32AlBeam_New.root");
   fcutPID2File.push_back("/projects/ceclub/giraud/git/ATTPCROOTv2/macro/Unpack_HDF5/e18008_S800/rootPID/afpx.root");
   fcutPID3File.push_back("/mnt/analysis/e21018/codes/ATTPCROOTv2-1/macro/Unpack_HDF5/e21018_S800/rootPID/Mg32CE_New.root");
   

    //e21018
    	 std::vector<Double_t> S800MTDCObjCorr;
	// S800MTDCObjCorr.push_back(105.);
	// S800MTDCObjCorr.push_back(0.015);
	 S800MTDCObjCorr.push_back(0);
	 S800MTDCObjCorr.push_back(0);
    	 std::vector<Double_t> S800E1dECorr;
	 S800E1dECorr.push_back(1.);//gain correction E1up
	 S800E1dECorr.push_back(1.);//E1down
	 std::vector<Double_t> S800MTDCObjRange;
	 //S800MTDCObjRange.push_back(-340);
	 //S800MTDCObjRange.push_back(-230);
         S800MTDCObjRange.push_back(-465);
	 S800MTDCObjRange.push_back(-290);
	 std::vector<Double_t> S800MTDCXfRange;
	 //S800MTDCXfRange.push_back(-200);
	 //S800MTDCXfRange.push_back(210);
	 S800MTDCXfRange.push_back(-150);
	 S800MTDCXfRange.push_back(50);

    //e18008
   //  std::vector<Double_t> S800MTDCObjCorr;
	//  S800MTDCObjCorr.push_back(70.);
	//  S800MTDCObjCorr.push_back(0.0085);
	//  std::vector<Double_t> S800MTDCObjRange;
	//  S800MTDCObjRange.push_back(-120);
	//  S800MTDCObjRange.push_back(-20);
	//  std::vector<Double_t> S800MTDCXfRange;
	//  S800MTDCXfRange.push_back(160);
	//  S800MTDCXfRange.push_back(240);


	s800Ana.SetPID1cut(fcutPID1File);
	s800Ana.SetPID2cut(fcutPID2File);
	s800Ana.SetPID3cut(fcutPID3File);
	s800Ana.SetMTDCXfRange(S800MTDCXfRange);
	s800Ana.SetMTDCObjRange(S800MTDCObjRange);
	s800Ana.SetTofObjCorr(S800MTDCObjCorr);
	s800Ana.SetE1dECorr(S800E1dECorr);

   //----------------------------------------------------------------------------

   TTree *anatree = new TTree("anatree", "new TTree");

   anatree->Branch("ivt", &ivt);
   anatree->Branch("irun", &irun);
   anatree->Branch("NVtxEvt", &NVtxEvt);
   anatree->Branch("NTracksVtx", &NTracksVtx);
   anatree->Branch("range_p1", &range_p1);
   anatree->Branch("charge1", &charge1);
   anatree->Branch("theta1", &theta1);
   anatree->Branch("theta1_corr", &theta1_corr);
   anatree->Branch("phi1", &phi1);
   anatree->Branch("lastX1", &lastX1);
   anatree->Branch("lastY1", &lastY1);
   anatree->Branch("lastZ1", &lastZ1);
   anatree->Branch("MaxR1", &MaxR1);
   anatree->Branch("vertexX", &vertexX);
   anatree->Branch("vertexY", &vertexY);
   anatree->Branch("vertexZ", &vertexZ);
   anatree->Branch("eLoss_p1_reco", &eLoss_p1_reco);
   anatree->Branch("theta_cm", &theta_cm);
   anatree->Branch("theta_lab", &theta_lab);
   anatree->Branch("Ex4", &Ex4);
   anatree->Branch("mom1_norm_reco", &mom1_norm_reco);
   anatree->Branch("Eje_ata", &Eje_ata);
   anatree->Branch("Eje_bta", &Eje_bta);
   anatree->Branch("Eje_dta", &Eje_dta);
   anatree->Branch("brho", &brho);
   anatree->Branch("holeAcc", &holeAcc);
   anatree->Branch("dtaAcc", &dtaAcc);

   anatree->Branch("S800_XfObj_ToF", &S800_XfObj_ToF);
   anatree->Branch("S800_ObjCorr_ToF", &S800_ObjCorr_ToF);
   anatree->Branch("S800_ICSum_dE", &S800_ICSum_dE);
   anatree->Branch("S800_E1_dE", &S800_E1_dE);
   anatree->Branch("S800_CRDC0_x",&S800_x0);
   anatree->Branch("S800_CRDC1_x",&S800_x1);
   anatree->Branch("S800_CRDC0_y",&S800_y0);
   anatree->Branch("S800_CRDC1_y",&S800_y1);
   anatree->Branch("S800_afp",&S800_afp);
   anatree->Branch("S800_bfp",&S800_bfp);

   ///============================= Event loop ====================================
   std::cout << " nEvents : " << nEvents << "\n";
   for (Int_t i = 0; i < nEvents; i++) {
      s800cal->Clear();
      bS800cal->GetEntry(i);
      reader.Next();

      //Bool_t isInPIDGates = kFALSE;
      //isInPIDGates = s800Ana.isInPID(s800cal);

      /* if (!isInPIDGates) */
      /*    continue; */

      //std::cout << " Event Number : " << i << "\n";

      //---- Get S800 data -----------------------------------------------------------
      s800Ana.Calc(s800cal);
		
      S800_XfObj_ToF = s800Ana.GetXfObj_ToF();
      S800_ObjCorr_ToF = s800Ana.GetObjCorr_ToF();
      S800_ICSum_dE = s800Ana.GetICSum_E();
      S800_E1_dE = s800Ana.GetE1_dE();
      std::vector<Double_t> S800_fpVar; //[0]=fX0 | [1]=fX1 | [2]=fY0 | [3]=fY1 | [4]=fAfp | [5]=fBfp
      S800_fpVar = s800Ana.GetFpVariables();
      S800_x0 = S800_fpVar.at(0);
      S800_x1 = S800_fpVar.at(1);
      S800_y0 = S800_fpVar.at(2);
      S800_y1 = S800_fpVar.at(3);
      S800_afp = S800_fpVar.at(4);
      S800_bfp = S800_fpVar.at(5);
      //------------------------------------------------------------------------------

      AtPatternEvent *patternEvent = (AtPatternEvent *)patternArray->At(0);

      if (patternEvent) {

         std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();

         AtFindVertex findVtx(30); // distance between lines
         findVtx.FindVertex(patternTrackCand, nbTracksPerVtx);
         std::vector<tracksFromVertex> tv;
         tv = findVtx.GetTracksVertex();
         NVtxEvt = tv.size();   // number of clusters of tracks forming a vertex (could have ex: 2 vertexes with 2 tracks each)
         NTracksVtx = 0; // number of tracks for each vertex

         for (size_t ive = 0; ive < NVtxEvt; ive++) {
            std::cout << "ive " << ive << " " << tv.at(ive).vertex.X() << " " << tv.at(ive).vertex.Y() << " "
                      << tv.at(ive).vertex.Z() << " " << tv.at(ive).tracks.at(0).GetGeoQEnergy() << " "
                      << std::endl;

            NTracksVtx = tv.at(ive).tracks.size();
            if (NTracksVtx != 1)
               continue; // don't analyze event with other than 1 track per vertex

            theta1 = 0.;
            phi1 = 0.;
            range_p1 = 0.;
            eLoss_p1_reco = 0.;
            mom1_norm_reco = 0.;
            theta1_corr = 0;
            theta_cm = 0.;
            Ex4 = 0.;
            MaxR1 = 0.;
            MaxZ1 = 0.;
            theta_lab = 0;
            charge1 = 0;

            XYZVector vertexMean = tv.at(ive).vertex;
            XYZVector lastPoint1 = (XYZVector)tv.at(ive).tracks.at(0).GetLastPoint();
            MaxR1 = lastPoint1.Rho();
            MaxZ1 = lastPoint1.Z();

            charge1 = tv.at(ive).tracks.at(0).GetGeoQEnergy();

            // tracks must stop within the chamber
            if (charge1 < 5e3 || 
                MaxR1 > 245. || MaxR1 < 35. ||
                MaxZ1 > 975. || MaxZ1 < 25. || 
                vertexMean.Z() < 25. || vertexMean.Z() > 975.)
               continue;

            //------------------------------------------------------------------------------
            // refit only the last part of the track and if necessary do small adjustement for the drift vel.
            std::vector<AtTrack> patternTrackCandReFit;
            AtTrack trackToReFit;
            std::vector<AtHit> hitArray = ContainerManip::GetObjectVector(tv.at(ive).tracks.at(0).GetHitArray());
            for (Int_t iHit = 0; iHit < hitArray.size(); iHit++) {
                auto hit = hitArray.at(iHit);
                XYZPoint position = hit.GetPosition();
                // position = position.SetXYZ(position.X(), position.Y(), (1000. / aDVel[irun]) * (position.Z()));
                position = position.SetXYZ(position.X(), position.Y(), position.Z());
                hit.SetPosition(position);
                if (sqrt(pow(position.X(), 2) + pow(position.Y(), 2)) > 25.) { //35
                    trackToReFit.AddHit(hit);
                }
            }

            auto patternType = AtPatterns::PatternType::kLine;
            auto pattern = AtPatterns::CreatePattern(patternType);
            pattern->FitPattern(ContainerManip::GetObjectVector(trackToReFit.GetHitArray()), 50); // charge threshold, defined in the unpack macro as well
            trackToReFit.SetPattern(pattern->Clone());
            patternTrackCandReFit.push_back(trackToReFit);

            // find new vertex
            std::vector<tracksFromVertex> tvReFit;
            if (patternTrackCandReFit.size() >= nbTracksPerVtx) {
               AtFindVertex findVtxReFit(30); // distance between lines
               findVtxReFit.FindVertex(patternTrackCandReFit, nbTracksPerVtx);
               tvReFit = findVtxReFit.GetTracksVertex();
            }
            // std::cout << "tvReFit.size(): "
            //           << " " << tvReFit.size() <<" tv.size(): " <<NVtxEvt<< " nbTracksPerVtx: "
            //           << nbTracksPerVtx << " patternTrackCandReFit.size(): " << patternTrackCandReFit.size()
            //           << std::endl;
            if (tvReFit.size() < 1)
               continue; // tvReFit size should be 1

            vertexMean = (XYZPoint)tvReFit.at(0).vertex;
            auto ransacLine1 = dynamic_cast<const AtPatterns::AtPatternLine *>(tvReFit.at(0).tracks.at(0).GetPattern());
            lastPoint1 = tvReFit.at(0).tracks.at(0).GetLastPoint();
            XYZVector lastPoint1proj = ptOnLine(ransacLine1->GetPatternPar(), lastPoint1); // projection of the last point of the track on the parametric line
            MaxR1 = lastPoint1proj.Rho();
            MaxZ1 = lastPoint1proj.Z();
            // charge1 = tvReFit.at(0).tracks.at(0).GetGeoQEnergy();
            // std::cout << "itvReFit "
            //           << " " << vertexMean.X() << " " << vertexMean.Y() << " " << vertexMean.Z() << " " << MaxZ1 << " " << std::endl;

            lastX1 = lastPoint1proj.X();
            lastY1 = lastPoint1proj.Y();
            lastZ1 = lastPoint1proj.Z();
            vertexX = vertexMean.X();
            vertexY = vertexMean.Y();
            vertexZ = vertexMean.Z();

            theta1 = GetThetaPhi(vertexMean, lastPoint1proj).first; // GetThetaPhi(..,..,-1) for simu;
            phi1 = GetThetaPhi(vertexMean, lastPoint1proj).second;

            range_p1 = tvReFit.at(0).tracks.at(0).GetLinearRange((XYZPoint)vertexMean, (XYZPoint)lastPoint1proj);

            // again with updated parameters
            if (charge1 < 5e3 || 
                MaxR1 > 245. || MaxR1 < 35. ||
                MaxZ1 > 975. || MaxZ1 < 25. || 
                vertexMean.Z() < 25. || vertexMean.Z() > 975.)
               continue;
            //==============================================================================
            // get fragment parameters at vertex location from S800 inverse map

            //Double_t zta = 1.066-vertexMean.Z()/1000;
            //std::vector<Double_t> invMapVars;
            //invMapVars = s800Ana.CalcInverseMap(zta);
            //std::cout<<"invMap results, ata, bta, yta, dta, thetaLab, phi: "<<invMapVars.at(0)<<" "<<invMapVars.at(1)<<" "<<invMapVars.at(2)
            //<<" "<<invMapVars.at(3)<<" "<<invMapVars.at(4)<<" "<<invMapVars.at(5)<<" "<<std::endl;
            //Eje_ata = invMapVars.at(0);
            //Eje_bta = invMapVars.at(1);
            //Eje_dta = invMapVars.at(3);
            //==============================================================================

            //==============================================================================
            // methods to get the proton eloss

            eLoss_p1_reco = splineEloss->Eval(range_p1);

            //==============================================================================

            mom1_norm_reco = TMath::Sqrt(eLoss_p1_reco * eLoss_p1_reco + 2.0 * eLoss_p1_reco * recoil_mass);
            mom_proton1_reco.SetX(mom1_norm_reco * TMath::Sin(theta1) * TMath::Cos(phi1));
            mom_proton1_reco.SetY(mom1_norm_reco * TMath::Sin(theta1) * TMath::Sin(phi1));
            mom_proton1_reco.SetZ(mom1_norm_reco * TMath::Cos(theta1));

            Double_t mom_beam = sqrt(pow(Ekin_proj + proj_mass, 2) - pow(proj_mass, 2));
            Double_t missing_mom;
            missing_mom = sqrt((mom_beam * beamDir - mom_proton1_reco).Mag2());
            Double_t missing_energy = (Ekin_proj + proj_mass + target_mass - (eLoss_p1_reco + recoil_mass));
            Double_t missing_mass = sqrt(pow(missing_energy, 2) - pow(missing_mom, 2));

            Ex4 = missing_mass - ejectile_mass;

            Double_t sInv = pow(target_mass + proj_mass, 2) + 2. * target_mass * Ekin_proj;
            Double_t momCMScat = sqrt((pow(sInv - pow(recoil_mass, 2) - pow(Ex4 + ejectile_mass, 2), 2) -
                                       4. * pow(recoil_mass, 2) * pow(Ex4 + ejectile_mass, 2)) /
                                      (4. * sInv));

            // //------- rotation of track vectors so that Theta and Phi are in the beam frame
            TVector3 momBuff; // dirty trick to use Rotate functions (not available with XYZvector)
            momBuff.SetXYZ(mom_proton1_reco.X(), mom_proton1_reco.Y(), mom_proton1_reco.Z());
            Double_t aRX = TMath::ATan2(beamDir.Y(), beamDir.Z());
            Double_t aRY = TMath::ATan2(-beamDir.X(), beamDir.Z());
            momBuff.RotateX(aRX); // rotate in trigo sens, y to z
            momBuff.RotateY(aRY); // rotate in trigo sens, z to x
            mom_proton1_reco.SetXYZ(momBuff.X(), momBuff.Y(), momBuff.Z());

            theta1_corr= mom_proton1_reco.Theta() * TMath::RadToDeg();
            Double_t thetaCMScat = asin(sqrt(mom_proton1_reco.Mag2()) / momCMScat * sin(theta1_corr* TMath::DegToRad()));
            theta_cm = thetaCMScat * TMath::RadToDeg();

            std::cout << " check values, Ex  " << Ex4 << " theta_cm " << theta_cm << " "
                      <<range_p1<<" "<<eLoss_p1_reco<< std::endl;


            ivt = i;
            anatree->Fill();
         } // for tv size (ive)
      }    // RANSAC null pointer
   }       // Event loop
   
   /// --------------------- End event loop ---------------------------------------
   outfile->cd();
   splineEloss->Write();
   anatree->Write();
   outfile->Close();

} // end main
