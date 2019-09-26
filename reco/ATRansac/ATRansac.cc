#include "ATRansac.hh"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

using namespace ROOT::Math;



ClassImp(ATRANSACN::ATRansac)

ATRANSACN::ATRansac::ATRansac()
{
  fVertex_1.SetXYZ(-10000,-10000,-10000);
  fVertex_2.SetXYZ(-10000,-10000,-10000);
  fMinimum = -1.0;
  fLineDistThreshold = 3.0;

  fRANSACModel = pcl::SACMODEL_LINE;
  fRANSACThreshold = 5.0;
  fMinHitsLine = 5;

  fRPhiSpace = kFALSE;
  fXCenter = 0.0;
  fYCenter = 0.0;

  fRANSACPointThreshold = 0.01; //Default 10%

  fVertexTime = -1000.0;

  /*FairLogger *fLogger=FairLogger::GetLogger();
  ATDigiPar *fPar;

  FairRun *run = FairRun::Instance();
  if (!run)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No analysis run!");

  FairRuntimeDb *db = run -> GetRuntimeDb();
  if (!db)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No runtime database!");

  fPar = (ATDigiPar *) db -> getContainer("ATDigiPar");
  if (!fPar)
    fLogger -> Fatal(MESSAGE_ORIGIN, "ATDigiPar not found!!");*/

    fTiltAng = 6.4; //fPar->GetTiltAngle();


  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

}

ATRANSACN::ATRansac::~ATRansac()
{
}


TVector3 ATRANSACN::ATRansac::GetVertex1()                                                                   {return fVertex_1;}
TVector3 ATRANSACN::ATRansac::GetVertex2()                                                                   {return fVertex_2;}
Double_t ATRANSACN::ATRansac::GetMinimum()                                                                   {return fMinimum;}
std::vector<ATTrack> ATRANSACN::ATRansac::GetTrackCand()                                                     {return fTrackCand;}
std::vector<ATRANSACN::ATRansac::PairedLines> ATRANSACN::ATRansac::GetPairedLinesArray()                     {return PLines;}
std::pair<Int_t,Int_t> ATRANSACN::ATRansac::GetPairTracksIndex()                                             {return fVertex_tracks;}
Double_t ATRANSACN::ATRansac::GetVertexTime()                                                                {return fVertexTime;}
TVector3 ATRANSACN::ATRansac::GetVertexMean()                                                                {return fVertex_mean;}


void ATRANSACN::ATRansac::SetModelType(int model)                                       { fRANSACModel = model;}
void ATRANSACN::ATRansac::SetDistanceThreshold(Float_t threshold)                       { fRANSACThreshold = threshold;}
void ATRANSACN::ATRansac::SetMinHitsLine(Int_t nhits)                                   { fMinHitsLine = nhits;}
void ATRANSACN::ATRansac::SetRPhiSpace()                                                { fRPhiSpace = kTRUE;}
void ATRANSACN::ATRansac::SetXYCenter(Double_t xc, Double_t yc)                         { fXCenter = xc;fYCenter=yc;}
void ATRANSACN::ATRansac::SetRANSACPointThreshold(Float_t val)                          { fRANSACPointThreshold = val;}
void ATRANSACN::ATRansac::SetVertexTime(Double_t val)                                   { fVertexTime = val;}


void ATRANSACN::ATRansac::CalcRANSAC(ATEvent *event)
{

    std::vector<ATTrack*> tracks = RansacPCL(event);
    //std::cout<<" Number of tracks : "<<tracks.size()<<std::endl;

    if(tracks.size()>1){ //Defined in CalcGenHoughSpace
      for(Int_t ntrack=0;ntrack<tracks.size();ntrack++){
        std::vector<ATHit>* trackHits = tracks.at(ntrack)->GetHitArray();
        Int_t nHits = trackHits->size();
        //std::cout<<" Num  Hits : "<<nHits<<std::endl;
          if(nHits>5)
          {
          MinimizeTrack(tracks.at(ntrack));


          }
      }// Tracks loop
      FindVertex(tracks);
    }// Minimum tracks

    // Drawing tracks against the Event
      /*TH2F* vis_RAD = new TH2F("vis_RAD","vis_RAD",1000,0,1000,1000,-250,250);
      TH2F* exp_RAD = new TH2F("exp_RAD","exp_RAD",1000,0,1000,1000,-250,250);

      TH2F* exp_amp =  new TH2F("exp_amp","exp_amp",1000,-250,250,1000,0,4000);
      TH2F* track_amp =  new TH2F("track_amp","track_amp",1000,-250,250,1000,0,4000);

      vis_RAD->SetMarkerColor(kGreen);
      vis_RAD->SetMarkerStyle(20);
      vis_RAD->SetMarkerSize(1.5);

      exp_RAD->SetMarkerColor(kRed);
      exp_RAD->SetMarkerStyle(20);
      exp_RAD->SetMarkerSize(1.0);

      exp_amp->SetMarkerColor(kGreen);
      exp_amp->SetMarkerStyle(20);
      exp_amp->SetMarkerSize(1.5);

      track_amp->SetMarkerColor(kRed);
      track_amp->SetMarkerStyle(20);
      track_amp->SetMarkerSize(1.0);

      if(tracks.size()>0){
          for(Int_t i=0;i<1;i++){
           std::vector<ATHit> hit_track = *tracks.at(i)->GetHitArray();

                  for(Int_t j=0;j<hit_track.size();j++)
                  {
                    ATHit hitT =  hit_track.at(j);
                    TVector3 posSol = hitT.GetPosition();
                    Double_t rad = TMath::Sqrt( TMath::Power(posSol.X(),2) + TMath::Power(posSol.Y(),2) );
                    vis_RAD->Fill(posSol.Z(),rad);
                    track_amp->Fill(posSol.Y(),hitT.GetCharge());

                  }

         }
      }

      std::vector<ATHit> *hit_track = event->GetHitArray();

          for(Int_t i=0;i<hit_track->size();i++){
                ATHit hit =  hit_track->at(i);
                TVector3 pos = hit.GetPosition();
                Double_t rad = TMath::Sqrt( TMath::Power(pos.X(),2) + TMath::Power(pos.Y(),2) );
                exp_RAD->Fill(pos.Z(),rad);
                exp_amp->Fill(pos.Y(),hit.GetCharge());

          }

          track_amp->Draw();
          exp_amp->Draw("SAME");

        //vis_RAD->Draw();
        //exp_RAD->Draw("SAME");

        */

}

void ATRANSACN::ATRansac::CalcRANSACFull(ATEvent *event)
{

        std::vector<ATTrack*> tracks = RansacPCL(event);

        XYZVector Z_1(0.0,0.0,1.0); // Beam direction

        if(tracks.size()>1){ //Defined in CalcGenHoughSpace
          for(Int_t ntrack=0;ntrack<tracks.size();ntrack++){
            std::vector<ATHit>* trackHits = tracks.at(ntrack)->GetHitArray();
            Int_t nHits = trackHits->size();

              if(nHits>fMinHitsLine) //We only accept lines with more than 5 hits and a maximum number of lines of 5
              {
              MinimizeTrack(tracks.at(ntrack));
              tracks.at(ntrack)->SetTrackID(ntrack);
              std::vector<Double_t> p = tracks.at(ntrack)->GetFitPar();
                  if(p.size()==4){
                    XYZVector L_1(p[1], p[3], 1. );
                    Double_t angZDeti = GetAngleTracks(L_1,Z_1);
                    tracks.at(ntrack)->SetAngleZAxis(angZDeti);
                    fTrackCand.push_back(*tracks.at(ntrack));
                  }
              }
          }// Tracks loop
              if(fTrackCand.size()>5) fTrackCand.resize(5);
        }// Minimum tracks


}

std::vector<ATTrack*> ATRANSACN::ATRansac::Ransac(std::vector<ATHit>* hits)
{
    std::vector<ATTrack*> tracks;

    //Data writer
    //pcl::PCDWriter writer;

    // initialize PointClouds
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGBA>);

    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);

    if(hits->size()<5) return tracks;

    Int_t nHits = hits->size();
    cloud->points.resize(nHits);

    for(Int_t iHit=0; iHit<nHits; iHit++){

          ATHit* hit = &hits->at(iHit);
          Int_t PadNumHit = hit->GetHitPadNum();
          TVector3 position = hit->GetPosition();

        if(fRPhiSpace){ //TODO: Pass a vector of hits with the proper RxPhi conversion
          cloud->points[iHit].x = hit->GetTimeStamp();
          cloud->points[iHit].y = TMath::Sqrt(  TMath::Power((fXCenter-position.X()),2)   +  TMath::Power((fYCenter-position.Y()),2)    )*TMath::ATan2(fXCenter-position.X(),fYCenter-position.Y());
          cloud->points[iHit].z = 0.0;
          cloud->points[iHit].rgb = iHit;


        }
        else{
          cloud->points[iHit].x = position.X();
          cloud->points[iHit].y = position.Y();
          cloud->points[iHit].z = position.Z();
          cloud->points[iHit].rgb = iHit; // Storing the position of the hit in the event container
        }

    }

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(fRANSACModel);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(fRANSACThreshold);

    // Create the filtering object
 pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

int i = 0, nr_points = (int) cloud->points.size ();


while (cloud->points.size () > fRANSACPointThreshold * nr_points)
 {
 // Segment the largest planar component from the remaining cloud
 //std::cout<<cloud->points.size()<<std::endl;

     seg.setInputCloud (cloud);
     seg.segment(*inliers, *coefficients);

     std::vector<Double_t> coeff;

     /*std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                         << coefficients->values[1] << " "
                                         << coefficients->values[2] << " " 
                                         << coefficients->values[3] << " "
                                         << coefficients->values[4] << "  "
                                         << coefficients->values[5] << std::endl;*/

     for(auto icoeff=0;icoeff<6;++icoeff)
          coeff.push_back(coefficients->values[icoeff]);                                    



     if (inliers->indices.size () == 0)
     {
       //std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
       break;
     }
  // Extract the inliers
     extract.setInputCloud (cloud);
     extract.setIndices (inliers);
     extract.setNegative (false);
     extract.filter (*cloud_p);
     //std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
     //TODO: Possible memory leak, change to vector of objects!

     //std::cout<<" Cloud p size "<<cloud_p->points.size()<<" "<<cloud_p->width<<"  "<<cloud_p->height<<"\n";

     if(cloud_p->points.size()>0){
         ATTrack *track = new ATTrack();

              for(Int_t iHit=0;iHit<cloud_p->points.size();iHit++)
              {
                if(&hits->at(cloud_p->points[iHit].rgb)) track->AddHit(&hits->at(cloud_p->points[iHit].rgb));
              }

         track->SetRANSACCoeff(coeff);
         

         tracks.push_back(track);
      }   
     //std::stringstream ss;
     //ss << "../track_" << i << ".pcd";
     //writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

     // Create the filtering object
     extract.setNegative(true);
     extract.filter(*cloud_f);
     cloud.swap(cloud_f);
     i++;


}


    return tracks;


}


std::vector<ATTrack*> ATRANSACN::ATRansac::RansacPCL(ATEvent *event)
{

    return ATRANSACN::ATRansac::Ransac(event->GetHitArray());

}

Int_t ATRANSACN::ATRansac::MinimizeTrack(ATTrack* track)
{

         gErrorIgnoreLevel=kFatal;
         Int_t nd = 10000;
         TGraph2D * gr = new TGraph2D(); /////NB: This should be created on the heap only once so it should move outside of this function!!!!!!!!!!!!!!!
         std::vector<ATHit> *HitArray = track->GetHitArray();

         double p0[4] = {10,20,1,2}; //For the moment those are dummy parameters

            for(Int_t N=0;N<HitArray->size();N++){
              ATHit hit = HitArray->at(N);
              TVector3 pos = hit.GetPosition();
              gr->SetPoint(N,pos.X(),pos.Y(),pos.Z());
            }

            ROOT::Fit::Fitter fitter;
            SumDistance2 sdist(gr);
            #ifdef __CINT__
            ROOT::Math::Functor fcn(&sdist,4,"SumDistance2");
            #else
            ROOT::Math::Functor fcn(sdist,4);
            #endif
            // set the function and the initial parameter values
            double pStart[4] = {1,1,1,1};
            fitter.SetFCN(fcn,pStart);
            // set step sizes different than default ones (0.3 times parameter values)
            for (int i = 0; i <4; ++i) fitter.Config().ParSettings(i).SetStepSize(0.01);

            bool ok = fitter.FitFCN();
            if (!ok) {
              Error("line3Dfit","Line3D Fit failed");
              return 1;
            }

             const ROOT::Fit::FitResult & result = fitter.Result();
             const ROOT::Math::Minimizer * min = fitter.GetMinimizer();
             double sigma2 = 25.0; //Size of the pad
             double Chi2_min = min->MinValue();
             int NDF = min->NFree();
             int npoints = gr->GetN();
             const double * parFitBuff = result.GetParams();
             std::vector<Double_t> parFit;
             for(Int_t i=0;i<4;i++) parFit.push_back(parFitBuff[i]); //4 parameters per fit
             track->SetFitPar(parFit);
             track->SetMinimum(Chi2_min);
             track->SetNFree(NDF);

             std::cout<<parFit[0]<<" "<<parFit[1]<<"  "<<parFit[2]<<" "<<parFit[3]<<std::endl;
 		         std::cout<<" Chi2 (Minuit) : "<<Chi2_min<<" NDF : "<<NDF<<std::endl;
             std::cout<<" Chi2 reduced  : "<<(Chi2_min/sigma2/(double) npoints)<<std::endl;



            //std::cout << "Total final distance square " << result.MinFcnValue() << std::endl;
            //result.Print(std::cout);


                //Draw the fit
                /*gr->Draw("p0");
                const double * parFit = result.GetParams();
                int n = 1000;
                double t0 = 0;
                double dt = 1000;
                TPolyLine3D *l = new TPolyLine3D(n);
                for (int i = 0; i <n;++i) {
                   double t = t0+ dt*i/n;
                   double x,y,z;
                   SetLine(t,parFit,x,y,z);
                   l->SetPoint(i,x,y,z);
                   //std::cout<<" x : "<<x<<" y : "<<y<<"  z : "<<z<<std::endl;
                }
                l->SetLineColor(kRed);
                l->Draw("same");*/

            return 0;



}

Int_t ATRANSACN::ATRansac::MinimizeTrackRPhi(ATTrack* track)
{

  gErrorIgnoreLevel=kFatal;
  TGraph* gr = new TGraph();

   std::vector<ATHit> *HitArray = track->GetHitArray();


      for(Int_t N=0;N<HitArray->size();N++){
        ATHit hit = HitArray->at(N);
        TVector3 pos = hit.GetPosition();
        Double_t xdum = hit.GetTimeStamp();
        Double_t ydum = TMath::Sqrt(  TMath::Power((fXCenter-pos.X()),2)   +  TMath::Power((fYCenter-pos.Y()),2)    )*TMath::ATan2(fXCenter-pos.X(),fYCenter-pos.Y());
        gr->SetPoint(N,xdum,ydum);

      }

      gr->Fit("pol1","FQ");
      int npoints = gr->GetN();
      //gr->Draw("A*");
      TF1 *fit = gr->GetFunction("pol1");
      std::vector<Double_t> parFit;
      parFit.push_back(fit->GetParameter(0));
      parFit.push_back(fit->GetParameter(1));
      double Chi2_min = fit->GetChisquare();
      double sigma2 = 25.0; //Size of the pad squared
      Int_t NDF = fit->GetNDF();
      track->SetFitPar(parFit);
      track->SetMinimum(Chi2_min);
      track->SetNFree(NDF);


      /*std::cout<<" Line Fit Results : "<<std::endl;
      std::cout<<parFit[0]<<" "<<parFit[1]<<"  "<<std::endl;
      std::cout<<" Chi2          : "<<Chi2_min<<" NDF : "<<NDF<<std::endl;
      std::cout<<" Chi2 reduced  : "<<(Chi2_min/sigma2/(double) npoints)<<std::endl;
      std::cout<<" Angle : "<<TMath::ATan2(parFit[1],1)*180.0/TMath::Pi()<<std::endl;*/

      //delete fit;
      //delete gr;
      return 0;



}

void ATRANSACN::ATRansac::FindVertex(std::vector<ATTrack*> tracks)
{

  // Assumes the minimum distance between two lines, with respect a given threshold, the first vertex candidate. Then evaluates the
  // distance of each remaining line with respect to the others (vertex) to decide the particles of the reaction.
  //std::cout<<" New find vertex call "<<std::endl;

  Double_t mad=10; // Minimum approach distance. This is the minimum distance between the lines in 3D. Must be bigger than fLineDistThreshold
  XYZVector c_1(-1000,-1000,-1000);
  XYZVector c_2(-1000,-1000,-1000);
  //std::vector<ATTrack*> *TrackCand;

      //Current  parametrization
      //x = p[0] + p[1]*t;
      //y = p[2] + p[3]*t;
      //z = t;
      // (x,y,z) = (p[0],p[2],0) + (p[1],p[3],1)*t

      // Equation of the Z axis in the solenoid frame
      XYZVector Z_0(0,0,1000.0);
      XYZVector Z_1(0,0,1);

      //Equation of the Y axis in the
      XYZVector Y_0(0,0,1000.0);
      XYZVector Y_1(0,1,0);

      // Counter clockwise rotations
      // Direction of the Z and Y axis in the detector frame (This is the rotated detector), this is used to calculater the theta and phi angles
      RotationX rx(-fTiltAng*TMath::Pi()/180.0);
      XYZVector Z_1_rot(rx*Z_1);

      RotationX ry(-fTiltAng*TMath::Pi()/180.0);
      XYZVector Y_1_rot(ry*Y_1);


      //Vector of the beam determined from the experimental data
      XYZVector BeamDir_1(-0.106359,-0.0348344,1.0);
      //TODO:: This is for 6.5 degrees of tilting angle. Need a function to set it.

      // Test each line against the others to find a vertex candidate
      for(Int_t i=0;i<tracks.size()-1;i++){

          ATTrack* track = tracks.at(i);
          track->SetTrackID(i);
          std::vector<Double_t> p = track->GetFitPar();

        if(p.size()>0)
        {
          XYZVector L_0(p[0], p[2], 0. );//p1
          XYZVector L_1(p[1], p[3], 1. );//d1

          //std::cout<<" L_1 p[0] : "<<p[0]<<" L_1 p[2] : "<<p[2]<<std::endl;
          //std::cout<<" L_1 p[1] : "<<p[1]<<" L_1 p[3] : "<<p[3]<<std::endl;

                    for(Int_t j=i+1; j<tracks.size();j++)
                    {
                        ATTrack* track_f = tracks.at(j);
                        track_f->SetTrackID(j);
                        std::vector<Double_t> p_f = track_f->GetFitPar();

                        if(p_f.size()>0)
                        {
                                      XYZVector L_f0(p_f[0], p_f[2], 0. );//p2
                                      XYZVector L_f1(p_f[1], p_f[3], 1. );//d2

                                      //std::cout<<" L_f0 p_f[0] : "<<p_f[0]<<" L_f1 p_f[2] : "<<p_f[2]<<std::endl;
                                      //std::cout<<" L_f1 p_f[1] : "<<p_f[1]<<" L_f1 p_f[3] : "<<p_f[3]<<std::endl;

                                      XYZVector L = L_1.Cross(L_f1);
                                      Double_t L_mag = L.Rho();
                                      XYZVector n_1 = L_1.Cross(L);
                                      XYZVector n_2 = L_f1.Cross(L);
                                      c_1 = L_0  + ( (L_f0 - L_0).Dot(n_2)*L_1  )/(  L_1.Dot(n_2)   );
                                      c_2 = L_f0 + ( (L_0  - L_f0).Dot(n_1)*L_f1 )/(  L_f1.Dot(n_1)  );



                                      //std::cout<<i<<" "<<j<<" "<<L_mag<<std::endl;

                                      if(L_mag>0)
                                      {
                                          XYZVector n = L/(Double_t)L_mag;
                                          Double_t d = TMath::Abs(n.Dot(L_0-L_f0));
                                          //std::cout<<" Distance of minimum approach : "<<d<<std::endl;
                                          Double_t num = L_1.X()*L_f1.X() + L_1.Y()*L_f1.Y() + L_1.Z()*L_f1.Z() ;
                                          Double_t den = TMath::Sqrt(L_1.X()*L_1.X() + L_1.Y()*L_1.Y() + L_1.Z()*L_1.Z())*TMath::Sqrt(L_f1.X()*L_f1.X() + L_f1.Y()*L_f1.Y() + L_f1.Z()*L_f1.Z());
                                          // NB:: The vector of hits is always sorted in time so the direction of porpagation of the particle is always positive.
                                          // This means that the angle will be always between 0 and 90 degree.
                                          // The vertex and the mean time of the track are needed to determine the direction of the track and then add 90 degrees.
                                          Double_t ang2 = TMath::ACos(num/den);
                                          TVector3 vertex1_buff;
                                          vertex1_buff.SetXYZ(c_1.X(),c_1.Y(),c_1.Z());
                                          TVector3 vertex2_buff;
                                          vertex2_buff.SetXYZ(c_2.X(),c_2.Y(),c_2.Z());

                                          //Angle with respect to solenoid
                                          Double_t angZi = GetAngleTracks(L_1,Z_1);
                                          Double_t angZj = GetAngleTracks(L_f1,Z_1);

                                          // Angles with respect to tilted detector
                                          Double_t angZDeti = GetAngleTracks(L_1,BeamDir_1);
                                          Double_t angZDetj = GetAngleTracks(L_f1,BeamDir_1);
                                          Double_t angYDeti = 0.0;//GetAngleTracks(L_1,Y_1_rot);
                                          Double_t angYDetj = 0.0;//GetAngleTracks(L_f1,Y_1_rot);


                                          track->SetAngleZAxis(angZi);
                                          track->SetAngleZDet(angZDeti);
                                          track->SetAngleYDet(angYDeti);

                                          track_f->SetAngleZAxis(angZj);
                                          track_f->SetAngleZDet(angZDetj);
                                          track_f->SetAngleYDet(angYDetj);

                                          track->SetTrackVertex(0.5*(vertex1_buff + vertex2_buff));
                                          track_f->SetTrackVertex(0.5*(vertex1_buff + vertex2_buff));


                                          if(d<mad){

                                             mad = d;
                                             //std::cout<<" New distance of minimum approach : "<<mad<<std::endl;
                                             //std::cout<<" Angle between lines i : "<<i<<" j : "<<j<<"  "<<ang2<<std::endl;

                                            // Global event variables
                                             fVertex_1.SetXYZ(c_1.X(),c_1.Y(),c_1.Z());
                                             fVertex_2.SetXYZ(c_2.X(),c_2.Y(),c_2.Z());
                                             fVertex_mean = 0.5*(fVertex_1 + fVertex_2);
                                             fVertex_tracks.first=i;
                                             fVertex_tracks.second=j;
                                             fMinimum = mad;


                                             if ( !CheckTrackID(track->GetTrackID(),&fTrackCand) ){
                                                fTrackCand.push_back(*track);
                                                PairedLines PL;
                                                PL.LinesID.first     = i;
                                                PL.LinesID.second    = j;
                                                PL.AngleZAxis.first  = angZi;
                                                PL.AngleZAxis.second = angZj;
                                                PL.AngleZDet.first  = angZDeti;
                                                PL.AngleZDet.second = angZDetj;
                                                PL.AngleYDet.first  = angYDeti;
                                                PL.AngleYDet.second = angYDetj;
                                                PL.minDist = mad;
                                                PL.meanVertex = 0.5*(fVertex_1 + fVertex_2);
                                                PL.angle = ang2;
                                                PLines.push_back(PL);
                                              }
                                             if ( !CheckTrackID(track_f->GetTrackID(),&fTrackCand) ){
                                                 fTrackCand.push_back(*track_f);
                                                 PairedLines PL;
                                                 PL.LinesID.first  = i;
                                                 PL.LinesID.second = j;
                                                 PL.AngleZAxis.first  = angZi;
                                                 PL.AngleZAxis.second = angZj;
                                                 PL.AngleZDet.first  = angZDeti;
                                                 PL.AngleZDet.second = angZDetj;
                                                 PL.AngleYDet.first  = angYDeti;
                                                 PL.AngleYDet.second = angYDetj;
                                                 PL.minDist = mad;
                                                 PL.meanVertex = 0.5*(fVertex_1 + fVertex_2);
                                                 PL.angle = ang2;
                                                 PLines.push_back(PL);
                                             }


                                          }

                                         if(d<fLineDistThreshold)
                                          {



                                             if ( !CheckTrackID(track->GetTrackID(),&fTrackCand) ){
                                              //std::cout<<" Add track"<<track->GetTrackID()<<std::endl;
                                              fTrackCand.push_back(*track);
                                              PairedLines PL;
                                              PL.LinesID.first  = i;
                                              PL.LinesID.second = j;
                                              PL.AngleZAxis.first  = angZi;
                                              PL.AngleZAxis.second = angZj;
                                              PL.AngleZDet.first  = angZDeti;
                                              PL.AngleZDet.second = angZDetj;
                                              PL.AngleYDet.first  = angYDeti;
                                              PL.AngleYDet.second = angYDetj;
                                              PL.minDist = d;
                                              PL.meanVertex = 0.5*(vertex1_buff + vertex2_buff);
                                              PL.angle = ang2;
                                              PLines.push_back(PL);
                                            }

                                             if ( !CheckTrackID(track_f->GetTrackID(),&fTrackCand) ){
                                              //std::cout<<" Add track f"<<track_f->GetTrackID()<<std::endl;
                                              fTrackCand.push_back(*track_f);
                                              PairedLines PL;
                                              PL.LinesID.first  = i;
                                              PL.LinesID.second = j;
                                              PL.AngleZAxis.first  = angZi;
                                              PL.AngleZAxis.second = angZj;
                                              PL.AngleZDet.first  = angZDeti;
                                              PL.AngleZDet.second = angZDetj;
                                              PL.AngleYDet.first  = angYDeti;
                                              PL.AngleYDet.second = angYDetj;
                                              PL.minDist = d;
                                              PL.meanVertex = 0.5*(vertex1_buff + vertex2_buff);
                                              PL.angle = ang2;
                                              PLines.push_back(PL);
                                             }
                                          }




                                      }

                        }//p_f size
                     }// End of track
            }//p size

       }// Loop over the tracks

     if(fTrackCand.size()>5) fTrackCand.resize(5); //Truncate the maximum number of lines to 5

}


Double_t ATRANSACN::ATRansac::distance2( double x,double y,double z, const double *p)
{

    // distance line point is D= | (xp-x0) cross  ux |
    // where ux is direction of line and x0 is a point in the line (like t = 0) and x1 is in t=1
    XYZVector xp(x,y,z);
    XYZVector x0(p[0], p[2], 0. );
    XYZVector x1(p[0] + p[1], p[2] + p[3], 1. );
    XYZVector u = (x1-x0).Unit();
    double d2 = ((xp-x0).Cross(u)).Mag2();
    return d2;
}

void ATRANSACN::ATRansac::SetLine(double t, const double *p, double &x, double &y, double &z)
{
      // a parameteric line is define from 6 parameters but 4 are independent
      // x0,y0,z0,z1,y1,z1 which are the coordinates of two points on the line
      // can choose z0 = 0 if line not parallel to x-y plane and z1 = 1;
      x = p[0] + p[1]*t;
      y = p[2] + p[3]*t;
      z = t;

}

Bool_t ATRANSACN::ATRansac::CheckTrackID(Int_t trackID, std::vector<ATTrack> *trackArray)
{
  auto it =  find_if( trackArray->begin(),trackArray->end(),[&trackID](ATTrack& track) {return track.GetTrackID()==trackID;}   );
  if(it != trackArray->end()){
     auto hitInd = std::distance<std::vector<ATTrack>::const_iterator>(trackArray->begin(),it);
     return kTRUE;
  }
  else return kFALSE;



}

Double_t ATRANSACN::ATRansac::GetAngleTracks(const ROOT::Math::XYZVector& vec1,const ROOT::Math::XYZVector& vec2)
{

  // NB:: The vector of hits is always sorted in time so the direction of porpagation of the particle is always positive.
  // This means that the angle will be always between 0 and 90 degree.
  // The vertex and the mean time of the track are needed to determine the direction of the track and then add 90 degrees.
  Double_t num = vec1.X()*vec2.X() + vec1.Y()*vec2.Y() + vec1.Z()*vec2.Z() ;
  Double_t den = TMath::Sqrt(vec1.X()*vec1.X() + vec1.Y()*vec1.Y() + vec1.Z()*vec1.Z())*TMath::Sqrt(vec2.X()*vec2.X() + vec2.Y()*vec2.Y() + vec2.Z()*vec2.Z());
  Double_t ang = TMath::ACos(num/den);

  return ang;

}

Int_t ATRANSACN::ATRansac::FindIndexTrack(Int_t index)
{
      auto it =  find_if( fTrackCand.begin(),fTrackCand.end(),[&index](ATTrack& track) {return track.GetTrackID()==index;}   );
      if(it != fTrackCand.end()){
         auto trackInd = std::distance<std::vector<ATTrack>::const_iterator>(fTrackCand.begin(),it);
         return trackInd;
      }
      else return -1;

}
