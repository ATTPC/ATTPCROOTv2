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

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

}

ATRANSACN::ATRansac::~ATRansac()
{
}


TVector3 ATRANSACN::ATRansac::GetVertex1()                                              {return fVertex_1;}
TVector3 ATRANSACN::ATRansac::GetVertex2()                                              {return fVertex_2;}
Double_t ATRANSACN::ATRansac::GetMinimum()                                              {return fMinimum;}
std::vector<ATTrack> ATRANSACN::ATRansac::GetTrackCand()                                {return fTrackCand;}


void ATRANSACN::ATRansac::SetModelType(int model)                                       { fRANSACModel = model;}
void ATRANSACN::ATRansac::SetDistanceThreshold(Float_t threshold)                       { fRANSACThreshold = threshold;}

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
          FindVertex(tracks);

          }
      }
    }

}

std::vector<ATTrack*> ATRANSACN::ATRansac::RansacPCL(ATEvent *event)
{

    std::vector<ATTrack*> tracks;


    //Data writer
	  pcl::PCDWriter writer;

	  // initialize PointClouds
  	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGBA>);
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);

    Int_t nHits = event->GetNumHits();
    cloud->points.resize(nHits);

    for(Int_t iHit=0; iHit<nHits; iHit++){

          ATHit* hit = event->GetHit(iHit);
          Int_t PadNumHit = hit->GetHitPadNum();
          TVector3 position = hit->GetPosition();
          cloud->points[iHit].x = position.X();
 			    cloud->points[iHit].y = position.Y();
			    cloud->points[iHit].z = position.Z();
          cloud->points[iHit].rgb = iHit; // Storing the position of the hit in the event container

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

// While 30% of the original cloud is still there
while (cloud->points.size () > 0.01 * nr_points)
 {
 // Segment the largest planar component from the remaining cloud
 //std::cout<<cloud->points.size()<<std::endl;

     seg.setInputCloud (cloud);
     seg.segment(*inliers, *coefficients);
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
     ATTrack *track = new ATTrack();

          for(Int_t iHit=0;iHit<(cloud_p->width*cloud_p->height);iHit++)
          {
            if(event->GetHit(cloud_p->points[iHit].rgb)) track->AddHit(event->GetHit(cloud_p->points[iHit].rgb));
          }
      tracks.push_back(track);
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

             /*std::cout<<parFit[0]<<" "<<parFit[1]<<"  "<<parFit[2]<<" "<<parFit[3]<<std::endl;
 		         std::cout<<" Chi2 (Minuit) : "<<Chi2_min<<" NDF : "<<NDF<<std::endl;
             std::cout<<" Chi2 reduced  : "<<(Chi2_min/sigma2/(double) npoints)<<std::endl;*/



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

void ATRANSACN::ATRansac::FindVertex(std::vector<ATTrack*> tracks)
{

  // Assumes the minimum distance between two lines, with respect a given threshold, the first vertex candidate. Then evaluates the
  // distance of each remaining line with respect to the others (vertex) to decide the particles of the reaction.


  Double_t mad=999999; // Minimum approach distance
  XYZVector c_1(-1000,-1000,-1000);
  XYZVector c_2(-1000,-1000,-1000);
  //std::vector<ATTrack*> *TrackCand;

      //Current  parametrization
      //x = p[0] + p[1]*t;
      //y = p[2] + p[3]*t;
      //z = t;
      // (x,y,z) = (p[0],p[2],0) + (p[1],p[3],1)*t

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


                                          if(d<mad){

                                             mad = d;
                                             //std::cout<<" New distance of minimum approach : "<<mad<<std::endl;
                                             fVertex_1.SetXYZ(c_1.X(),c_1.Y(),c_1.Z());
                                             fVertex_2.SetXYZ(c_2.X(),c_2.Y(),c_2.Z());
                                             fMinimum = mad;
                                             if ( !CheckTrackID(track->GetTrackID(),fTrackCand) ) fTrackCand.push_back(*track);
                                             if ( !CheckTrackID(track_f->GetTrackID(),fTrackCand) )  fTrackCand.push_back(*track_f);



                                          }

                                         if(d<fLineDistThreshold)
                                          {



                                             if ( !CheckTrackID(track->GetTrackID(),fTrackCand) ){
                                              //std::cout<<" Add track"<<track->GetTrackID()<<std::endl;
                                              fTrackCand.push_back(*track);
                                            }

                                             if ( !CheckTrackID(track_f->GetTrackID(),fTrackCand) ){
                                              //std::cout<<" Add track f"<<track_f->GetTrackID()<<std::endl;
                                              fTrackCand.push_back(*track_f);
                                             }
                                          }




                                      }

                                      //for(Int_t i=0;i<fTrackCand.size();i++)
                                          //std::cout<<fTrackCand.at(i).GetTrackID()<<std::endl;


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

Bool_t ATRANSACN::ATRansac::CheckTrackID(Int_t trackID, std::vector<ATTrack> trackArray)
{
  auto it =  find_if( trackArray.begin(),trackArray.end(),[&trackID](ATTrack& track) {return track.GetTrackID()==trackID;}   );
  if(it != trackArray.end()){
     auto hitInd = std::distance<std::vector<ATTrack>::const_iterator>(trackArray.begin(),it);
     return kTRUE;
  }
  else return kFALSE;



}
