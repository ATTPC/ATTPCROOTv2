#include "ATRansacMod.hh"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

#include<iostream>

using namespace std;



ClassImp(ATRansacMod)

ATRansacMod::ATRansacMod()
{

  fRANSACMaxIteration = 500;
	fRANSACMinPoints = 30;
	fRANSACThreshold = 15;
  fLineDistThreshold = 3.0;

  fVertex_1.SetXYZ(-10000,-10000,-10000);
  fVertex_2.SetXYZ(-10000,-10000,-10000);
  fVertexTime = -1000.0;
  fMinimum = -1.0;
}

ATRansacMod::~ATRansacMod()
{

}

void ATRansacMod::Init(ATEvent *event)
{

  Reset();

  std::vector<ATHit>* hitArray = event->GetHitArray();
  Int_t nHits = hitArray->size();

  for(Int_t iHit=0; iHit<nHits; iHit++){
      ATHit hit = hitArray->at(iHit);
      TVector3 position = hit.GetPosition();
      Double_t tq = hit.GetCharge();
      vX.push_back(position.X());
      vY.push_back(position.Y());
      vZ.push_back(position.Z());
      vQ.push_back(tq);
    }

	fOriginalCloudSize = vX.size();
	double TotalCharge=0;
	for(unsigned int i=0; i< vQ.size(); i++){
		TotalCharge += vQ[i];
	}
	fTotalCharge = TotalCharge;


}

void ATRansacMod::Reset()
{

	vX.clear();
	vY.clear();
	vZ.clear();
	vQ.clear();
  Vs.Clear();
  Ps.Clear();
  cluster_vector.clear();
}



vector<double> ATRansacMod::GetPDF(const std::vector<int>  samplesIdx){

  size_t pclouds = samplesIdx.size();
  double Tcharge = 0;
  for(int i=0;i<pclouds;i++) Tcharge += vQ[samplesIdx[i]];

  SetAvCharge(Tcharge/pclouds);
  std::vector<double> w;
  if(Tcharge>0)
  for(int i=0;i<pclouds;i++) w.push_back(vQ[samplesIdx[i]]/Tcharge);

  return w;
}

vector<int> ATRansacMod::RandSam(vector<int> indX)
{
  size_t pclouds = indX.size();
  std::vector<double> Proba = GetPDF(indX);
  int p1,p2;
  double w1,w2;

  //-------Uniform sampling
  p1=(int)(gRandom->Uniform(0,pclouds));

	 do{
      p2=(int)(gRandom->Uniform(0,pclouds));
    } while(p2==p1);

  vector<int> ranpair{indX[p1],indX[p2]};
  

  /*
  //--------Gaussian sampling
  double dist = 0;
  double sigma = 30.0;
  double y = 0;
  double gauss = 0;
  int counter = 0;
  p1=(int)(gRandom->Uniform(0,pclouds));
  TVector3 P1 ={vX[indX[p1]],vY[indX[p1]],vZ[indX[p1]]};
	 do{
      p2=(int)(gRandom->Uniform(0,pclouds));
      TVector3 P2 ={vX[indX[p2]],vY[indX[p2]],vZ[indX[p2]]};
      TVector3 dif = P2-P1;
      dist = dif.Mag();
      gauss = 1.0*exp(-1.0*pow(dist/sigma,2.0));
      y = (gRandom->Uniform(0,1));
      counter++;
      if(counter>20 && p2!=p1) break;
    } while(p2==p1 || y>gauss);

  vector<int> ranpair{indX[p1],indX[p2]};
  */

  /*
  //-------Weighted sampling
  bool cond = false;
  p1=(int)(gRandom->Uniform(0,pclouds));
   do{
      p2=(int)(gRandom->Uniform(0,pclouds));
      cond = false;
      double TwiceAvCharge = 2*GetAvCharge();
      if(Proba.size()==pclouds){
        w2 = gRandom->Uniform(0,TwiceAvCharge);
        if(Proba[p2]>=w2) cond = true;
      }else{
        w2 = 1;
        cond = true;
      }
    } while(p2==p1 || cond==false);

  vector<int> ranpair{indX[p1],indX[p2]};
  */

  /*
  //-------Weighted sampling + Gauss dist.
  bool cond = false;
  double dist = 0;
  double sigma = 2.0;
  double y = 0;
  double gauss = 0;
  p1=(int)(gRandom->Uniform(0,pclouds));
  TVector3 P1 ={vX[indX[p1]],vY[indX[p1]],vZ[indX[p1]]};
  do{
      p2=(int)(gRandom->Uniform(0,pclouds));
      TVector3 P2 ={vX[indX[p2]],vY[indX[p2]],vZ[indX[p2]]};
      TVector3 dif = P2-P1;
      dist = dif.Mag();
      gauss = 1.0*exp(-1*pow(dist/sigma,2));
      y = (gRandom->Uniform(0,1));

      cond = false;
      double TwiceAvCharge = 2*GetAvCharge();
      if(Proba.size()==pclouds){
        w2 = gRandom->Uniform(0,TwiceAvCharge);
        if(Proba[p2]>=w2) cond = true;
      }else{
        w2 = 1;
        cond = true;
      }

    } while(p2==p1 || cond==false || y>gauss);

  vector<int> ranpair{indX[p1],indX[p2]};
  */


  return ranpair;

}


void ATRansacMod::EstimModel(const std::vector<int>  samplesIdx)
{

  //line from two points
  TVector3 Po1 = {vX[samplesIdx[0]], vY[samplesIdx[0]], vZ[samplesIdx[0]]};
  TVector3 Po2 = {vX[samplesIdx[1]], vY[samplesIdx[1]], vZ[samplesIdx[1]]};

  Vs = Po2 - Po1;
  Ps = Po1;

}

double ATRansacMod::EstimError(int i)
{
    //distance point to line
    TVector3 newPoint = {vX[i], vY[i], vZ[i]};
    TVector3 vec = Ps - newPoint;
    TVector3 nD = Vs.Cross(vec);
	  double dist = nD.Mag()/Vs.Mag();

    return  dist;
}


void ATRansacMod::Solve()
{

    std::cout << "numero de puntos  "<<vX.size()<< '\n';
    std::vector<int> remainIndex;
    for (size_t i = 0; i < vX.size(); i++)
    remainIndex.push_back(i);

  	TVector3 V1, V2;
  	std::vector< int> inliners;
    inliners.clear();



  	  for(int i=0;i<fRANSACMaxIteration;i++){

        if(remainIndex.size()<fOriginalCloudSize*0.1) break; //less then 10% of total point cloud

        std::vector< int> Rsamples = RandSam(remainIndex);  //random sampling
        EstimModel(Rsamples); //estimate the linear model


        std::vector<int> inlIdxR;
        int nbInliers = 0;
        double weight = 0;

        for (auto j = remainIndex.begin(); j != remainIndex.end(); ++j){

  	      double error = EstimError(*j); //error of each point relative to the model
          error = error*error;
  	      if(error<(fRANSACThreshold*fRANSACThreshold)){
  	        	inlIdxR.push_back(*j);
              nbInliers++;
              weight +=error;
  	        	}
  		}


  	    if(nbInliers>fRANSACMinPoints){
  	    	TVector3 v1, v2;
  	      double chi2=Fit3D(inlIdxR,v1,v2);
          SetCluster(inlIdxR, weight/nbInliers, chi2,v1,v2);

          //Remove inliers from the point cloud (Sequential RANSAC)
          std::vector<int> tempRemain;
          std::set_difference(remainIndex.begin(), remainIndex.end(), inlIdxR.begin(), inlIdxR.end(),
          std::inserter(tempRemain, tempRemain.begin()));
          remainIndex = tempRemain;
          tempRemain.clear();
          inlIdxR.clear();

  			} //if a cluster was found

  		}//for RANSAC interactions
}

void ATRansacMod::CalcRANSACMod(ATEvent *event)
{

    std::cout << "Inicializa" << '\n';
    Init(event);
    std::cout << "Resuelve" << '\n';
    if(vX.size()>5){
      Solve();
	    AllClusters myClusters = GetClusters();
      //I know this step is stupid, but this part is meant to be in the future a clustering process
      std::cout << "Escribe tracks" << '\n';
      std::vector<ATTrack*> tracks = Clusters2Tracks(myClusters, event);

      Int_t tracksSize = tracks.size();
      std::cout<<"RansacMod tracks size : "<<tracksSize<<std::endl;
      if(tracksSize>1) FindVertex(tracks);
    }
}

void ATRansacMod::SetCluster(const std::vector<int> samplesIdx, const double cost, const double Chi2, TVector3 CP1, TVector3 CP2)
{
    Cluster cstr;
    cstr.ClusterIndex = samplesIdx;
    cstr.ClusterSize = samplesIdx.size();
    cstr.ClusterStrength = cost;
    cstr.ClusterChi2 = Chi2;
    cstr.ClusterFitP1 = CP1;
    cstr.ClusterFitP2 = CP2;
    cluster_vector.push_back(cstr);
}

std::vector<ATTrack*> ATRansacMod::Clusters2Tracks( AllClusters NClusters, ATEvent *event)
{

  std::vector<ATTrack*> tracks;

  std::vector<ATHit>* hits = event->GetHitArray();

  int numclus = NClusters.size();
	std::cout << "numero de clusters "<<numclus << '\n';

	for(int i = 0; i < numclus; i++){
      size_t clustersize = NClusters[i].ClusterSize;
      std::vector<int> indicesCluster = NClusters[i].ClusterIndex;
      //double costo =  NClusters[i].ClusterStrength;
      double Chi2 = NClusters[i].ClusterChi2;
      TVector3 punto1 = NClusters[i].ClusterFitP1;
      TVector3 punto2 = NClusters[i].ClusterFitP2;
      TVector3 pdiff = punto2 - punto1;
      ATTrack *track = new ATTrack();

      for(int j =0; j<clustersize; j++) track->AddHit(&hits->at(indicesCluster[j]));

      std::vector<Double_t> par;
      par.push_back(punto1.X()); //0
      par.push_back(pdiff.X()); //1
      par.push_back(punto1.Y()); //2
      par.push_back(pdiff.Y()); //3
      par.push_back(punto1.Z()); //4
      par.push_back(pdiff.Z()); //5

      track->SetFitPar(par);
      track->SetMinimum(Chi2);
      track->SetNFree(clustersize-6);

      tracks.push_back(track);

      delete track;

    }



  return tracks;
}


void ATRansacMod::FindVertex(std::vector<ATTrack*> tracks)
{

  // Assumes the minimum distance between two lines, with respect a given threshold, the first vertex candidate. Then evaluates the
  // distance of each remaining line with respect to the others (vertex) to decide the particles of the reaction.
  //std::cout<<" New find vertex call "<<std::endl;

  Double_t mad=10; // Minimum approach distance. This is the minimum distance between the lines in 3D. Must be bigger than fLineDistThreshold
  ROOT::Math::XYZVector c_1(-1000,-1000,-1000);
  ROOT::Math::XYZVector c_2(-1000,-1000,-1000);
  //std::vector<ATTrack*> *TrackCand;

      //Current  parametrization
      //x = p[0] + p[1]*t;
      //y = p[2] + p[3]*t;
      //z = p[4] + p[5]*t;
      // (x,y,z) = (p[0],p[2],p[4]) + (p[1],p[3],p[5])*t

      // Equation of the Z axis in the solenoid frame
      ROOT::Math::XYZVector Z_0(0,0,1000.0);
      ROOT::Math::XYZVector Z_1(0,0,1);

      //Equation of the Y axis in the
      ROOT::Math::XYZVector Y_0(0,0,1000.0);
      ROOT::Math::XYZVector Y_1(0,1,0);



      //Vector of the beam determined from the experimental data
     // ROOT::Math::XYZVector BeamDir_1(-0.106359,-0.0348344,1.0);
      ROOT::Math::XYZVector BeamDir_1(0.,0.,1.0);
      //TODO:: This is for 6.5 degrees of tilting angle. Need a function to set it.

      // Test each line against the others to find a vertex candidate
      for(Int_t i=0;i<int(tracks.size())-1;i++){

          ATTrack* track = tracks.at(i);
          track->SetTrackID(i);
          std::vector<Double_t> p = track->GetFitPar();

        if(p.size()>0)
        {

          ROOT::Math::XYZVector L_0(p[0], p[2], p[4] );//p1
          ROOT::Math::XYZVector L_1(p[1], p[3], p[5] );//d1

          //std::cout<<" L_1 p[0] : "<<p[0]<<" L_1 p[2] : "<<p[2]<<std::endl;
          //std::cout<<" L_1 p[1] : "<<p[1]<<" L_1 p[3] : "<<p[3]<<std::endl;

                    for(Int_t j=i+1; j<tracks.size();j++)
                    {
                        ATTrack* track_f = tracks.at(j);
                        track_f->SetTrackID(j);
                        std::vector<Double_t> p_f = track_f->GetFitPar();

                        if(p_f.size()>0)
                        {

                                      ROOT::Math::XYZVector L_f0(p_f[0], p_f[2], p_f[4] );//p2
                                      ROOT::Math::XYZVector L_f1(p_f[1], p_f[3], p_f[5] );//d2

                                      //std::cout<<" L_f0 p_f[0] : "<<p_f[0]<<" L_f1 p_f[2] : "<<p_f[2]<<std::endl;
                                      //std::cout<<" L_f1 p_f[1] : "<<p_f[1]<<" L_f1 p_f[3] : "<<p_f[3]<<std::endl;

                                      ROOT::Math::XYZVector L = L_1.Cross(L_f1);
                                      Double_t L_mag = L.Rho();
                                      ROOT::Math::XYZVector n_1 = L_1.Cross(L);
                                      ROOT::Math::XYZVector n_2 = L_f1.Cross(L);
                                      c_1 = L_0  + ( (L_f0 - L_0).Dot(n_2)*L_1  )/(  L_1.Dot(n_2)   );
                                      c_2 = L_f0 + ( (L_0  - L_f0).Dot(n_1)*L_f1 )/(  L_f1.Dot(n_1)  );



                                      //std::cout<<i<<" "<<j<<" "<<L_mag<<std::endl;

                                      if(L_mag>0)
                                      {
                                          ROOT::Math::XYZVector n = L/(Double_t)L_mag;
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


Double_t ATRansacMod::GetAngleTracks(const ROOT::Math::XYZVector& vec1,const ROOT::Math::XYZVector& vec2)
{

  // NB:: The vector of hits is always sorted in time so the direction of porpagation of the particle is always positive.
  // This means that the angle will be always between 0 and 90 degree.
  // The vertex and the mean time of the track are needed to determine the direction of the track and then add 90 degrees.
  Double_t num = vec1.X()*vec2.X() + vec1.Y()*vec2.Y() + vec1.Z()*vec2.Z() ;
  Double_t den = TMath::Sqrt(vec1.X()*vec1.X() + vec1.Y()*vec1.Y() + vec1.Z()*vec1.Z())*TMath::Sqrt(vec2.X()*vec2.X() + vec2.Y()*vec2.Y() + vec2.Z()*vec2.Z());
  Double_t ang = TMath::ACos(num/den);

  return ang;

}

Bool_t ATRansacMod::CheckTrackID(Int_t trackID, std::vector<ATTrack> *trackArray)
{
  auto it =  find_if( trackArray->begin(),trackArray->end(),[&trackID](ATTrack& track) {return track.GetTrackID()==trackID;}   );
  if(it != trackArray->end()){
     auto hitInd = std::distance<std::vector<ATTrack>::const_iterator>(trackArray->begin(),it);
     return kTRUE;
  }
  else return kFALSE;

}


Double_t ATRansacMod::Fit3D(vector<int> inliners, TVector3& V1, TVector3& V2)
{

    //------3D Line Regression
    //----- adapted from: http://fr.scribd.com/doc/31477970/Regressions-et-trajectoires-3D
    int R, C;
    double Q;
    double Xm,Ym,Zm;
    double Xh,Yh,Zh;
    double a,b;
    double Sxx,Sxy,Syy,Sxz,Szz,Syz;
    double theta;
    double K11,K22,K12,K10,K01,K00;
    double c0,c1,c2;
    double p,q,r,dm2;
    double rho,phi;

    Q=Xm=Ym=Zm=0.;
		double total_charge=0;
    Sxx=Syy=Szz=Sxy=Sxz=Syz=0.;

    for (auto i : inliners)
    {
        Q+=vQ[i]/10.;
        Xm+=vX[i]*vQ[i]/10.;
        Ym+=vY[i]*vQ[i]/10.;
        Zm+=vZ[i]*vQ[i]/10.;
        Sxx+=vX[i]*vX[i]*vQ[i]/10.;
        Syy+=vY[i]*vY[i]*vQ[i]/10.;
        Szz+=vZ[i]*vZ[i]*vQ[i]/10.;
        Sxy+=vX[i]*vY[i]*vQ[i]/10.;
        Sxz+=vX[i]*vZ[i]*vQ[i]/10.;
        Syz+=vY[i]*vZ[i]*vQ[i]/10.;
    }
    //vTrackCharge.push_back(total_charge);

    Xm/=Q;
    Ym/=Q;
    Zm/=Q;
    Sxx/=Q;
    Syy/=Q;
    Szz/=Q;
    Sxy/=Q;
    Sxz/=Q;
    Syz/=Q;
    Sxx-=(Xm*Xm);
    Syy-=(Ym*Ym);
    Szz-=(Zm*Zm);
    Sxy-=(Xm*Ym);
    Sxz-=(Xm*Zm);
    Syz-=(Ym*Zm);

    theta=0.5*atan((2.*Sxy)/(Sxx-Syy));

    K11=(Syy+Szz)*pow(cos(theta),2)+(Sxx+Szz)*pow(sin(theta),2)-2.*Sxy*cos(theta)*sin(theta);
    K22=(Syy+Szz)*pow(sin(theta),2)+(Sxx+Szz)*pow(cos(theta),2)+2.*Sxy*cos(theta)*sin(theta);
    K12=-Sxy*(pow(cos(theta),2)-pow(sin(theta),2))+(Sxx-Syy)*cos(theta)*sin(theta);
    K10=Sxz*cos(theta)+Syz*sin(theta);
    K01=-Sxz*sin(theta)+Syz*cos(theta);
    K00=Sxx+Syy;

    c2=-K00-K11-K22;
    c1=K00*K11+K00*K22+K11*K22-K01*K01-K10*K10;
    c0=K01*K01*K11+K10*K10*K22-K00*K11*K22;


    p=c1-pow(c2,2)/3.;
    q=2.*pow(c2,3)/27.-c1*c2/3.+c0;
    r=pow(q/2.,2)+pow(p,3)/27.;


    if(r>0) dm2=-c2/3.+pow(-q/2.+sqrt(r),1./3.)+pow(-q/2.-sqrt(r),1./3.);
    if(r<0)
    {
        rho=sqrt(-pow(p,3)/27.);
        phi=acos(-q/(2.*rho));
        dm2=min(-c2/3.+2.*pow(rho,1./3.)*cos(phi/3.),min(-c2/3.+2.*pow(rho,1./3.)*cos((phi+2.*TMath::Pi())/3.),-c2/3.+2.*pow(rho,1./3.)*cos((phi+4.*TMath::Pi())/3.)));
    }

    a=-K10*cos(theta)/(K11-dm2)+K01*sin(theta)/(K22-dm2);
    b=-K10*sin(theta)/(K11-dm2)-K01*cos(theta)/(K22-dm2);

    Xh=((1.+b*b)*Xm-a*b*Ym+a*Zm)/(1.+a*a+b*b);
    Yh=((1.+a*a)*Ym-a*b*Xm+b*Zm)/(1.+a*a+b*b);
    Zh=((a*a+b*b)*Zm+a*Xm+b*Ym)/(1.+a*a+b*b);

    V1.SetXYZ(Xm,Ym,Zm);
    V2.SetXYZ(Xh,Yh,Zh);

    return(fabs(dm2/Q));
}
