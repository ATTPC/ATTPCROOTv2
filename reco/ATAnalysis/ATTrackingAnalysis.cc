#include "ATTrackingAnalysis.hh"
#include "TMath.h"
#ifdef _OPENMP
#include <omp.h>
#endif
#include <memory>

ClassImp(ATTrackingAnalysis)

ATTrackingAnalysis::ATTrackingAnalysis()
{

}

ATTrackingAnalysis::~ATTrackingAnalysis()
{
}

void ATTrackingAnalysis::SetElossParameters(std::vector<Double_t> (&parE)[10])                             {for(Int_t i=0;i<10;i++) fElossPar[i] = parE[i];}
void ATTrackingAnalysis::SetEtoRParameters(std::vector<Double_t> (&parRtoE)[10])                           {for(Int_t i=0;i<10;i++) fEtoRPar[i] = parRtoE[i];}
void ATTrackingAnalysis::AddParticle(std::vector<std::pair<Int_t,Int_t>>& ptcl)                            {fParticleAZ=ptcl;}


void ATTrackingAnalysis::Analyze(ATRANSACN::ATRansac *Ransac,ATTrackingEventAna *trackingEventAna,TH2Poly* hPadPlane,const multiarray& PadCoord)
{

  // Analysis function for a RANSAC event. An iterative loop over the tracks found by RANSAC is created.
  // A new minimization pointer has to be created for each independent track.
  // The algorithm will try to fit the track according to the possible particles added by the user from the macro/database.
  // For the moment the ELoss function is taken from this class but eventually the user can define their own parametrization.


  //TODO:: Only one function is considered for the moment

  if(fElossPar[0].size()>0 && fParticleAZ.size()>0){



                ATMCQMinimization *min = new ATMCQMinimization();
                min->ResetParameters();

                // Adding Eloss functions
                std::function<Double_t(Double_t,std::vector<Double_t>&)> ELossFunc = std::bind(GetEloss,std::placeholders::_1,std::placeholders::_2);
                min->AddELossFunc(ELossFunc);
                // Adding Range-Energy functions
                std::function<Double_t(Double_t,std::vector<Double_t>&)> RtoEFunc  = std::bind(GetEnergyFromRange,std::placeholders::_1,std::placeholders::_2);
                min->AddRtoEFunc(RtoEFunc);

                // Adding n-vectors of 11 parameters
                min->AddELossPar(fElossPar);
                min->AddRtoEPar(fEtoRPar);
                // Adding n-particles
                min->AddParticle(fParticleAZ);

                std::vector<ATTrack>  trackCand = Ransac->GetTrackCand();
                std::vector<ATRANSACN::ATRansac::PairedLines> trackCorr = Ransac->GetPairedLinesArray();

                std::pair<Int_t,Int_t> pairTrackIndex = Ransac->GetPairTracksIndex();
                std::vector<ATTrack*> mininizationTracks;
                Double_t vertexTime = GetVertexTime(Ransac);
                TVector3 Vertex_mean = Ransac->GetVertexMean();

                std::cout<<cYELLOW<<" =========== New Analysis event ============"<<cNORMAL<<std::endl;

                if(trackCand.size()>1){ // At least two tracks to form a vertex

                      // For debuggin purposes
                      /*for(Int_t i=0;i<trackCand.size();i++)
                      {
                        ATTrack track = trackCand.at(i);
                        std::cout<<" Track ID : "<<track.GetTrackID()<<std::endl;
                        std::cout<<" Loop Index : "<<i<<std::endl;
                        std::cout<<" Index from ID : "<<Ransac->FindIndexTrack(track.GetTrackID())<<std::endl;
                      }*/

                      for(Int_t i=0;i<2;i++){
                         Int_t trackID=0;
                         if(i==0) trackID = pairTrackIndex.first;
                         else if(i==1) trackID = pairTrackIndex.second;

                         Int_t indexID = Ransac->FindIndexTrack(trackID);

                        if(indexID>-1 && indexID<trackCand.size()){
                               ATTrack* track = &trackCand.at(indexID);

                               Double_t trackTime  = track->GetMeanTime();
                               Double_t trackAngle = 180.0*track->GetAngleZAxis()/TMath::Pi();

                               //std::cout<<cYELLOW<<" trackTime : "<<trackTime<<" Vertex Time : "<<vertexTime<<std::endl;
                               //std::cout<<" Track Angle : "<<trackAngle<<"  Tilting Angle+0.5 "<<(fTiltAng+0.5)<<cNORMAL<<std::endl;

                                if(trackAngle>(fTiltAng+0.5) ){
                                    std::cout<<" Accepted Line : "<<track->GetTrackID()<<" with angle : "<<track->GetAngleZAxis()<<std::endl;
                                    mininizationTracks.push_back(track);
                                }

                        }
                     }

                     //Try to find at least 2 candidates
                     if(mininizationTracks.size()<2)
                     {

                          for(Int_t i=0;i<trackCand.size();i++)
                          {
                              if(i!=pairTrackIndex.first && i!=pairTrackIndex.second)
                              {
                                  ATTrack* track = &trackCand.at(i);
                                  TVector3 trackVertex = track->GetTrackVertex();
                                  Double_t dist = TMath::Sqrt( TMath::Power((trackVertex.X()-Vertex_mean.X()),2) + TMath::Power((trackVertex.Y()-Vertex_mean.Y()),2)
                                   + TMath::Power((trackVertex.Z()-Vertex_mean.Z()),2));

                                   std::cout<<" Evaluating Line : "<<track->GetTrackID()<<" with angle : "<<track->GetAngleZAxis()<<std::endl;
                                   std::cout<<" Distance from vertex : "<<dist<<std::endl;


                              }

                          }


                     }//if less 2 candidates

                  }// if trackCand has more than 1 track


                  // TODO: Wrong, the EntTB is extrapolated from the vertex and tilting angle.
                  //min->SetEntTB(GetVertexTime(Ransac));



                delete min;


  }else{
    std::cout<<cYELLOW<<" ATTrackingAnalysis::Analyze - Warning! No Energy Loss or Particle parameters found! "<<cNORMAL<<std::endl;
  }


}

// Default ELoss function for protons in 20 torr Isobutane
Double_t ATTrackingAnalysis::GetEloss(Double_t c0,std::vector<Double_t>& par)
{
   if(par.size()==11){
   //return par[0]*(1./TMath::Power(c0,par[1]))*(1./(par[2]+par[3]/TMath::Power(c0,par[4])))+par[5]*TMath::Exp(-par[6]*TMath::Power((c0-par[7]),2));
   return par[0]*(1./TMath::Power(c0,par[1]))*(1./(par[2]+par[3]/TMath::Power(c0,par[4]))) + par[5]*TMath::Exp(par[6]*TMath::Power((c0+par[7]),2))
          + par[8]*TMath::Exp(par[9]*TMath::Power((c0+par[10]),2));
 }else{
   std::cerr<<cRED<<" ATHoughSpaceCircle::GetEloss -  Warning ! Wrong number of parameters."<<std::endl;
   return 0;
 }
}

Double_t ATTrackingAnalysis::GetEnergyFromRange(Double_t range,std::vector<Double_t>& par)
{
    //This is a parametrization calculated at 1 atm. The Range is normalized by the pressure
    if(par.size()==7)
    {
      return par[0]*TMath::Sqrt(range) +  par[1]*( par[2]+1.0/TMath::Sqrt(range) + par[3]) + par[4]/( TMath::Power(TMath::Sqrt(range)+par[5],2) + par[6]   )  ;

    }else
      std::cerr<<cRED<<" ATHoughSpaceCircle::GetEnergyFromRange -  Warning ! Wrong number of parameters."<<std::endl;
      return 0;

}

Double_t ATTrackingAnalysis::GetVertexTime(ATRANSACN::ATRansac *Ransac)
{

          // Function that infers the timebucket from the vertex.
          // It is based on the Z calibration: fZk - (fEntTB - peakIdx)*fTBTime*fDriftVelocity/100.;
          TVector3 Vertex1 = Ransac->GetVertex1();
          TVector3 Vertex2 = Ransac->GetVertex2();
          Double_t mean_Z = (Vertex1.Z()+Vertex2.Z())*0.5;
          Double_t vertex_time = ( 100.0*(mean_Z - fZk)/(fTBTime*fDriftVelocity) ) + fEntTB;
          Ransac->SetVertexTime(vertex_time);
          return vertex_time;

}
