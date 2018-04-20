#include "ATTrackingAnalysis.hh"
#include "TMath.h"
#ifdef _OPENMP
#include <omp.h>
#endif
#include <memory>

ClassImp(ATTrackingAnalysis)

ATTrackingAnalysis::ATTrackingAnalysis()
{

        fVertex = -1000.0;
        fEntTB_calc = -1000.0;
        fTrackFit = new TGraph();
        fFitResult = new TF1();
        fTrackFitXY = new TGraph();
        fFitResultXY = new TF1();
        fMultiplicity = 2;


}

ATTrackingAnalysis::~ATTrackingAnalysis()
{
  delete fTrackFit;
  delete fFitResult;
  delete fTrackFitXY;
  delete fFitResultXY;

}

void ATTrackingAnalysis::SetElossParameters(std::vector<Double_t> (&parE)[10])                             {for(Int_t i=0;i<10;i++) fElossPar[i] = parE[i];}
void ATTrackingAnalysis::SetEtoRParameters(std::vector<Double_t> (&parRtoE)[10])                           {for(Int_t i=0;i<10;i++) fEtoRPar[i] = parRtoE[i];}
void ATTrackingAnalysis::AddParticle(std::vector<std::pair<Int_t,Int_t>>& ptcl)                            {fParticleAZ=ptcl;}

void ATTrackingAnalysis::Analyze(ATPatternEvent *patternEvent,ATTrackingEventAna *trackingEventAna,TH2Poly* hPadPlane,const multiarray& PadCoord)
{
      if(fElossPar[0].size()>0 && fParticleAZ.size()>0){

                ATMCQMinimization *min = new ATMCQMinimization();
                min->ResetParameters();
                min->SetBackWardPropagation(kFALSE); //Disabled when vertex can be measured
                min->SetRangeChi2(kTRUE);

                Double_t step_par[10];
                auto init = std::initializer_list<Double_t>({8.0,8.0,0.1,0.1,0.1,0.1,0.0,0.0,0.1,0.0}); //MonteCarlo steps
                std::copy(init.begin(), init.end(),step_par);
                min->SetStepParameters(step_par);
                min->SetGainCalibration(fGain);//Micromegas + gas gain calibration
                min->SetLongDiffCoef(fCoefL);
                min->SetTranDiffCoef(fCoefT);

                Double_t *parameter = new Double_t[8];
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

                TVector3 Vertex_mean(0,0,0); // NB::Vertex from the Pattern Recognition analysis needed

                std::vector<ATTrack>  trackCand = patternEvent->GetTrackCand();

                for(Int_t i=0;i<trackCand.size();++i){

                  ATTrack* trackToMin = &trackCand.at(i);

                  parameter[0]=Vertex_mean.X();
                  parameter[1]=Vertex_mean.Y();
                  parameter[2]=Vertex_mean.Z();
                  parameter[3]= 0.0; //Initial Time Bucket 
                  parameter[6]= 0.0; //Theta
                  parameter[5]= 0.0; //Curvature
                  parameter[4]= 0.0; //Phi
                  parameter[7]=trackToMin->GetHitArray()->size();

                  //Custom function to pass the method to extract the Hit Array
                  std::function<std::vector<ATHit>*()> func = std::bind(&ATTrack::GetHitArray,trackToMin);
                  min->MinimizeGen(parameter,trackToMin,func,hPadPlane,PadCoord);
                  trackToMin->SetPosMin(min->GetPosXMin(),min->GetPosYMin(),min->GetPosZMin(),min->GetPosXBack(),min->GetPosYBack(),min->GetPosZBack());
                  trackToMin->SetPosExp(min->GetPosXExp(),min->GetPosYExp(),min->GetPosZExp(),min->GetPosXInt(),min->GetPosYInt(),min->GetPosZInt());
                  trackToMin->FitParameters.sThetaMin        = min->FitParameters.sThetaMin;
                  trackToMin->FitParameters.sEnerMin         = min->FitParameters.sEnerMin;
                  trackToMin->FitParameters.sPosMin          = min->FitParameters.sPosMin;
                  trackToMin->FitParameters.sBrhoMin         = min->FitParameters.sBrhoMin;
                  trackToMin->FitParameters.sBMin            = min->FitParameters.sBMin;
                  trackToMin->FitParameters.sPhiMin          = min->FitParameters.sPhiMin;
                  trackToMin->FitParameters.sChi2Min         = min->FitParameters.sChi2Min;
                  trackToMin->FitParameters.sVertexPos       = min->FitParameters.sVertexPos;
                  trackToMin->FitParameters.sVertexEner      = min->FitParameters.sVertexEner;
                  trackToMin->FitParameters.sMinDistAppr     = min->FitParameters.sMinDistAppr;
                  trackToMin->FitParameters.sNumMCPoint      = min->FitParameters.sNumMCPoint;
                  trackToMin->FitParameters.sNormChi2        = min->FitParameters.sNormChi2;
                  trackToMin->FitParameters.sChi2Q           = min->FitParameters.sChi2Q;
                  trackToMin->FitParameters.sChi2Range       = min->FitParameters.sChi2Range;
                  trackToMin->SetMCFit(kTRUE);
                  trackingEventAna->SetTrack(trackToMin);


                }



      }else{
        std::cout<<cYELLOW<<" ATTrackingAnalysis::Analyze - Warning! No Energy Loss or Particle parameters found! "<<cNORMAL<<std::endl;
      }




}


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
                min->SetBackWardPropagation(kFALSE); //Disabled when vertex can be measured
                min->SetRangeChi2(kTRUE);

                Double_t step_par[10];
                auto init = std::initializer_list<Double_t>({8.0,8.0,0.1,0.1,0.1,0.1,0.0,0.0,0.1,0.0}); //MonteCarlo steps
                std::copy(init.begin(), init.end(),step_par);
                min->SetStepParameters(step_par);
                min->SetGainCalibration(fGain);//Micromegas + gas gain calibration
                min->SetLongDiffCoef(fCoefL);
                min->SetTranDiffCoef(fCoefT);

                Double_t *parameter = new Double_t[8];
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

                //std::cout<<cYELLOW<<" =========== New Analysis event ============"<<cNORMAL<<std::endl;

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

                                if(trackAngle>(fTiltAng+1.0) ){
                                    //std::cout<<" Accepted Line : "<<track->GetTrackID()<<" with angle : "<<track->GetAngleZAxis()<<std::endl;
                                    track->SetGeoRange(track->GetLinearRange(Vertex_mean));
                                    Double_t ener = RtoEFunc(track->GetLinearRange(Vertex_mean)*(fPressure/760.0), fEtoRPar[0]);
                                    track->SetGeoEnergy(ener);
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
                                  Double_t trackAngle = 180.0*track->GetAngleZAxis()/TMath::Pi();
                                  TVector3 trackVertex = track->GetTrackVertex();
                                  Double_t dist = TMath::Sqrt( TMath::Power((trackVertex.X()-Vertex_mean.X()),2) + TMath::Power((trackVertex.Y()-Vertex_mean.Y()),2)
                                   + TMath::Power((trackVertex.Z()-Vertex_mean.Z()),2));

                                   //std::cout<<" Evaluating Line : "<<track->GetTrackID()<<" with angle : "<<track->GetAngleZAxis()<<std::endl;
                                   //std::cout<<" Distance from vertex : "<<dist<<std::endl;

                                  if(trackAngle>(fTiltAng+1.0) && dist<6.0){
                                      //3std::cout<<" Accepted Line : "<<track->GetTrackID()<<" with angle : "<<track->GetAngleZAxis()<<std::endl;
                                      track->SetGeoRange(track->GetLinearRange(Vertex_mean));
                                      Double_t ener = RtoEFunc(track->GetLinearRange(Vertex_mean)*(fPressure/760.0), fEtoRPar[0]);
                                      track->SetGeoEnergy(ener);
                                      mininizationTracks.push_back(track);

                                  }

                              }

                          }


                     }//if less 2 candidates

                  }// if trackCand has more than 1 track


                  // Calculation of the entrance time bucket and rotation of the detector frame to find the angles in the solenoid frame.
                  Double_t Z0 = Vertex_mean.Z() + TMath::Sqrt(TMath::Power(Vertex_mean.X(),2) + TMath::Power(Vertex_mean.Y(),2) )/TMath::Tan(fTiltAng*TMath::Pi()/180.0);
                  fEntTB_calc = GetTime(Z0);

                  //std::cout<<" Z0 : "<<Z0<<"  Vertex Time : "<<vertexTime<<std::endl;
                  //std::cout<<" TB0 : "<<fEntTB_calc<<std::endl;
                  //std::cout<<" Vertex_mean.Z() : "<<Vertex_mean.Z()<<std::endl;

                  fVertex       = fMaxRange  - (Z0 - Vertex_mean.Z()); //Vertex position for the excitation function.
                  fVertexEnergy = RtoEFunc(fVertex*(fPressure/760.0), fEtoRPar[0]); // Default case: 4He+4He. Every function and parameter list is the same.
                  min->SetEntTB(fEntTB_calc);
                  min->SetZGeoVertex(kTRUE);  //If true, the MC will start where the geometrical vertex is found,
                                              //according to the Z calibration done in the PSATask with the EntTB passed through the parameter file
                  min->SetEntZ0(Z0); //Calculated entrance position (actually this is the cathode position)


                  if(mininizationTracks.size()==fMultiplicity)
                  {

                        //Minimization starts
                        for(Int_t i=0;i<mininizationTracks.size();i++){

                          std::cout<<cRED<<" Minimizing particle : "<<i<<std::endl;
                          std::pair<Double_t,Double_t> angles = GetAnglesSolenoid(mininizationTracks.at(i));

                          ATTrack* trackToMin = mininizationTracks.at(i);

                          parameter[0]=Vertex_mean.X();
                          parameter[1]=Vertex_mean.Y();
                          parameter[2]=Vertex_mean.Z();
                          parameter[3]=(Int_t) vertexTime;
                          parameter[6]=angles.first; //Theta
                          parameter[5]=trackToMin->GetLinearRange(Vertex_mean);
                          parameter[4]=angles.second; //Phi
                          parameter[7]=trackToMin->GetHitArray()->size();

                          //Custom function to pass the method to extract the Hit Array
                          std::function<std::vector<ATHit>*()> func = std::bind(&ATTrack::GetHitArray,trackToMin);
                          min->MinimizeGen(parameter,trackToMin,func,hPadPlane,PadCoord);
                          trackToMin->SetPosMin(min->GetPosXMin(),min->GetPosYMin(),min->GetPosZMin(),min->GetPosXBack(),min->GetPosYBack(),min->GetPosZBack());
                          trackToMin->SetPosExp(min->GetPosXExp(),min->GetPosYExp(),min->GetPosZExp(),min->GetPosXInt(),min->GetPosYInt(),min->GetPosZInt());
                          trackToMin->FitParameters.sThetaMin        = min->FitParameters.sThetaMin;
                          trackToMin->FitParameters.sEnerMin         = min->FitParameters.sEnerMin;
                          trackToMin->FitParameters.sPosMin          = min->FitParameters.sPosMin;
                          trackToMin->FitParameters.sBrhoMin         = min->FitParameters.sBrhoMin;
                          trackToMin->FitParameters.sBMin            = min->FitParameters.sBMin;
                          trackToMin->FitParameters.sPhiMin          = min->FitParameters.sPhiMin;
                          trackToMin->FitParameters.sChi2Min         = min->FitParameters.sChi2Min;
                          trackToMin->FitParameters.sVertexPos       = min->FitParameters.sVertexPos;
                          trackToMin->FitParameters.sVertexEner      = min->FitParameters.sVertexEner;
                          trackToMin->FitParameters.sMinDistAppr     = min->FitParameters.sMinDistAppr;
                          trackToMin->FitParameters.sNumMCPoint      = min->FitParameters.sNumMCPoint;
                          trackToMin->FitParameters.sNormChi2        = min->FitParameters.sNormChi2;
                          trackToMin->FitParameters.sChi2Q           = min->FitParameters.sChi2Q;
                          trackToMin->FitParameters.sChi2Range       = min->FitParameters.sChi2Range;
                          trackToMin->SetMCFit(kTRUE);
                          trackingEventAna->SetTrack(trackToMin);


                        }



                        //After minimization we copy the tracks
                        //for(Int_t t=0;t<mininizationTracks.size();t++) trackingEventAna->SetTrack(mininizationTracks.at(t));
                        trackingEventAna->SetVertex(fVertex);
                        trackingEventAna->SetVertexEnergy(fVertexEnergy);
                        trackingEventAna->SetGeoVertex(Vertex_mean);


                  }




                  delete parameter;
                  delete min;

                  //For debugging: Visualize the rotation of the event in the X-Z and Y-Z plane
                  /*  TH2F* vis_XY = new TH2F("vis_XY","vis_XY",1000,-250,250,1000,-250,250);
                    TH2F* vis_XZ = new TH2F("vis_XZ","vis_XZ",1000,0,1000,1000,-250,250);
                    TH2F* vis_YZ = new TH2F("vis_YZ","vis_YZ",1000,0,1000,1000,-250,250);
                    TH2F* vis_RAD = new TH2F("vis_RAD","vis_RAD",1000,0,1000,1000,-250,250);
                    vis_YZ->SetMarkerColor(kRed);
                    vis_YZ->SetMarkerStyle(20);
                    vis_YZ->SetMarkerSize(1.0);
                    vis_XZ->SetMarkerStyle(20);
                    vis_XZ->SetMarkerSize(1.0);
                    vis_RAD->SetMarkerColor(kGreen);
                    vis_RAD->SetMarkerStyle(20);
                    vis_RAD->SetMarkerSize(1.0);
                    vis_XY->SetMarkerStyle(20);
                    vis_XY->SetMarkerSize(1.0);



                  if(trackCand.size()>0){
                      for(Int_t i=0;i<trackCand.size();i++){
                       std::vector<ATHit> hit_track = RotateEvent(&trackCand.at(i));

                              for(Int_t j=0;j<hit_track.size();j++)
                              {
                                ATHit hitT =  hit_track.at(j);
                                TVector3 posSol = hitT.GetPosition();
                                Double_t rad = TMath::Sqrt( TMath::Power(posSol.X(),2) + TMath::Power(posSol.Y(),2) );
                                //vis_XY->Fill(posSol.X(),posSol.Y());
                                //vis_XZ->Fill(posSol.Z(),posSol.X());
                                //vis_YZ->Fill(posSol.Z(),posSol.Y());
                                vis_RAD->Fill(posSol.Z(),rad);

                              }

                     }
                  }

                  //  vis_XZ->Draw();
                  //  vis_YZ->Draw("SAME");
                    vis_RAD->Draw();
                  //vis_XY->Draw();
                  */




  }else{
    std::cout<<cYELLOW<<" ATTrackingAnalysis::Analyze - Warning! No Energy Loss or Particle parameters found! "<<cNORMAL<<std::endl;
  }


}

void ATTrackingAnalysis::AnalyzeSimple(ATRANSACN::ATRansac *Ransac,ATTrackingEventAna *trackingEventAna,TH2Poly* hPadPlane,const multiarray& PadCoord)
{

      std::function<Double_t(Double_t,std::vector<Double_t>&)> RtoEFunc  = std::bind(GetEnergyFromRange,std::placeholders::_1,std::placeholders::_2);
      std::vector<ATTrack>  trackCand = Ransac->GetTrackCand();

      if(trackCand.size()>0){
            for(Int_t i=0;i<trackCand.size();i++)
            {
              ATTrack* track = &trackCand.at(i);
              Double_t ener = RtoEFunc(track->GetLinearRange()*(fPressure/760.0), fEtoRPar[0]);
              track->SetGeoRange(track->GetLinearRange());
              track->SetGeoEnergy(ener);
              track->SetGeoQEnergy(track->GetGeoQEnergy());
              trackingEventAna->SetTrack(track);
            }

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
      return par[0]*TMath::Sqrt(range) +  par[1]*( 1.0+par[2]/(TMath::Sqrt(range) + par[3])) + par[4]/( TMath::Power(TMath::Sqrt(range)+par[5],2) + par[6]   )  ;

    }else if(par.size()==5){

      return par[0]*TMath::Sqrt(range) + par[1] + par[2]*range + par[3]/(range+par[4]);

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
          Double_t vertex_time = GetTime(mean_Z);
          Ransac->SetVertexTime(vertex_time);
          return vertex_time;

}

Double_t ATTrackingAnalysis::GetTime(Double_t Z)
{
      //This function returns the original time according to the original Z calculation that takes fEntTb (and no fEntTB_calc)
      return ( 100.0*(Z - fZk)/(fTBTime*fDriftVelocity) ) + fEntTB;

}

std::pair<Double_t,Double_t> ATTrackingAnalysis::GetAnglesSolenoid(ATTrack* track)
{
          std::vector<ATHit> *hitArray = track->GetHitArray();
          TVector3 posRot;
          TVector3 posSol;
          std::pair<Double_t,Double_t> angles;
          fTrackFit->Set(0);
          fTrackFitXY->Set(0);

          Double_t x_mean=0.0;
          Double_t y_mean=0.0;
          Double_t z_mean=0.0;


          Double_t TiltAng  = -fTiltAng*TMath::Pi()/180.0;
          Double_t thetaPad = -fThetaPad;

          std::vector<TVector3> RotVectorArray;

          for(Int_t i=0;i<hitArray->size();i++)
          {


                  ATHit hit = hitArray->at(i);
                  TVector3 position = hit.GetPosition();




                  posRot.SetX(position.X()*TMath::Cos(thetaPad) - position.Y()*TMath::Sin(thetaPad));
                  posRot.SetY(position.X()*TMath::Sin(thetaPad) + position.Y()*TMath::Cos(thetaPad));
                  posRot.SetZ(  (-fEntTB_calc+hit.GetTimeStamp())*fTBTime*fDriftVelocity/100. + fZk  );

                  posSol.SetX(posRot.X());
                  posSol.SetY( -(fZk-posRot.Z())*TMath::Sin(TiltAng)   + posRot.Y()*TMath::Cos(TiltAng)  );
                  posSol.SetZ( posRot.Z()*TMath::Cos(TiltAng) - posRot.Y()*TMath::Sin(TiltAng)  );

                  RotVectorArray.push_back(posSol);
                  x_mean+=posSol.X();
                  y_mean+=posSol.Y();
                  z_mean+=posSol.Z();

                  Double_t rad = TMath::Sqrt( TMath::Power(posSol.X(),2) + TMath::Power(posSol.Y(),2) );

                  fTrackFit->SetPoint(fTrackFit->GetN(),posSol.Z(),rad);
                  fTrackFitXY->SetPoint(fTrackFitXY->GetN(),posSol.X(),posSol.Y());

          }

          x_mean/=hitArray->size();
          y_mean/=hitArray->size();
          z_mean/=hitArray->size();

          /*std::reverse(RotVectorArray.begin(), RotVectorArray.end());

          TVector3 originVec = RotVectorArray.front();
          Double_t phiAng=0.0;
          Double_t polarAng=0.0;
          Int_t cnt=0;

          Int_t ini_num_points = (Int_t) 0.1*RotVectorArray.size();

          for(Int_t j=ini_num_points;j<RotVectorArray.size();j++)
          {

              TVector3 vec = RotVectorArray.at(j);
              TVector3 dir_vec = (originVec - vec);
              phiAng+= dir_vec.Phi();
              polarAng+=dir_vec.Theta();
              cnt++;
              //std::cout<<" X ori : "<<originVec.X()<<" Y ori : "<<originVec.Y()<<" Z ori : "<<originVec.Z()<<std::endl;
              //std::cout<<" X  : "<<vec.X()<<" Y  : "<<vec.Y()<<" Z  : "<<vec.Z()<<std::endl;
              //std::cout<<" X dir  : "<<dir_vec.X()<<" Y dir : "<<dir_vec.Y()<<" Z dir : "<<dir_vec.Z()<<std::endl;
              //std::cout<<" Phi Angle : "<<phiAng<<std::endl;


          }

          phiAng/=cnt;
          polarAng/=cnt;
          */

          fTrackFit->Fit("pol1","FQ","",0,1000);
          fFitResult = fTrackFit->GetFunction("pol1");

          fTrackFitXY->Fit("pol1","FQ","",-1000,1000);
          fFitResultXY = fTrackFitXY->GetFunction("pol1");

          Double_t afit = 0.0;
          Double_t afitXY = 0.0;
          Double_t par0 = 0.0;
          Double_t par1 = 0.0;
          Double_t par0XY = 0.0;
          Double_t par1XY = 0.0;

          if(fFitResult){
           par0 = fFitResult->GetParameter(0);
           par1 = fFitResult->GetParameter(1);
         }

         if(fFitResult){
           par0XY = fFitResultXY->GetParameter(0);
           par1XY = fFitResultXY->GetParameter(1);
         }

          if(par1>=0) afit = TMath::Pi()-TMath::ATan2(TMath::Abs(par1),1);
          else if(par1<0)  afit = TMath::ATan2(TMath::Abs(par1),1);

          afitXY = TMath::ATan2(y_mean,x_mean);

          if(x_mean>0 && y_mean>0){
            //afitXY = TMath::Abs(TMath::ATan(par1XY));
            track->SetQuadrant(0);
          }else if(x_mean<0 && y_mean>0){
            //afitXY = TMath::Pi()-TMath::Abs(TMath::ATan(par1XY));
            track->SetQuadrant(1);
          }else if(x_mean<0 && y_mean<0){
            //afitXY = TMath::Pi()+TMath::Abs(TMath::ATan(par1XY));
            track->SetQuadrant(2);
          }else if(x_mean>0 && y_mean<0){
            //afitXY = 2.0*TMath::Pi()-TMath::Abs(TMath::ATan(par1XY));
            track->SetQuadrant(3);
          }else afitXY=0.0;

          //std::cout<<" X mean : "<<x_mean<<" Y mean : "<<y_mean<<std::endl;
          //std::cout<<" Scattering angle : "<<afit*180.0/TMath::Pi()<<std::endl;
          //std::cout<<" Azimuthal angle : "<<afitXY*180.0/TMath::Pi()<<std::endl;
          //std::cout<<" Azimuthal  angle : "<<phiAng*180.0/TMath::Pi()<<std::endl;
          //std::cout<<" Polar angle : "<<polarAng*180.0/TMath::Pi()<<std::endl;

          angles.first = afit;
          angles.second = afitXY;
          track->SetGeoTheta(afit);
          track->SetGeoPhi(afitXY);

          //fTrackFit->Draw("A*");
          //fTrackFitXY->Draw("A*");*/

          return angles;

}

std::vector<ATHit>  ATTrackingAnalysis::RotateEvent(ATTrack* track)
{

            std::vector<ATHit> *hitArray = track->GetHitArray();
            TVector3 posRot;
            TVector3 posSol;

            std::vector<ATHit> hit_track;

            Double_t TiltAng  = -fTiltAng*TMath::Pi()/180.0;
            Double_t thetaPad = -fThetaPad;


            for(Int_t i=0;i<hitArray->size();i++){

                    ATHit hit = hitArray->at(i);
                    TVector3 position = hit.GetPosition();
                    posRot.SetX(position.X()*TMath::Cos(thetaPad) - position.Y()*TMath::Sin(thetaPad));
                    posRot.SetY(position.X()*TMath::Sin(thetaPad) + position.Y()*TMath::Cos(thetaPad));
                    posRot.SetZ(  (-fEntTB_calc+hit.GetTimeStamp())*fTBTime*fDriftVelocity/100. + fZk  );



                    posSol.SetX(posRot.X());
                    posSol.SetY( -(fZk-posRot.Z())*TMath::Sin(TiltAng)   + posRot.Y()*TMath::Cos(TiltAng)  );
                    posSol.SetZ( posRot.Z()*TMath::Cos(TiltAng) - posRot.Y()*TMath::Sin(TiltAng)  );

                    ATHit hitRot(i,posSol,hit.GetCharge());
                    hit_track.push_back(hitRot);

            }

           return hit_track;

}
