#include "ATHoughSpaceCircle.hh"
#include "ATMCMinimization.hh"
#include "ATMCQMinimization.hh"
#ifdef _OPENMP
#include <omp.h>
#endif

#include "TH1.h"
#include "TH2.h"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(ATHoughSpaceCircle)

ATHoughSpaceCircle::ATHoughSpaceCircle()
{
    HoughPar.clear();
    fXCenter=-10000.0;
    fYCenter=-10000.0;
    HistHoughXY = new TH2F("HistHoughXY","HistHoughXY",50,0,3.15,100,-1000,1000);
    HistHoughAux = new TH2F("HistHoughAux","HistHoughAux",500,0,3.15,1000,-500,500);
    fRadius = new std::vector<Double_t>;
    fTimeStamp = new std::vector<Int_t>;
    fPhi = new std::vector<Double_t>;
    fDl = new std::vector<Double_t>;
    fTheta = new std::vector<Double_t>;
    fIniHit = new ATHit();
    fIniHitRansac = new ATHit();
    fClusteredHits = new std::vector<ATHit>;
    for(Int_t i;i<8;i++) fParameter[i]=0.0;

    kDebug = kFALSE;
    kHough = kFALSE;



    fIniTS = 0;
    fIniRadius = 0.0;
    fIniRadiusRansac = 0.0;
    fIniPhi = 0.0;
    fIniPhiRansac = 0.0;
    fIniHitID =0;
    fStdDeviationLimit = 4.0;


    FairLogger *fLogger=FairLogger::GetLogger();
    ATDigiPar *fPar;

    FairRun *run = FairRun::Instance();
    if (!run)
      fLogger -> Fatal(MESSAGE_ORIGIN, "No analysis run!");

    FairRuntimeDb *db = run -> GetRuntimeDb();
    if (!db)
      fLogger -> Fatal(MESSAGE_ORIGIN, "No runtime database!");

    fPar = (ATDigiPar *) db -> getContainer("ATDigiPar");
    if (!fPar)
      fLogger -> Fatal(MESSAGE_ORIGIN, "ATDigiPar not found!!");

      fDriftVelocity = fPar -> GetDriftVelocity();
      fTBTime = fPar -> GetTBTime();
}

ATHoughSpaceCircle::~ATHoughSpaceCircle()
{
	delete HistHoughXY;
  delete HistHoughAux;
}


//void ATHoughSpaceCircle::SetThreshold(Double_t value) {fThreshold = value;}

std::pair<Double_t,Double_t> ATHoughSpaceCircle::GetHoughPar() {return fHoughLinePar;}

std::vector<Double_t>  ATHoughSpaceCircle::GetRansacPar()      {return fRansacLinePar;}

TH2F* ATHoughSpaceCircle::GetHoughSpace(TString ProjPlane)   {return HistHoughXY;}

void ATHoughSpaceCircle::CalcMultiHoughSpace(ATEvent* event)
{


}

void ATHoughSpaceCircle::CalcHoughSpace(ATEvent* event)
{


}

void ATHoughSpaceCircle::CalcHoughSpace(ATEvent* event,TH2Poly* hPadPlane)
{

  Int_t nHits = event->GetNumHits();
  Int_t nstep = 10;
  Double_t drift_cal = fDriftVelocity*fTBTime/100.0;//mm


  ATMinimization *min = new ATMCMinimization();
  min->ResetParameters();

  for(Int_t iHit=0; iHit<(nHits-nstep); iHit++){

              //std::cout<<omp_get_num_threads()<<std::endl;
              ATHit hit = event->GetHitArray()->at(iHit);
              ATHit hit_forw = event->GetHitArray()->at(iHit+nstep);
              ATHit hit_next = event->GetHitArray()->at(iHit+1);
              Int_t PadNumHit = hit.GetHitPadNum();
              Int_t PadNumHit_forw = hit_forw.GetHitPadNum();

              if(hit.GetCharge()<fThreshold) continue;
              TVector3 position = hit.GetPosition();
              TVector3 position_forw = hit_forw.GetPosition();
              TVector3 position_next = hit_next.GetPosition();


              Double_t hitdist = TMath::Sqrt( TMath::Power(position.X()-position_next.X(),2) + TMath::Power(position.Y()-position_next.Y(),2) + TMath::Power(position.Z()-position_next.Z(),2)   );


              for(Int_t itheta = 0; itheta <1023; itheta++){
                  Float_t angle = TMath::Pi()*(static_cast<Float_t>(itheta)/1023);
                  Float_t d0_XY_inv = 2.0* ( (position.Y()-position_forw.Y())*TMath::Sin(angle) + (position.X()-position_forw.X())*TMath::Cos(angle)  )
                  / ( (TMath::Power(position.Y(),2)-TMath::Power(position_forw.Y(),2) ) +   (TMath::Power(position.X(),2)-TMath::Power(position_forw.X(),2) )        );
                  HistHoughXY->Fill(angle,1.0/d0_XY_inv);
              }
    }//Hit loop
//    }// Parallel region


                  Int_t locmaxx,locmaxy,locmaxz;
                  HistHoughXY->GetMaximumBin(locmaxx,locmaxy,locmaxz);
                  Double_t xpos = HistHoughXY->GetXaxis()->GetBinCenter(locmaxx);
                  Double_t ypos = HistHoughXY->GetYaxis()->GetBinCenter(locmaxy);

                  fXCenter =  ypos*TMath::Cos(xpos);
                  fYCenter =  ypos*TMath::Sin(xpos);


                    fIniRadius=0.0;
                    fIniPhi=0.0;
                    fIniTheta=0.0;


                    for(Int_t iHit=0; iHit<nHits-1; iHit++){

                           ATHit hit = event->GetHitArray()->at(iHit);
                           TVector3 position = hit.GetPosition();
                           ATHit hit_forw = event->GetHitArray()->at(iHit+1);
                           TVector3 position_forw = hit_forw.GetPosition();

                           fRadius->push_back(TMath::Sqrt(  TMath::Power((fXCenter-position.X()),2)   +  TMath::Power((fYCenter-position.Y()),2)    ));
                           fTimeStamp->push_back(hit.GetTimeStamp());
                           fPhi->push_back(TMath::ATan2(fXCenter-position.X(),fYCenter-position.Y()));
                           Double_t dl = TMath::Sqrt( TMath::Power(position_forw.X()-position.X(),2) + TMath::Power(position_forw.Y()-position.Y(),2)  );
                           Double_t dz = (hit_forw.GetTimeStamp()-hit.GetTimeStamp())*drift_cal;
                           //fTheta->push_back(TMath::ATan2(dl,dz)); // Obsolete


                              //Second Hough Space Calculation for PhixRadius

                                 for(Int_t itheta = 0; itheta <1023; itheta++){
                                      Float_t angle = TMath::Pi()*(static_cast<Float_t>(itheta)/1023);
                                      Float_t d0_XY = (TMath::Cos(angle)*fPhi->at(iHit)*fRadius->at(iHit))  +  (TMath::Sin(angle)*hit.GetTimeStamp());
                                      HistHoughAux->Fill(angle,d0_XY);
                                      //FillHoughMap(angle,d0_XY);

                                }


                    }

                  fHoughLinePar =  CalHoughParameters(HistHoughAux);//Histo
                  //std::pair<Double_t,Double_t> HoughLinePar = CalHoughParameters();//std
                  Double_t IniPosZ =0.0;


                  Int_t IniTSBuff=0;
                  Double_t IniRadiusBuff =0.0;
                  Double_t IniPhiBuff =0.0;


                  for(Int_t iHit=0; iHit<nHits-1; iHit++)
                  {
                    ATHit hit = event->GetHitArray()->at(iHit);
                    TVector3 position = hit.GetPosition();
                    Double_t geo_dist = TMath::Abs (TMath::Cos(fHoughLinePar.first)*fPhi->at(iHit)*fRadius->at(iHit)  + TMath::Sin(fHoughLinePar.first)*hit.GetTimeStamp()  - fHoughLinePar.second);
                    Double_t hit_dist = TMath::Sqrt( TMath::Power( (fPhi->at(iHit)*fRadius->at(iHit) - IniPhiBuff*IniRadiusBuff),2) + TMath::Power(hit.GetTimeStamp()-IniTSBuff ,2) );
                    //if(hit_dist<50) std::cout<<" Hist Dist pre :"<<hit_dist<<" Geo Dist : "<<geo_dist<<std::endl;



                            if(geo_dist<10.0){

                                      IniTSBuff = hit.GetTimeStamp();
                                      IniRadiusBuff = fRadius->at(iHit);
                                      IniPhiBuff = fPhi->at(iHit);

                                        if(hit_dist<10.0){


                                          fClusteredHits->push_back(&hit);
                                          fIniHit->SetHit(hit.GetHitPadNum(),hit.GetHitID(),position.X(),position.Y(),position.Z(),hit.GetCharge());
                                          IniPosZ = position.Z();
                                          fIniHit->SetTimeStamp(hit.GetTimeStamp());
                                          fIniTS = IniTSBuff;
                                          fIniRadius = IniRadiusBuff;
                                          fIniPhi = IniPhiBuff;
                                          fIniHitID = iHit; // Easier and faster to use the position of the hit in the vector


                                        }




                                }


                   }// End of clustering algorithm



                    Double_t dl_theta=0.0;
                    Double_t theta_avg=0.0;
                    Int_t clusterSize=fClusteredHits->size();
                    Int_t TB_Scan = 0;
                    for(Int_t iHitTB=0;iHitTB<clusterSize-1;iHitTB++){

                      Double_t posx=0.0;
                      Double_t posy=0.0;
                      Double_t posz=0.0;
                      Double_t totCharge=0.0;
                      Double_t cmsHits_X =0.0;
                      Double_t cmsHits_Y =0.0;
                      Int_t TB =0;
                      Double_t posx_forw=0.0;
                      Double_t posy_forw=0.0;
                      Double_t posz_forw=0.0;
                      Double_t totCharge_forw=0.0;
                      Double_t cmsHits_X_forw =0.0;
                      Double_t cmsHits_Y_forw =0.0;
                      Int_t TB_forw = 0;


                      // TODO: Only neighboring TB are taken into account
                      std::vector<ATHit>  fClusteredTBHits;
                      std::vector<ATHit>  fClusteredTBHits_forw;


                      fClusteredTBHits=min->GetTBHitArray(fIniTS-iHitTB-TB_Scan,fClusteredHits);

                    if(fClusteredTBHits.size()>0){ //Found at least 1 clustered hit in that time bucket, if not go to the next loop iteration
                            //std::cout<<" ================================ "<<std::endl;
                            //std::cout<<"  Starting cluster with TB : "<<fIniTS-iHitTB<<std::endl;
                            fClusteredTBHits_forw=min->GetTBHitArray(fIniTS-iHitTB-1-TB_Scan,fClusteredHits);

                     if(fClusteredTBHits_forw.size()==0){// If forward hit is empty scan until the end of the vector or until the next non-zero solution


                          while(fClusteredTBHits_forw.size()==0  &&  fIniTS-iHitTB-1-TB_Scan>0){ //TODO: Use a lambda function for this mess
                              TB_Scan++;
                              fClusteredTBHits_forw=min->GetTBHitArray(fIniTS-iHitTB-1-TB_Scan,fClusteredHits);
                              //std::cout<<TB_Scan<<std::endl;
                              //std::cout<<fClusteredTBHits_forw.size()<<std::endl;
                          }

                      }

                      if(fClusteredTBHits_forw.size()>0){




                                    if( fClusteredTBHits.size()>0  && fClusteredTBHits_forw.size()>0 ){

                                          for(Int_t iHit=0;iHit<fClusteredTBHits.size();iHit++){
                                            ATHit hitTB = fClusteredTBHits.at(iHit);
                                            TVector3 positionTB = hitTB.GetPosition();
                                            TB = hitTB.GetTimeStamp();
                                            cmsHits_X+= positionTB.X()*hitTB.GetCharge();
                                            cmsHits_Y+= positionTB.Y()*hitTB.GetCharge();
                                            totCharge+=hitTB.GetCharge();
                                            //std::cout<<" Hit TB : "<<TB<<" Position X : "<<positionTB.X()<<" Position Y : "<<positionTB.Y()<<std::endl;
                                          }

                                          for(Int_t iHit=0;iHit<fClusteredTBHits_forw.size();iHit++){
                                            ATHit hitTB_forw = fClusteredTBHits_forw.at(iHit);
                                            TVector3 positionTB_forw = hitTB_forw.GetPosition();
                                            TB_forw = hitTB_forw.GetTimeStamp();
                                            cmsHits_X_forw+= positionTB_forw.X()*hitTB_forw.GetCharge();
                                            cmsHits_Y_forw+= positionTB_forw.Y()*hitTB_forw.GetCharge();
                                            totCharge_forw+=hitTB_forw.GetCharge();
                                            //std::cout<<" Hit TB forw : "<<TB_forw<<" Position forw X : "<<positionTB_forw.X()<<" Position forw Y : "<<positionTB_forw.Y()<<std::endl;
                                          }

                                    }

                                    posx=cmsHits_X/totCharge;
                                    posy=cmsHits_Y/totCharge;
                                    posx_forw=cmsHits_X_forw/totCharge_forw;
                                    posy_forw=cmsHits_Y_forw/totCharge_forw;
                                    //std::cout<<" Average Pos X : "<<posx<<" Average Pos Y : "<<posy<<std::endl;
                                    //std::cout<<" Average Pos Forward X : "<<posx_forw<<" Average Pos Forward Y : "<<posy_forw<<std::endl;

                          }//Hit forward if>0
                        }//Hit if>0

                        Double_t distXY = TMath::Sqrt( TMath::Power(posx-posx_forw,2) + TMath::Power(posy-posy_forw,2) );
                        Double_t dl = (TB-TB_forw)*drift_cal;
                        theta_avg=TMath::ATan2(distXY,dl);
                        fTheta->push_back(theta_avg);
                        fDl->push_back(TB);
                        //std::cout<<TB<<std::endl;


                   }// Loop over clusterSize


                  Double_t fThetaVal=0.0;
                  Int_t thetacnt=0; //Needed to remove theta values with 0

                  if(!fTheta->empty()){

                      for(Int_t i=0;i<fTheta->size();++i){
                          if(fTheta->at(i)>0){
                           fThetaVal+= fTheta->at(i);
                           thetacnt++;
                         }
                      }
                      fIniTheta = fThetaVal/thetacnt;
                  }


                  TVector3 IniHitPos = fIniHit->GetPosition();
                  Double_t *parameter = new Double_t[8];
                  parameter[0]=IniHitPos.X();
                  parameter[1]=IniHitPos.Y();
                  parameter[2]=IniHitPos.Z();
                  parameter[3]=fIniTS;
                  parameter[4]=fIniPhi;
                  parameter[5]=fIniRadius;
                  parameter[6]=fIniTheta;
                  parameter[7]=fIniHitID;

                  for(Int_t i=0;i<8;i++) fParameter[i]=parameter[i];


                  Double_t HoughAngleDeg = fHoughLinePar.first*180.0/TMath::Pi();


              if (   HoughAngleDeg<90.0 && HoughAngleDeg>45.0 ) { // Check RxPhi plot to adjust the angle

                 min->MinimizeOptMap(parameter,event,hPadPlane);
                 fPosXmin = min->GetPosXMin();
                 fPosYmin = min->GetPosYMin();
                 fPosZmin = min->GetPosZMin();
                 fPosXexp = min->GetPosXExp();
                 fPosYexp = min->GetPosYExp();
                 fPosZexp = min->GetPosZExp();
                 fPosXinter = min->GetPosXInt();
                 fPosYinter = min->GetPosYInt();
                 fPosZinter = min->GetPosZInt();
                 fPosXBack = min->GetPosXBack();
                 fPosYBack = min->GetPosYBack();
                 fPosZBack = min->GetPosZBack();
                 ATHoughSpaceCircle::FitParameters.sThetaMin        = min->FitParameters.sThetaMin;
                 ATHoughSpaceCircle::FitParameters.sThetaMin        = min->FitParameters.sThetaMin;
                 ATHoughSpaceCircle::FitParameters.sEnerMin         = min->FitParameters.sEnerMin;
                 ATHoughSpaceCircle::FitParameters.sPosMin          = min->FitParameters.sPosMin;
                 ATHoughSpaceCircle::FitParameters.sBrhoMin         = min->FitParameters.sBrhoMin;
                 ATHoughSpaceCircle::FitParameters.sBMin            = min->FitParameters.sBMin;
                 ATHoughSpaceCircle::FitParameters.sPhiMin          = min->FitParameters.sPhiMin;
                 ATHoughSpaceCircle::FitParameters.sChi2Min         = min->FitParameters.sChi2Min;
                 ATHoughSpaceCircle::FitParameters.sVertexPos       = min->FitParameters.sVertexPos;
                 ATHoughSpaceCircle::FitParameters.sVertexEner      = min->FitParameters.sVertexEner;
                 ATHoughSpaceCircle::FitParameters.sMinDistAppr     = min->FitParameters.sMinDistAppr;
                 ATHoughSpaceCircle::FitParameters.sNumMCPoint      = min->FitParameters.sNumMCPoint;
                 ATHoughSpaceCircle::FitParameters.sNormChi2        = min->FitParameters.sNormChi2;
                }
                else
                {
                  fPosXmin.clear();
                  fPosYmin.clear();
                  fPosZmin.clear();
                  fPosXexp.clear();
                  fPosYexp.clear();
                  fPosZexp.clear();
                  fPosXinter.clear();
                  fPosYinter.clear();
                  fPosZinter.clear();
                  fPosXBack.clear();
                  fPosYBack.clear();
                  fPosZBack.clear();
                  ATHoughSpaceCircle::FitParameters.sThetaMin        = 0;
                  ATHoughSpaceCircle::FitParameters.sThetaMin        = 0;
                  ATHoughSpaceCircle::FitParameters.sEnerMin         = 0;
                  ATHoughSpaceCircle::FitParameters.sPosMin.SetXYZ(0,0,0);
                  ATHoughSpaceCircle::FitParameters.sBrhoMin         = 0;
                  ATHoughSpaceCircle::FitParameters.sBMin            = 0;
                  ATHoughSpaceCircle::FitParameters.sPhiMin          = 0;
                  ATHoughSpaceCircle::FitParameters.sChi2Min         = 0;
                  ATHoughSpaceCircle::FitParameters.sVertexPos.SetXYZ(0,0,0);
                  ATHoughSpaceCircle::FitParameters.sVertexEner      = 0;
                  ATHoughSpaceCircle::FitParameters.sMinDistAppr     = 0;
                  ATHoughSpaceCircle::FitParameters.sNumMCPoint      = 0;
                  ATHoughSpaceCircle::FitParameters.sNormChi2        = 0;


                }
                 delete min;
                 delete parameter;

}

void ATHoughSpaceCircle::CalcHoughSpace(ATEvent* event,Bool_t YZplane,Bool_t XYplane, Bool_t XZplane){

	Int_t nHits = event->GetNumHits();
  Int_t nstep = 10;
  //Double_t drift_cal = 4.16;//mm
  Double_t drift_cal = fDriftVelocity*fTBTime/100.0;//mm


  ATMinimization *min = new ATMCMinimization();
  min->ResetParameters();

  //TH2F *RadVSTb = new TH2F("RadVSTb","RadVSTb",100,0,500,100,0,500);
  //TH1F *Phi = new TH1F("Phi","Phi",512,0,512);
  //TH1F *hdist = new TH1F("hdist","hdist",100,0,1000);
  //TH2F *distVSTb = new TH2F("DistVSTb","DistVSTb",1000,0,1000,100,0,2000);




//   #pragma omp parallel
//   {
//    #pragma omp for ordered schedule(dynamic,1)
		for(Int_t iHit=0; iHit<(nHits-nstep); iHit++){

                //std::cout<<omp_get_num_threads()<<std::endl;
          			ATHit hit = event->GetHitArray()->at(iHit);
                ATHit hit_forw = event->GetHitArray()->at(iHit+nstep);
                ATHit hit_next = event->GetHitArray()->at(iHit+1);
              	Int_t PadNumHit = hit.GetHitPadNum();
                Int_t PadNumHit_forw = hit_forw.GetHitPadNum();

          			if(hit.GetCharge()<fThreshold) continue;
             		TVector3 position = hit.GetPosition();
                TVector3 position_forw = hit_forw.GetPosition();
                TVector3 position_next = hit_next.GetPosition();

                //if(position.X()<130.0) continue;
                //if(position_forw.X()<130.0)

                Double_t hitdist = TMath::Sqrt( TMath::Power(position.X()-position_next.X(),2) + TMath::Power(position.Y()-position_next.Y(),2) + TMath::Power(position.Z()-position_next.Z(),2)   );
                //std::cout<<" Hit Coordinates - X : "<<position.X()<<" Y : "<<position.Y()<<" - Hit Coordinates Forward - X : "<<position_forw.X()<<" Y : "<<position_forw.Y()<<std::endl;
                //std::cout<<" Distance : "<<hitdist<<std::endl;
                //std::cout<<" Position Z : "<<position.Z()<<std::endl;
                //hdist->Fill(hitdist);

                //if(hitdist<4.0) continue;
                //distVSTb->Fill(hitdist,position.Z());

                //if(hitdist>5.0) continue;

          			for(Int_t itheta = 0; itheta <1023; itheta++){
          	 				Float_t angle = TMath::Pi()*(static_cast<Float_t>(itheta)/1023);
                    Float_t d0_XY_inv = 2.0* ( (position.Y()-position_forw.Y())*TMath::Sin(angle) + (position.X()-position_forw.X())*TMath::Cos(angle)  )
                    / ( (TMath::Power(position.Y(),2)-TMath::Power(position_forw.Y(),2) ) +   (TMath::Power(position.X(),2)-TMath::Power(position_forw.X(),2) )        );
                    HistHoughXY->Fill(angle,1.0/d0_XY_inv);
                }
      }//Hit loop
//    }// Parallel region


                    Int_t locmaxx,locmaxy,locmaxz;
                    HistHoughXY->GetMaximumBin(locmaxx,locmaxy,locmaxz);
                    Double_t xpos = HistHoughXY->GetXaxis()->GetBinCenter(locmaxx);
                    Double_t ypos = HistHoughXY->GetYaxis()->GetBinCenter(locmaxy);

                    fXCenter =  ypos*TMath::Cos(xpos);
                    fYCenter =  ypos*TMath::Sin(xpos);

                        //std::cout<<" yPos"<<ypos<<std::endl;
                        //std::cout<<" XCenter"<<fXCenter<<std::endl;
                        //std::cout<<" YCenter"<<fYCenter<<std::endl;
                      /// Radius, Phi, Theta, B, x0,y0,z0

                      fIniRadius=0.0;
                      fIniPhi=0.0;
                      fIniTheta=0.0;


                      for(Int_t iHit=0; iHit<nHits-1; iHit++){

                             ATHit hit = event->GetHitArray()->at(iHit);
                             TVector3 position = hit.GetPosition();
                             ATHit hit_forw = event->GetHitArray()->at(iHit+1);
                             TVector3 position_forw = hit_forw.GetPosition();

                             fRadius->push_back(TMath::Sqrt(  TMath::Power((fXCenter-position.X()),2)   +  TMath::Power((fYCenter-position.Y()),2)    ));
                             fTimeStamp->push_back(hit.GetTimeStamp());
                             fPhi->push_back(TMath::ATan2(fXCenter-position.X(),fYCenter-position.Y()));
                             Double_t dl = TMath::Sqrt( TMath::Power(position_forw.X()-position.X(),2) + TMath::Power(position_forw.Y()-position.Y(),2)  );
                             Double_t dz = (hit_forw.GetTimeStamp()-hit.GetTimeStamp())*drift_cal;
                             //fTheta->push_back(TMath::ATan2(dl,dz)); // Obsolete


                                //Second Hough Space Calculation for PhixRadius

                                   for(Int_t itheta = 0; itheta <1023; itheta++){
                                        Float_t angle = TMath::Pi()*(static_cast<Float_t>(itheta)/1023);
                                        Float_t d0_XY = (TMath::Cos(angle)*fPhi->at(iHit)*fRadius->at(iHit))  +  (TMath::Sin(angle)*hit.GetTimeStamp());
                                        HistHoughAux->Fill(angle,d0_XY);
                                        //FillHoughMap(angle,d0_XY);

                                  }

                             //Phi->SetBinContent(hit.GetTimeStamp(),fTheta->at(iHit));
                             //RadVSTb->Fill(hit.GetTimeStamp(),fRadius->at(iHit));
                             //std::cout<<" Hit Time Stamp : "<<hit.GetTimeStamp()<<std::endl;
                             //std::cout<<" Phi "<<fPhi->at(iHit)<<std::endl;
                      }

                    fHoughLinePar =  CalHoughParameters(HistHoughAux);//Histo
                    //std::pair<Double_t,Double_t> HoughLinePar = CalHoughParameters();//std
                    Double_t IniPosZ =0.0;


                    Int_t IniTSBuff=0;
                    Double_t IniRadiusBuff =0.0;
                    Double_t IniPhiBuff =0.0;


                    for(Int_t iHit=0; iHit<nHits-1; iHit++)
                    {
                      ATHit hit = event->GetHitArray()->at(iHit);
                      TVector3 position = hit.GetPosition();
                      Double_t geo_dist = TMath::Abs (TMath::Cos(fHoughLinePar.first)*fPhi->at(iHit)*fRadius->at(iHit)  + TMath::Sin(fHoughLinePar.first)*hit.GetTimeStamp()  - fHoughLinePar.second);
                      Double_t hit_dist = TMath::Sqrt( TMath::Power( (fPhi->at(iHit)*fRadius->at(iHit) - IniPhiBuff*IniRadiusBuff),2) + TMath::Power(hit.GetTimeStamp()-IniTSBuff ,2) );
                      //if(hit_dist<50) std::cout<<" Hist Dist pre :"<<hit_dist<<" Geo Dist : "<<geo_dist<<std::endl;



                              if(geo_dist<10.0){
                                        //fIniHit->SetHit(hit.GetHitPadNum(),hit.GetHitID(),position.X(),position.Y(),position.Z(),hit.GetCharge());
                                        //IniPosZ = position.Z();
                                        //fIniHit->SetTimeStamp(hit.GetTimeStamp());
                                        IniTSBuff = hit.GetTimeStamp();
                                        IniRadiusBuff = fRadius->at(iHit);
                                        IniPhiBuff = fPhi->at(iHit);

                                          if(hit_dist<10.0){


                                            fClusteredHits->push_back(&hit);
                                            fIniHit->SetHit(hit.GetHitPadNum(),hit.GetHitID(),position.X(),position.Y(),position.Z(),hit.GetCharge());
                                            IniPosZ = position.Z();
                                            fIniHit->SetTimeStamp(hit.GetTimeStamp());
                                            fIniTS = IniTSBuff;
                                            fIniRadius = IniRadiusBuff;
                                            fIniPhi = IniPhiBuff;
                                            fIniHitID = iHit; // Easier and faster to use the position of the hit in the vector
                                            //fIniHitID = hit.GetHitID();
                                            //std::cout<<" Position Z : "<<position.Z()<<std::endl;
                                            //std::cout<<" Hit : "<<iHit<<" Timebucket : "<<hit.GetTimeStamp()<<" Geometrical distance : "<<geo_dist<< " Hit Dist : "<<hit_dist<<std::endl;


                                          }




                                  }


                     }// End of clustering algorithm

                    //std::cout<<" ==========================================="<<std::endl;

                      Double_t dl_theta=0.0;
                      Double_t theta_avg=0.0;
                      Int_t clusterSize=fClusteredHits->size();
                      /*for(Int_t i=0;i<clusterSize;i++){
                         ATHit hittest = fClusteredHits->at(i);
                         std::cout<<" Clustered Hit num : "<<i<<" Time Stamp : "<<hittest.GetTimeStamp()<<std::endl;
                       }*/

                      //More precise determination of the scattering angle
                      //std::cout<<" ++++++++ Event Angle calculation ++++++++++"<<std::endl;
                      Int_t TB_Scan = 0;
                      for(Int_t iHitTB=0;iHitTB<clusterSize-1;iHitTB++){

                        Double_t posx=0.0;
                        Double_t posy=0.0;
                        Double_t posz=0.0;
                        Double_t totCharge=0.0;
                        Double_t cmsHits_X =0.0;
                        Double_t cmsHits_Y =0.0;
                        Int_t TB =0;
                        Double_t posx_forw=0.0;
                        Double_t posy_forw=0.0;
                        Double_t posz_forw=0.0;
                        Double_t totCharge_forw=0.0;
                        Double_t cmsHits_X_forw =0.0;
                        Double_t cmsHits_Y_forw =0.0;
                        Int_t TB_forw = 0;


                        // TODO: Only neighboring TB are taken into account
                        std::vector<ATHit>  fClusteredTBHits;
                        std::vector<ATHit>  fClusteredTBHits_forw;


                        fClusteredTBHits=min->GetTBHitArray(fIniTS-iHitTB-TB_Scan,fClusteredHits);

                      if(fClusteredTBHits.size()>0){ //Found at least 1 clustered hit in that time bucket, if not go to the next loop iteration
                              //std::cout<<" ================================ "<<std::endl;
                              //std::cout<<"  Starting cluster with TB : "<<fIniTS-iHitTB<<std::endl;
                              fClusteredTBHits_forw=min->GetTBHitArray(fIniTS-iHitTB-1-TB_Scan,fClusteredHits);

                       if(fClusteredTBHits_forw.size()==0){// If forward hit is empty scan until the end of the vector or until the next non-zero solution


                            while(fClusteredTBHits_forw.size()==0  &&  fIniTS-iHitTB-1-TB_Scan>0){ //TODO: Use a lambda function for this mess
                                TB_Scan++;
                                fClusteredTBHits_forw=min->GetTBHitArray(fIniTS-iHitTB-1-TB_Scan,fClusteredHits);
                                //std::cout<<TB_Scan<<std::endl;
                                //std::cout<<fClusteredTBHits_forw.size()<<std::endl;
                            }

                        }

                        if(fClusteredTBHits_forw.size()>0){




                                      if( fClusteredTBHits.size()>0  && fClusteredTBHits_forw.size()>0 ){

                                            for(Int_t iHit=0;iHit<fClusteredTBHits.size();iHit++){
                                              ATHit hitTB = fClusteredTBHits.at(iHit);
                                              TVector3 positionTB = hitTB.GetPosition();
                                              TB = hitTB.GetTimeStamp();
                                              cmsHits_X+= positionTB.X()*hitTB.GetCharge();
                                              cmsHits_Y+= positionTB.Y()*hitTB.GetCharge();
                                              totCharge+=hitTB.GetCharge();
                                              //std::cout<<" Hit TB : "<<TB<<" Position X : "<<positionTB.X()<<" Position Y : "<<positionTB.Y()<<std::endl;
                                            }

                                            for(Int_t iHit=0;iHit<fClusteredTBHits_forw.size();iHit++){
                                              ATHit hitTB_forw = fClusteredTBHits_forw.at(iHit);
                                              TVector3 positionTB_forw = hitTB_forw.GetPosition();
                                              TB_forw = hitTB_forw.GetTimeStamp();
                                              cmsHits_X_forw+= positionTB_forw.X()*hitTB_forw.GetCharge();
                                              cmsHits_Y_forw+= positionTB_forw.Y()*hitTB_forw.GetCharge();
                                              totCharge_forw+=hitTB_forw.GetCharge();
                                              //std::cout<<" Hit TB forw : "<<TB_forw<<" Position forw X : "<<positionTB_forw.X()<<" Position forw Y : "<<positionTB_forw.Y()<<std::endl;
                                            }

                                      }

                                      posx=cmsHits_X/totCharge;
                                      posy=cmsHits_Y/totCharge;
                                      posx_forw=cmsHits_X_forw/totCharge_forw;
                                      posy_forw=cmsHits_Y_forw/totCharge_forw;
                                      //std::cout<<" Average Pos X : "<<posx<<" Average Pos Y : "<<posy<<std::endl;
                                      //std::cout<<" Average Pos Forward X : "<<posx_forw<<" Average Pos Forward Y : "<<posy_forw<<std::endl;

                            }//Hit forward if>0
                          }//Hit if>0

                          Double_t distXY = TMath::Sqrt( TMath::Power(posx-posx_forw,2) + TMath::Power(posy-posy_forw,2) );
                          Double_t dl = (TB-TB_forw)*drift_cal;
                          theta_avg=TMath::ATan2(distXY,dl);
                          fTheta->push_back(theta_avg);
                          fDl->push_back(TB);
                          //std::cout<<TB<<std::endl;


                     }// Loop over clusterSize


                       // Path integration
                       /*for(Int_t iHitT=0; iHitT<clusterSize-1;iHitT++ ){
                          //std::cout<<" iHit : "<<iHitT<<" Cluster Size - Hit : "<<clusterSize-iHitT<<std::endl;

                        // ATHit hitT = event->GetHitArray()->at(nHits-iHitT);
                        // ATHit hitT_forw = event->GetHitArray()->at(nHits-1-iHitT);
                         ATHit hitT = fClusteredHits->at(clusterSize-iHitT-1);
                         ATHit hitT_forw = fClusteredHits->at(clusterSize-iHitT-2);
                         TVector3 positionT = hitT.GetPosition();
                         TVector3 positionT_forw = hitT_forw.GetPosition();


                               //if(positionT_forw.Z()-IniPosZ>=0){
                               dl_theta+= TMath::Sqrt( TMath::Power(positionT_forw.X()-positionT.X(),2) + TMath::Power(positionT_forw.Y()-positionT.Y(),2)  );
                               fDl->push_back(dl_theta);
                               fTheta->push_back( TMath::ATan2(dl_theta,(positionT_forw.Z() - IniPosZ ) ) ); // Distance in time between origin and Hit point (not HitT)
                               //std::cout<<" Hit : "<<iHitT<<" Position Forw : "<<positionT_forw.Z()-IniPosZ<<" dl : "<<dl_theta<<" Time Stamp : "<<hitT.GetTimeStamp()<<std::endl;

                              //}


                       }*/

                    //RadVSTb->Draw();
                    //Phi->Draw();
                    //HistHoughAux->Draw("zcol");

                    Double_t fThetaVal=0.0;
                    Int_t thetacnt=0; //Needed to remove theta values with 0

                    if(!fTheta->empty()){

                        for(Int_t i=0;i<fTheta->size();++i){
                            if(fTheta->at(i)>0){
                             fThetaVal+= fTheta->at(i);
                             thetacnt++;
                           }
                        }
                        fIniTheta = fThetaVal/thetacnt;
                        //std::cout<<" Theta Size : "<<fTheta->size()<<std::endl;
                        //std::cout<<" fIniTheta  : "<<fIniTheta<<" thetacnt : "<<thetacnt<<std::endl;
                        //fIniTheta = fTheta->back();
                    }


                    TVector3 IniHitPos = fIniHit->GetPosition();
                    Double_t *parameter = new Double_t[8];
                    parameter[0]=IniHitPos.X();
                    parameter[1]=IniHitPos.Y();
                    parameter[2]=IniHitPos.Z();
                    parameter[3]=fIniTS;
                    parameter[4]=fIniPhi;
                    parameter[5]=fIniRadius;
                    parameter[6]=fIniTheta;
                    parameter[7]=fIniHitID;

                    for(Int_t i=0;i<8;i++) fParameter[i]=parameter[i];


                    Double_t HoughAngleDeg = fHoughLinePar.first*180.0/TMath::Pi();

                    //std::cout<<" Hough Angle "<<HoughAngleDeg<<std::endl;

                if (   HoughAngleDeg<90.0 && HoughAngleDeg>45.0 ) { // Check RxPhi plot to adjust the angle

                   //min->MinimizeOpt(parameter,event);
                   //min->MinimizeOptMapAmp(parameter,event,);
                   fPosXmin = min->GetPosXMin();
                   fPosYmin = min->GetPosYMin();
                   fPosZmin = min->GetPosZMin();
                   fPosXexp = min->GetPosXExp();
                   fPosYexp = min->GetPosYExp();
                   fPosZexp = min->GetPosZExp();
                   fPosXinter = min->GetPosXInt();
                   fPosYinter = min->GetPosYInt();
                   fPosZinter = min->GetPosZInt();
                   fPosXBack = min->GetPosXBack();
                   fPosYBack = min->GetPosYBack();
                   fPosZBack = min->GetPosZBack();
                   ATHoughSpaceCircle::FitParameters.sThetaMin        = min->FitParameters.sThetaMin;
                   ATHoughSpaceCircle::FitParameters.sThetaMin        = min->FitParameters.sThetaMin;
                   ATHoughSpaceCircle::FitParameters.sEnerMin         = min->FitParameters.sEnerMin;
                   ATHoughSpaceCircle::FitParameters.sPosMin          = min->FitParameters.sPosMin;
                   ATHoughSpaceCircle::FitParameters.sBrhoMin         = min->FitParameters.sBrhoMin;
                   ATHoughSpaceCircle::FitParameters.sBMin            = min->FitParameters.sBMin;
                   ATHoughSpaceCircle::FitParameters.sPhiMin          = min->FitParameters.sPhiMin;
                   ATHoughSpaceCircle::FitParameters.sChi2Min         = min->FitParameters.sChi2Min;
                   ATHoughSpaceCircle::FitParameters.sVertexPos       = min->FitParameters.sVertexPos;
                   ATHoughSpaceCircle::FitParameters.sVertexEner      = min->FitParameters.sVertexEner;
                   ATHoughSpaceCircle::FitParameters.sMinDistAppr     = min->FitParameters.sMinDistAppr;
                   ATHoughSpaceCircle::FitParameters.sNumMCPoint      = min->FitParameters.sNumMCPoint;
                   ATHoughSpaceCircle::FitParameters.sNormChi2        = min->FitParameters.sNormChi2;
                  }
                  else
                  {
                    fPosXmin.clear();
                    fPosYmin.clear();
                    fPosZmin.clear();
                    fPosXexp.clear();
                    fPosYexp.clear();
                    fPosZexp.clear();
                    fPosXinter.clear();
                    fPosYinter.clear();
                    fPosZinter.clear();
                    fPosXBack.clear();
                    fPosYBack.clear();
                    fPosZBack.clear();
                    ATHoughSpaceCircle::FitParameters.sThetaMin        = 0;
                    ATHoughSpaceCircle::FitParameters.sThetaMin        = 0;
                    ATHoughSpaceCircle::FitParameters.sEnerMin         = 0;
                    ATHoughSpaceCircle::FitParameters.sPosMin.SetXYZ(0,0,0);
                    ATHoughSpaceCircle::FitParameters.sBrhoMin         = 0;
                    ATHoughSpaceCircle::FitParameters.sBMin            = 0;
                    ATHoughSpaceCircle::FitParameters.sPhiMin          = 0;
                    ATHoughSpaceCircle::FitParameters.sChi2Min         = 0;
                    ATHoughSpaceCircle::FitParameters.sVertexPos.SetXYZ(0,0,0);
                    ATHoughSpaceCircle::FitParameters.sVertexEner      = 0;
                    ATHoughSpaceCircle::FitParameters.sMinDistAppr     = 0;
                    ATHoughSpaceCircle::FitParameters.sNumMCPoint      = 0;
                    ATHoughSpaceCircle::FitParameters.sNormChi2        = 0;


                  }
                   delete min;
                   delete parameter;

                    //hdist->Draw();
                    //distVSTb->Draw("zcol");

}

void ATHoughSpaceCircle::CalcHoughSpace(ATEvent* event,TH2Poly* hPadPlane,const multiarray& PadCoord)
{

  Int_t nHits = event->GetNumHits();
  Int_t nstep = 10;
  Double_t drift_cal = fDriftVelocity*fTBTime/100.0;//mm


  ATMinimization *min = new ATMCQMinimization();
  //ATMinimization *min = new ATMCMinimization();
  min->ResetParameters();

////////////////////////////  Circular Hough Space Block /////////////////////////////
  for(Int_t iHit=0; iHit<(nHits-nstep); iHit++){

              //std::cout<<omp_get_num_threads()<<std::endl;
              ATHit hit = event->GetHitArray()->at(iHit);
              ATHit hit_forw = event->GetHitArray()->at(iHit+nstep);
              ATHit hit_next = event->GetHitArray()->at(iHit+1);
              Int_t PadNumHit = hit.GetHitPadNum();
              Int_t PadNumHit_forw = hit_forw.GetHitPadNum();

              if(hit.GetCharge()<fThreshold) continue;
              TVector3 position = hit.GetPosition();
              TVector3 position_forw = hit_forw.GetPosition();
              TVector3 position_next = hit_next.GetPosition();


              Double_t hitdist = TMath::Sqrt( TMath::Power(position.X()-position_next.X(),2) + TMath::Power(position.Y()-position_next.Y(),2) + TMath::Power(position.Z()-position_next.Z(),2)   );


              for(Int_t itheta = 0; itheta <1023; itheta++){
                  Float_t angle = TMath::Pi()*(static_cast<Float_t>(itheta)/1023);
                  Float_t d0_XY_inv = 2.0* ( (position.Y()-position_forw.Y())*TMath::Sin(angle) + (position.X()-position_forw.X())*TMath::Cos(angle)  )
                  / ( (TMath::Power(position.Y(),2)-TMath::Power(position_forw.Y(),2) ) +   (TMath::Power(position.X(),2)-TMath::Power(position_forw.X(),2) )        );
                  HistHoughXY->Fill(angle,1.0/d0_XY_inv);
              }
    }//Hit loop




                  Int_t locmaxx,locmaxy,locmaxz;
                  HistHoughXY->GetMaximumBin(locmaxx,locmaxy,locmaxz);
                  Double_t xpos = HistHoughXY->GetXaxis()->GetBinCenter(locmaxx);
                  Double_t ypos = HistHoughXY->GetYaxis()->GetBinCenter(locmaxy);

                  fXCenter =  ypos*TMath::Cos(xpos);
                  fYCenter =  ypos*TMath::Sin(xpos);

////////////////////////////  End of Circular Hough Space Block /////////////////////////////



//////////////////////////// Track calculation (Including linear Hough Space calculation)  /////////////////////////////


                    for(Int_t iHit=0; iHit<nHits-1; iHit++){

                           ATHit hit = event->GetHitArray()->at(iHit);
                           TVector3 position = hit.GetPosition();
                           ATHit hit_forw = event->GetHitArray()->at(iHit+1);
                           TVector3 position_forw = hit_forw.GetPosition();

                           fRadius->push_back(TMath::Sqrt(  TMath::Power((fXCenter-position.X()),2)   +  TMath::Power((fYCenter-position.Y()),2)    ));
                           fTimeStamp->push_back(hit.GetTimeStamp());
                           fPhi->push_back(TMath::ATan2(fXCenter-position.X(),fYCenter-position.Y()));
                           Double_t dl = TMath::Sqrt( TMath::Power(position_forw.X()-position.X(),2) + TMath::Power(position_forw.Y()-position.Y(),2)  );
                           Double_t dz = (hit_forw.GetTimeStamp()-hit.GetTimeStamp())*drift_cal;
                           //fTheta->push_back(TMath::ATan2(dl,dz)); // Obsolete


                              //Second Hough Space Calculation for PhixRadius

                              if(kHough){
                                 for(Int_t itheta = 0; itheta <1023; itheta++){
                                      Float_t angle = TMath::Pi()*(static_cast<Float_t>(itheta)/1023);
                                      Float_t d0_XY = (TMath::Cos(angle)*fPhi->at(iHit)*fRadius->at(iHit))  +  (TMath::Sin(angle)*hit.GetTimeStamp());
                                      HistHoughAux->Fill(angle,d0_XY);
                                      //FillHoughMap(angle,d0_XY);

                                }
                              }//kHough


                    }

//////////////////////////// End of Track calculation  /////////////////////////////


/////////////////////////// Hough Space Track Calculation   /////////////////////////////

                              fIniRadius=0.0;
                              fIniPhi=0.0;
                              fIniTheta=0.0;

                  if(kHough){

                            fHoughLinePar =  CalHoughParameters(HistHoughAux);//Histo
                           //std::pair<Double_t,Double_t> HoughLinePar = CalHoughParameters();//std


                            Double_t IniPosZ =0.0;
                            Int_t IniTSBuff=0;
                            Double_t IniRadiusBuff =0.0;
                            Double_t IniPhiBuff =0.0;


                            for(Int_t iHit=0; iHit<nHits-1; iHit++)
                            {
                              ATHit hit = event->GetHitArray()->at(iHit);
                              TVector3 position = hit.GetPosition();
                              Double_t geo_dist = TMath::Abs (TMath::Cos(fHoughLinePar.first)*fPhi->at(iHit)*fRadius->at(iHit)  + TMath::Sin(fHoughLinePar.first)*hit.GetTimeStamp()  - fHoughLinePar.second);
                              Double_t hit_dist = TMath::Sqrt( TMath::Power( (fPhi->at(iHit)*fRadius->at(iHit) - IniPhiBuff*IniRadiusBuff),2) + TMath::Power(hit.GetTimeStamp()-IniTSBuff ,2) );
                              //if(hit_dist<50) std::cout<<" Hist Dist pre :"<<hit_dist<<" Geo Dist : "<<geo_dist<<std::endl;
                              //std::cout<<" Hit ID : "<<hit.GetHitID()<<" Hit Time Stamp : "<< hit.GetTimeStamp()<<std::endl;


                                      if(geo_dist<10.0){

                                                IniTSBuff = hit.GetTimeStamp();
                                                IniRadiusBuff = fRadius->at(iHit);
                                                IniPhiBuff = fPhi->at(iHit);

                                                  if(hit_dist<10.0){


                                                    fClusteredHits->push_back(&hit);
                                                    fIniHit->SetHit(hit.GetHitPadNum(),hit.GetHitID(),position.X(),position.Y(),position.Z(),hit.GetCharge());
                                                    IniPosZ = position.Z();
                                                    fIniHit->SetTimeStamp(hit.GetTimeStamp());
                                                    fIniTS = IniTSBuff;
                                                    fIniRadius = IniRadiusBuff;
                                                    fIniPhi = IniPhiBuff;
                                                    fIniHitID = iHit; // Easier and faster to use the position of the hit in the vector


                                                  }




                                          }


                             }// End of clustering algorithm

                 }// kHough

/////////////////////////// End of Hough Space Track Calculation   /////////////////////////////

                 else{

/////////////////////////// RANSAC Track Calculation   /////////////////////////////

                  if(kDebug) std::cout<<cRED<<" New RANSAC Event "<<cNORMAL<<std::endl;

                   // RANSAC calculation for RxPhi:
                  // This part of the code extracts the lines found in the RxPhi space using the RANSAC algorithm of the PCL library.
                  // RANSAC returns the number of lines sorted by higher probabilty.
                  // The candidate line is found among other lines by comparing the slope, intercept and distance between them.
                  // Once the line is selected, the initial hit is chosen taking into account how many hits are found in a given TB
                  // and the density of hits along the RANSAC line.
                  // Parameters to adjust:
                  // 1)  SetRANSACPointThreshold: Number of points to process (0.1 means that the algorithm will stop when only 10% of the points are remaing)
                  // 2)  SetDistanceThreshold: Threshold distance to accept inliers
                  // 3)  _Hits: Minimum number of hits per line found
                  // 4)  n_track_cnt: Maximum number of tracks found
                  // 5)  fGoodDensity: Density of points per time bucket interval
                  // 6)  tb_range: Range of time buckets to calculate the density
                  // Inside FindCandidateTrack
                  // 7)  fSlopeLimit: Limit of the slope of the Ransac fitted track
                  // 8)  fInterceptLimit: Same as slopelimite but for the intercept: Not beign used.
                  // 9)  fStdDevTB: Number of TB that contribute to the std deviation above a value (fixed by the user) in percetage
                  // 10) fRatioLP: Ratio between the number of points and the distance between the first and last point in the line
                  // Global Parameters
                  // 11) fStdDeviationLimit: Limit for the std deviation in each time bucket. Also affects fStdDevTB.

                  fIniRadiusRansac=0.0;
                  fIniPhiRansac=0.0;
                  fIniThetaRansac=0.0;


                   ATRANSACN::ATRansac *Ransac = new ATRANSACN::ATRansac();
                   Ransac->SetModelType(pcl::SACMODEL_LINE);
                   Ransac->SetRANSACPointThreshold(0.1);
                   Ransac->SetDistanceThreshold(3.0);
                   Ransac->SetRPhiSpace();
                   Ransac->SetXYCenter(fXCenter,fYCenter);
                   Int_t n_track_cnt = 0;
                   std::vector<ATTrack*> trackSpiral = Ransac->RansacPCL(event);
                   std::vector<ATTrack*> trackSpiral_buff;
                   //std::cout<<" Found : "<<trackSpiral.size()<<" tracks "<<std::endl;
                   if(trackSpiral.size()>1){
                     for(Int_t ntrack=0;ntrack<trackSpiral.size();ntrack++){
                       std::vector<ATHit>* trackHits = trackSpiral.at(ntrack)->GetHitArray();
                       Int_t _Hits = trackHits->size();
                       //std::cout<<" Num  Hits : "<<_Hits<<std::endl;
                       //Int_t tb_range = abs(trackHits->front().GetTimeStamp()-trackHits->back().GetTimeStamp());


                      // TODO: WARNING, this part depends strongly on the experiment
                      Int_t tb_range=0;
                      auto f_TB = trackHits->front().GetTimeStamp();
                      auto b_TB = trackHits->back().GetTimeStamp();

                      // This part filters tracks with less than three time buckets with multiplicty bigger than 1
                      if(f_TB<b_TB){
                            for(Int_t tb=f_TB; tb<b_TB+1;tb++) // Checking how many Tb have more than 1 hit
                            {

                                std::vector<ATHit> hitsTB = GetTBHitArray(tb,trackHits);
                                //std::cout<<" hitsTB "<<hitsTB.size()<<" TB_hit : "<<tb<<std::endl;
                                if(hitsTB.size()>1) tb_range++;
                            }
                      }

                         //std::cout<<" tb_range : "<<tb_range<<std::endl;
                         if(_Hits>5 && n_track_cnt<5){ //Limited to 5 lines with more than 5 hits and spanning more than 2 Time Buckets
                              // Since the method returns the "stronger" lines ordered we limit the attemps to 5
                               Ransac->MinimizeTrackRPhi(trackSpiral.at(ntrack)); //Limited to 5 lines with more than 5 hits
                               std::vector<Double_t> LinePar = trackSpiral.at(ntrack)->GetFitPar();
                                  if(LinePar.size()!=0 && LinePar.at(1)<0){
                                     trackSpiral_buff.push_back(trackSpiral.at(ntrack));
                                     n_track_cnt++;
                                   } //Remove failed fits and lines with positive slope
                         }


                     }// Tracks loop (trackSpiral)

                     //std::cout<<" Remaining : "<<trackSpiral_buff.size()<<" tracks "<<std::endl;

                  if(trackSpiral_buff.size()>0){

                      ATTrack &trackCand = FindCandidateTrack(trackSpiral_buff); // Find the most probable line and then scan points in the line
                      std::vector<ATHit>* trackHits = trackCand.GetHitArray();

                      std::reverse(trackHits->begin(), trackHits->end());
                      ATHit firstHit = trackHits->front();
                      ATHit lastHit = trackHits->back();
                      Int_t index = 0;
                      Int_t lastHitMult =0;
                      Double_t x_mean = 0.0;
                      Double_t y_mean = 0.0;
                      Bool_t kGoodDensity = kFALSE;
                      Int_t fGoodDensity = 6; //Minimum density for the starting point

                      while(lastHitMult==0 && index<trackHits->size() && !kGoodDensity){
                              kGoodDensity = kFALSE;
                              //std::cout<<" first Hit "<<firstHit.GetTimeStamp()<<std::endl;
                              //std::cout<<" last Hit "<<lastHit.GetTimeStamp()<<std::endl;
                              lastHitMult = GetTBMult(trackHits->at(index).GetTimeStamp(),trackHits,index); // Get the last RANSAC hit with TB mult >1
                              if(lastHitMult){
                              ATHit hit = trackHits->at(index);
                              TVector3 position = hit.GetPosition();
                              Int_t tb_range = 4; //Range of timebuckets to calculate density of hits
                              Int_t hitdens = GetDensityOfHits(trackHits,hit.GetTimeStamp(),tb_range);
                              //std::cout<<" Hit Time Bucket : "<<hit.GetTimeStamp()<<" lastHitMult "<<lastHitMult<<" hitdens "<<hitdens<<std::endl;
                              if(hitdens>fGoodDensity){

                                  kGoodDensity = kTRUE;
                                  fIniHitRansac->SetHit(hit.GetHitPadNum(),hit.GetHitID(),position.X(),position.Y(),position.Z(),hit.GetCharge());
                                  if(kDebug) std::cout<<cGREEN<<" Ini Hit TimeStamp : "<<hit.GetTimeStamp()<<cNORMAL<<std::endl;
                                  fIniHitRansac->SetTimeStamp(hit.GetTimeStamp());
                                  fIniRadiusRansac = TMath::Sqrt(  TMath::Power((fXCenter-position.X()),2)   +  TMath::Power((fYCenter-position.Y()),2)    );
                                  fIniPhiRansac = TMath::ATan2(fXCenter-position.X(),fYCenter-position.Y());
                                  fRansacLinePar = trackCand.GetFitPar();
                                  // Moving from Hough Space to RANSAC
                                  fClusteredHits = trackCand.GetHitArray();
                                  //Shared with Hough Space
                                  fIniTS = hit.GetTimeStamp();
                                  fIniHitID = trackHits->size();

                              }else lastHitMult=0;


                              }
                              //std::cout<<" last hit with TB mult>0 "<<trackHits->at(lastHitMult).GetTimeStamp()<<std::endl;
                              //std::cout<<" Index "<<lastHitMult<<std::endl;
                              //std::cout<<" Size : "<<trackHits->size()<<std::endl;

                              index++;
                      }

                   }// Minimum tracks

                 }//track spiral size

               }// RANSAC block if !kHough

/////////////////////////// End of RANSAC Track Calculation   /////////////////////////////

/////////////////////////// Theta Average Calculation   /////////////////////////////


                    Double_t dl_theta=0.0;
                    Double_t theta_avg=0.0;
                    Int_t clusterSize=fClusteredHits->size();
                    Int_t TB_Scan = 0;
                    for(Int_t iHitTB=0;iHitTB<clusterSize-1;iHitTB++){

                      Double_t posx=0.0;
                      Double_t posy=0.0;
                      Double_t posz=0.0;
                      Double_t totCharge=0.0;
                      Double_t cmsHits_X =0.0;
                      Double_t cmsHits_Y =0.0;
                      Int_t TB =0;
                      Double_t posx_forw=0.0;
                      Double_t posy_forw=0.0;
                      Double_t posz_forw=0.0;
                      Double_t totCharge_forw=0.0;
                      Double_t cmsHits_X_forw =0.0;
                      Double_t cmsHits_Y_forw =0.0;
                      Int_t TB_forw = 0;


                      // TODO: Only neighboring TB are taken into account
                      std::vector<ATHit>  fClusteredTBHits;
                      std::vector<ATHit>  fClusteredTBHits_forw;


                      fClusteredTBHits=min->GetTBHitArray(fIniTS-iHitTB-TB_Scan,fClusteredHits);

                    if(fClusteredTBHits.size()>0){ //Found at least 1 clustered hit in that time bucket, if not go to the next loop iteration
                            //std::cout<<" ================================ "<<std::endl;
                            //std::cout<<"  Starting cluster with TB : "<<fIniTS-iHitTB<<std::endl;
                            fClusteredTBHits_forw=min->GetTBHitArray(fIniTS-iHitTB-1-TB_Scan,fClusteredHits);

                     if(fClusteredTBHits_forw.size()==0){// If forward hit is empty scan until the end of the vector or until the next non-zero solution


                          while(fClusteredTBHits_forw.size()==0  &&  fIniTS-iHitTB-1-TB_Scan>0){ //TODO: Use a lambda function for this mess
                              TB_Scan++;
                              fClusteredTBHits_forw=min->GetTBHitArray(fIniTS-iHitTB-1-TB_Scan,fClusteredHits);
                              //std::cout<<TB_Scan<<std::endl;
                              //std::cout<<fClusteredTBHits_forw.size()<<std::endl;
                          }

                      }

                      if(fClusteredTBHits_forw.size()>0){




                                    if( fClusteredTBHits.size()>0  && fClusteredTBHits_forw.size()>0 ){

                                          for(Int_t iHit=0;iHit<fClusteredTBHits.size();iHit++){
                                            ATHit hitTB = fClusteredTBHits.at(iHit);
                                            TVector3 positionTB = hitTB.GetPosition();
                                            TB = hitTB.GetTimeStamp();
                                            cmsHits_X+= positionTB.X()*hitTB.GetCharge();
                                            cmsHits_Y+= positionTB.Y()*hitTB.GetCharge();
                                            totCharge+=hitTB.GetCharge();
                                            //std::cout<<" Hit TB : "<<TB<<" Position X : "<<positionTB.X()<<" Position Y : "<<positionTB.Y()<<std::endl;
                                          }

                                          for(Int_t iHit=0;iHit<fClusteredTBHits_forw.size();iHit++){
                                            ATHit hitTB_forw = fClusteredTBHits_forw.at(iHit);
                                            TVector3 positionTB_forw = hitTB_forw.GetPosition();
                                            TB_forw = hitTB_forw.GetTimeStamp();
                                            cmsHits_X_forw+= positionTB_forw.X()*hitTB_forw.GetCharge();
                                            cmsHits_Y_forw+= positionTB_forw.Y()*hitTB_forw.GetCharge();
                                            totCharge_forw+=hitTB_forw.GetCharge();
                                            //std::cout<<" Hit TB forw : "<<TB_forw<<" Position forw X : "<<positionTB_forw.X()<<" Position forw Y : "<<positionTB_forw.Y()<<std::endl;
                                          }

                                    }

                                    posx=cmsHits_X/totCharge;
                                    posy=cmsHits_Y/totCharge;
                                    posx_forw=cmsHits_X_forw/totCharge_forw;
                                    posy_forw=cmsHits_Y_forw/totCharge_forw;
                                    //std::cout<<" Average Pos X : "<<posx<<" Average Pos Y : "<<posy<<std::endl;
                                    //std::cout<<" Average Pos Forward X : "<<posx_forw<<" Average Pos Forward Y : "<<posy_forw<<std::endl;

                          }//Hit forward if>0
                        }//Hit if>0

                        Double_t distXY = TMath::Sqrt( TMath::Power(posx-posx_forw,2) + TMath::Power(posy-posy_forw,2) );
                        Double_t dl = (TB-TB_forw)*drift_cal;
                        theta_avg=TMath::ATan2(distXY,dl);
                        fTheta->push_back(theta_avg);
                        fDl->push_back(TB);
                        //std::cout<<TB<<std::endl;


                   }// Loop over clusterSize

                   // ****************** End: RxPhi Hough Space + Clustering ********************** //



                  Double_t fThetaVal=0.0;
                  Int_t thetacnt=0; //Needed to remove theta values with 0

                  if(!fTheta->empty()){

                      for(Int_t i=0;i<fTheta->size();++i){
                          if(fTheta->at(i)>0){
                           fThetaVal+= fTheta->at(i);
                           thetacnt++;
                         }
                      }

                      // Ugly!! Debugging
                      fIniTheta = fThetaVal/thetacnt;
                      fIniThetaRansac = fThetaVal/thetacnt;
                  }

/////////////////////////// End of Theta Average Calculation   /////////////////////////////

                  Double_t *parameter = new Double_t[8];
                  TVector3 IniHitPos;

                 if(kHough){
                  IniHitPos = fIniHit->GetPosition();
                  parameter[4]=fIniPhi;
                  parameter[5]=fIniRadius;
                  parameter[6]=fIniTheta;
                 }else{
                  IniHitPos = fIniHitRansac->GetPosition();
                  parameter[4]=fIniPhiRansac;
                  parameter[5]=fIniRadiusRansac;
                  parameter[6]=fIniThetaRansac;
                }

                parameter[0]=IniHitPos.X();
                parameter[1]=IniHitPos.Y();
                parameter[2]=IniHitPos.Z();
                parameter[3]=fIniTS;
                parameter[7]=fIniHitID;


                  for(Int_t i=0;i<8;i++) fParameter[i]=parameter[i];


                  Double_t HoughAngleDeg = fHoughLinePar.first*180.0/TMath::Pi();

                  Double_t RansacAngle = 0.0;
                  if(fRansacLinePar.size()>0) RansacAngle = TMath::ATan2(fRansacLinePar.at(1),1)*180.0/TMath::Pi();

                  //std::cout<<cGREEN<<" Ransac angle : "<<RansacAngle<<" Hough Angle : "<<HoughAngleDeg<<cNORMAL<<std::endl;
                  Float_t upperlimit=0;
                  Float_t lowerlimit=0;
                  Float_t condAngle=180.0;

                  if(kHough){
                    upperlimit=90.0;
                    lowerlimit=45.0;
                    condAngle=HoughAngleDeg;
                  }else{
                    upperlimit=-45.0;
                    lowerlimit=-90.0;
                    condAngle=RansacAngle;
                  }


              //if (   RansacAngle>-90.0 && RansacAngle<-45.0 ) { // Check RxPhi plot to adjust the angle
              //if (   HoughAngleDeg<90.0 && HoughAngleDeg>45.0 ) {

              if(condAngle<upperlimit && condAngle>lowerlimit){
                 min->MinimizeOptMapAmp(parameter,event,hPadPlane,PadCoord);
                 //min->MinimizeOpt(parameter,event);
                 fPosXmin = min->GetPosXMin();
                 fPosYmin = min->GetPosYMin();
                 fPosZmin = min->GetPosZMin();
                 fPosXexp = min->GetPosXExp();
                 fPosYexp = min->GetPosYExp();
                 fPosZexp = min->GetPosZExp();
                 fPosXinter = min->GetPosXInt();
                 fPosYinter = min->GetPosYInt();
                 fPosZinter = min->GetPosZInt();
                 fPosXBack = min->GetPosXBack();
                 fPosYBack = min->GetPosYBack();
                 fPosZBack = min->GetPosZBack();
                 ATHoughSpaceCircle::FitParameters.sThetaMin        = min->FitParameters.sThetaMin;
                 ATHoughSpaceCircle::FitParameters.sThetaMin        = min->FitParameters.sThetaMin;
                 ATHoughSpaceCircle::FitParameters.sEnerMin         = min->FitParameters.sEnerMin;
                 ATHoughSpaceCircle::FitParameters.sPosMin          = min->FitParameters.sPosMin;
                 ATHoughSpaceCircle::FitParameters.sBrhoMin         = min->FitParameters.sBrhoMin;
                 ATHoughSpaceCircle::FitParameters.sBMin            = min->FitParameters.sBMin;
                 ATHoughSpaceCircle::FitParameters.sPhiMin          = min->FitParameters.sPhiMin;
                 ATHoughSpaceCircle::FitParameters.sChi2Min         = min->FitParameters.sChi2Min;
                 ATHoughSpaceCircle::FitParameters.sVertexPos       = min->FitParameters.sVertexPos;
                 ATHoughSpaceCircle::FitParameters.sVertexEner      = min->FitParameters.sVertexEner;
                 ATHoughSpaceCircle::FitParameters.sMinDistAppr     = min->FitParameters.sMinDistAppr;
                 ATHoughSpaceCircle::FitParameters.sNumMCPoint      = min->FitParameters.sNumMCPoint;
                 ATHoughSpaceCircle::FitParameters.sNormChi2        = min->FitParameters.sNormChi2;
                }
                else
                {
                  fPosXmin.clear();
                  fPosYmin.clear();
                  fPosZmin.clear();
                  fPosXexp.clear();
                  fPosYexp.clear();
                  fPosZexp.clear();
                  fPosXinter.clear();
                  fPosYinter.clear();
                  fPosZinter.clear();
                  fPosXBack.clear();
                  fPosYBack.clear();
                  fPosZBack.clear();
                  ATHoughSpaceCircle::FitParameters.sThetaMin        = 0;
                  ATHoughSpaceCircle::FitParameters.sThetaMin        = 0;
                  ATHoughSpaceCircle::FitParameters.sEnerMin         = 0;
                  ATHoughSpaceCircle::FitParameters.sPosMin.SetXYZ(0,0,0);
                  ATHoughSpaceCircle::FitParameters.sBrhoMin         = 0;
                  ATHoughSpaceCircle::FitParameters.sBMin            = 0;
                  ATHoughSpaceCircle::FitParameters.sPhiMin          = 0;
                  ATHoughSpaceCircle::FitParameters.sChi2Min         = 0;
                  ATHoughSpaceCircle::FitParameters.sVertexPos.SetXYZ(0,0,0);
                  ATHoughSpaceCircle::FitParameters.sVertexEner      = 0;
                  ATHoughSpaceCircle::FitParameters.sMinDistAppr     = 0;
                  ATHoughSpaceCircle::FitParameters.sNumMCPoint      = 0;
                  ATHoughSpaceCircle::FitParameters.sNormChi2        = 0;


                }

                 //delete Ransac;
                 delete min;
                 delete parameter;



}


std::pair<Double_t,Double_t> ATHoughSpaceCircle::CalHoughParameters(TH2F* hist){

		std::pair<Double_t,Double_t> HoughParBuff;
		Int_t locmaxx,locmaxy,locmaxz;
    hist->GetMaximumBin(locmaxx,locmaxy,locmaxz);
    Double_t xpos = hist->GetXaxis()->GetBinCenter(locmaxx);
    Double_t ypos = hist->GetYaxis()->GetBinCenter(locmaxy);
		//std::cout<<" X Hough Position : "<<180-xpos*180/TMath::Pi()<<std::endl;
    //std::cout<<" X Hough Position : "<<xpos<<std::endl;
	  //std::cout<<" Y Hough Position : "<<ypos<<std::endl;
		HoughParBuff.first= xpos;
		HoughParBuff.second= ypos;
		return HoughParBuff;

}

void ATHoughSpaceCircle::FillHoughMap(Double_t ang, Double_t dist)
{


      ULong64_t MapKey;

      //if(ang<0 || dist<0)  std::cout<<" -I- ATHougSpaceLine : Negative values for Hough Space parameters "<<std::endl;

      if(ang>0 || dist>0){
      Int_t dist_enc = static_cast<Int_t>(round(dist*1.5));
      Int_t ang_enc = static_cast<Int_t>(round(ang*160.0));
      /*std::cout<<" Filling std::map for the Hough Space "<<std::endl;
      std::cout<<" Hough Angle : "<<ang<<" Hough Dist : "<<dist<<std::endl;
      std::cout<<" Encoded Hough Angle : "<<ang_enc<<" Encoded Hough Dist : "<<dist_enc<<std::endl;*/

      MapKey = dist_enc&0xFFFFFFFF;
      MapKey = (MapKey<<32) + (ang_enc&0xFFFFFFFF);
      HoughMap[MapKey]++;

    /*  std::cout<<" Map Key : "<<MapKey<<std::endl;
      std::cout<<" Map Key Dist "<<(MapKey&0xFFFFFFFF)<<std::endl;
      std::cout<<" Map Key Ang "<<((MapKey>>32)&(0xFFFFFFFF))<<std::endl;*/

      }


}

std::pair<Double_t,Double_t> ATHoughSpaceCircle::CalHoughParameters()
{

    std::pair<Double_t,Double_t> HoughParBuff;
    // Method 1:
    /*  using pair_type = decltype(HoughMap)::value_type;

       auto pr = std::max_element
        (
            std::begin(HoughMap), std::end(HoughMap),
            [] (const pair_type & p1, const pair_type & p2) {
              return p1.second < p2.second;
            }
         );

          std::cout << "pr->first : " << pr->first<< '\n';
          std::cout << "pr->second : " << pr->second<< '\n';
          std::cout << " pr->first&0xFFFFFFFF " << (pr->first&0xFFFFFFFF) << '\n';
          std::cout << " ((pr->first>>32)&0xFFFFFFFF) " << ((pr->first>>32)&0xFFFFFFFF) << '\n';*/


     //Method 2:
           std::map<ULong64_t,Int_t>::iterator maximum = std::max_element(HoughMap.begin(), HoughMap.end(), maxpersecond());
           HoughParBuff.first=(maximum->first&0xFFFFFFFF)/160.0; //Binning of the associated histogram
           HoughParBuff.second=((maximum->first&0xFFFFFFFF00000000)>>32)/1.5;
          // std::cout<<" Size of First Element in Hough Map (byte) :"<<sizeof(maximum->first)<<std::endl;
          // std::cout<<" Size of Second Element of Houg Map (byte) :"<<sizeof(maximum->second)<<std::endl;
          // std::cout<<" Angle : "<<(maximum->first&0xFFFFFFFF)/160.0<<std::endl;
          // std::cout<<" Distance : "<<((maximum->first&0xFFFFFFFF00000000)>>32)/1.5<<std::endl;
          // std::cout<<" Number of Maximum Votes : "<<maximum->second<<std::endl;
          return HoughParBuff;

}

void ATHoughSpaceCircle::CalcHoughSpace(ATProtoEvent* protoevent,Bool_t q1,Bool_t q2, Bool_t q3, Bool_t q4){



}

Int_t ATHoughSpaceCircle::GetTBMult(Int_t TB,std::vector<ATHit> *harray,Int_t index)
{

    auto it =  std::find_if( harray->begin()+index,harray->end(),[&TB](ATHit& hit){return hit.GetTimeStamp()==TB;}   );
    if(it != harray->end()){
       auto hitInd = std::distance<std::vector<ATHit>::const_iterator>(harray->begin(),it);
       return hitInd;
    }
    else{
      return 0;
    }

}

ATTrack& ATHoughSpaceCircle::FindCandidateTrack(const std::vector<ATTrack*>& tracks)
{

  // This function compares the candidate track lines in the RxPhi space
  // The algorithm tries to get the line with the larger time bucket within a predifined value of the slope and intercept
  // TODO: Slope and intercept limits should be externally implemented by visual inspection of the events


  // ************ Adjustable parameters ***********//
  Float_t fSlopeLimit = -25.0;
  Float_t fInterceptLimit = 200.0;
  Float_t fStdDevTB = 0.20;
  Float_t fRatioLP = 5.0;
  //////////////////////////////////////////////////

  Int_t index=0;
  Int_t max_TB = 0;

  if(tracks.size()>1){

    ATTrack* track_b = tracks.at(0);
    std::vector<Double_t> LinePar_b = track_b->GetFitPar();
    std::vector<ATHit>* trackHits_b = track_b->GetHitArray();
    Double_t chi2_b = track_b->GetMinimum();
    Int_t ndf_b  = track_b->GetNFree();

    ATHit hit_b_f = trackHits_b->front();
    ATHit hit_b_b = trackHits_b->back();

    Double_t x_mean_b = 0.0;
    Double_t y_mean_b = 0.0;

    Float_t ratio_maxdev_b=0;


    //GetDeviation(trackHits_b,x_mean_b,y_mean_b);
     GetTBDeviation(trackHits_b,ratio_maxdev_b,x_mean_b,y_mean_b);


    for(Int_t i=1;i<tracks.size();i++)
    {

                Float_t ratio_maxdev_f=0;
                ATTrack* track_f = tracks.at(i);
                std::vector<Double_t> LinePar_f = track_f->GetFitPar();
                std::vector<ATHit>* trackHits_f = track_f->GetHitArray();
                Double_t chi2_f = track_f->GetMinimum();
                Int_t ndf_f  = track_f->GetNFree();

                Double_t x_mean_f = 0.0;
                Double_t y_mean_f = 0.0;

                //GetDeviation(trackHits_f,x_mean_f,y_mean_f);
                 GetTBDeviation(trackHits_f,ratio_maxdev_f,x_mean_f,y_mean_f);

                ATHit hit_f_f = trackHits_f->front();
                ATHit hit_f_b = trackHits_f->back();

                TVector3 pos_b_f = hit_b_f.GetPosition();
                TVector3 pos_b_b = hit_b_b.GetPosition();
                TVector3 pos_f_f = hit_f_f.GetPosition();
                TVector3 pos_f_b = hit_f_b.GetPosition();

                //Debugging
                if(kDebug){
                std::cout<<" ====================================================== "<<std::endl;
                std::cout<<LinePar_b.at(0)<<"  "<<LinePar_b.at(1)<<std::endl;
                std::cout<<LinePar_f.at(0)<<"  "<<LinePar_f.at(1)<<std::endl;
                }

                // Distance in the RxPhi space
                Double_t xdum_b = hit_b_b.GetTimeStamp();
                Double_t ydum_b = TMath::Sqrt(  TMath::Power((fXCenter-pos_b_b.X()),2)   +  TMath::Power((fYCenter-pos_b_b.Y()),2)    )*TMath::ATan2(fXCenter-pos_b_b.X(),fYCenter-pos_b_b.Y());
                Double_t xdum_f = hit_f_b.GetTimeStamp();
                Double_t ydum_f = TMath::Sqrt(  TMath::Power((fXCenter-pos_f_b.X()),2)   +  TMath::Power((fYCenter-pos_f_b.Y()),2)    )*TMath::ATan2(fXCenter-pos_f_b.X(),fYCenter-pos_f_b.Y());


                Double_t dist_x = xdum_b - xdum_f;
                Double_t dist_y = ydum_b - ydum_f;
                Double_t dist_tot = TMath::Sqrt( TMath::Power(dist_x,2) + TMath::Power(dist_y,2) );

                Double_t ratio_lp_b = abs((hit_b_f.GetTimeStamp()-hit_b_b.GetTimeStamp())/(float)trackHits_b->size() );
                Double_t ratio_lp_f = abs((hit_f_f.GetTimeStamp()-hit_f_b.GetTimeStamp())/(float)trackHits_f->size()  );


                // Track hits are sorted in descending order, i.e. first hit is largest one in TB
                if(kDebug){
                std::cout<<" First hit of the first line : "<<hit_b_b.GetTimeStamp()<<" Number of hits : "<<trackHits_b->size()<<std::endl;
                std::cout<<" First hit of the second line : "<<hit_f_b.GetTimeStamp()<<" Number of hits : "<<trackHits_f->size()<<std::endl;
                std::cout<<" Last hit of the first line : "<<hit_b_f.GetTimeStamp()<<" Number of hits : "<<trackHits_b->size()<<std::endl;
                std::cout<<" Last hit of the second line : "<<hit_f_f.GetTimeStamp()<<" Number of hits : "<<trackHits_f->size()<<std::endl;
                std::cout<<" First line reduced Chi2 : "<<chi2_b/ndf_b<<"    Second line reduced Chi2 : "<<chi2_f/ndf_f<<std::endl;
                std::cout<<" First line std dev x : "<<x_mean_b<<" First line std dev y : "<<y_mean_b<<std::endl;
                std::cout<<" Second line std dev x : "<<x_mean_f<<" Second line std dev y : "<<y_mean_f<<std::endl;
                std::cout<<" Distance between first hit of the lines : "<<dist_tot<<std::endl;
                std::cout<<" Distance X between first hit of the lines : "<<dist_x<<std::endl;
                std::cout<<" Distance Y between first hit of the lines : "<<dist_y<<std::endl;
                std::cout<<" Length of the first line in TB : "<<hit_b_f.GetTimeStamp()-hit_b_b.GetTimeStamp()<<std::endl;
                std::cout<<" Length of the second line in TB : "<<hit_f_f.GetTimeStamp()-hit_f_b.GetTimeStamp()<<std::endl;
                std::cout<<" Ratio length/points First line : "<<ratio_lp_b<<std::endl;
                std::cout<<" Ratio length/points Second line : "<<ratio_lp_f<<std::endl;
                std::cout<<" Index i : "<<i<<" Index i+1 : "<<i+1<<std::endl;
                std::cout<<" Max TB : "<<max_TB<<std::endl;
                }


                  // Get the line on the right by comparing the last time bucket of each one and the slope. A maximum TB is defined to keep the line with the latest
                  // point when comparing every  track just once.
                  // Then the standard deviation of the points for each time bucket, the ratio between the number of points and length
                  // and the number of time buckets with an unsual large std deviation are evaluated
                  if((hit_b_b.GetTimeStamp()>hit_f_b.GetTimeStamp()) && (LinePar_b.at(1)>fSlopeLimit) && (hit_b_b.GetTimeStamp()>max_TB) ){
                     if(x_mean_b<fStdDeviationLimit && y_mean_b<fStdDeviationLimit)//Mean std deviation upper limit
                     {
                       if(ratio_maxdev_b<fStdDevTB){ //The number of TB that contribute to the std dev limit (in percentage)
                         if(ratio_lp_b<fRatioLP && ratio_lp_b>0){ //Ratio between the length in TB and the number of points
                                index = 0;
                                max_TB = hit_b_b.GetTimeStamp();
                         }
                       }
                     }
                  }else if ((hit_b_b.GetTimeStamp()<hit_f_b.GetTimeStamp()) && (LinePar_f.at(1)>fSlopeLimit)  && (hit_f_b.GetTimeStamp()>max_TB) ){
                    if(x_mean_f<fStdDeviationLimit && y_mean_f<fStdDeviationLimit)//Mean std deviation upper limit
                    {
                       if(ratio_maxdev_f<fStdDevTB){ //The number of TB that contribute to the std dev limit (in percentage)
                         if(ratio_lp_f<fRatioLP && ratio_lp_f>0){ //Ratio between the length in TB and the number of points
                               index = i;
                               max_TB = hit_f_b.GetTimeStamp();
                        }
                       }
                     }
                  }

                  if(kDebug) std::cout<<" Index inside : "<<index<<std::endl;


    }

    if(kDebug) std::cout<<" Index : "<<index<<std::endl;
    return *tracks.at(index);

}else return *tracks.at(0);


}

void ATHoughSpaceCircle::GetDeviation(std::vector<ATHit>* hits,Double_t& _x_dev,Double_t& _y_dev)
{

    Double_t mean_x=0;
    Double_t mean_y=0;
    Double_t sigma = 5.00; //Standard deviation due to the pad size in mm

      if(hits->size()>1){
          for(auto i=0;i<hits->size();i++)
          {
            TVector3 position = hits->at(i).GetPosition();
            mean_x+=position.X();
            mean_y+=position.Y();
          }

          mean_x/=hits->size();
          mean_y/=hits->size();

          for(auto i=0;i<hits->size();i++)
          {
            TVector3 position = hits->at(i).GetPosition();
            _x_dev+= TMath::Power(mean_x - position.X(),2)/TMath::Power(sigma,2);
            _y_dev+= TMath::Power(mean_y - position.Y(),2)/TMath::Power(sigma,2);
          }

           _x_dev/=hits->size();
           _y_dev/=hits->size();


      }


}

void ATHoughSpaceCircle::GetTBDeviation(std::vector<ATHit>* hits,Float_t& maxdev_ratio,Double_t& mean_dev_x,Double_t& mean_dev_y)
{

    Double_t sigma = 5.00; //Standard deviation due to the pad size in mm
    mean_dev_x = 0;
    mean_dev_y = 0;
    Int_t n_tb=0;

  if(hits->size()>1){ // At least two hits
    ATHit hit_f = hits->front();
    ATHit hit_b = hits->back();
    auto f_index = hit_f.GetTimeStamp();
    auto b_index = hit_b.GetTimeStamp();

    if(f_index<b_index){ //TODO: The problem comes when all the points have the same TB
          for(auto i=f_index;i<=b_index;i++)
          {
              std::vector<ATHit> hitArray = GetTBHitArray(i,hits); //Find all hit with same TB

              Double_t mean_x=0;
              Double_t mean_y=0;
              Double_t _x_dev=0;
              Double_t _y_dev=0;

                if(hitArray.size()>1){ // The deviation is only calculated for TB with more than 1 hit
                        for(auto j=0;j<hitArray.size();j++)
                        {
                            ATHit hitTB=hitArray.at(j);
                            TVector3 position = hitTB.GetPosition();
                            mean_x+=position.X();
                            mean_y+=position.Y();
                        }//for TB array

                            mean_x/=hitArray.size();
                            mean_y/=hitArray.size();

                        for(auto k=0;k<hitArray.size();k++)
                        {
                            TVector3 position = hitArray.at(k).GetPosition();
                              _x_dev+= TMath::Power(mean_x - position.X(),2)/TMath::Power(sigma,2);
                              _y_dev+= TMath::Power(mean_y - position.Y(),2)/TMath::Power(sigma,2);
                        }

                           _x_dev/=hitArray.size();
                           _y_dev/=hitArray.size();

                           if(_x_dev>fStdDeviationLimit || _y_dev>fStdDeviationLimit) maxdev_ratio++; //Number of time buckets with  std dev larger than a threshold

                           n_tb++;// valid TB
               }//hitArray>0

                            mean_dev_x+=_x_dev;
                            mean_dev_y+=_y_dev;



             if(kDebug) std::cout<<cRED<<" Hits : "<<hitArray.size()<<" TB : "<<i<<" _x_dev : "<<_x_dev<<" _y_dev : "<<_y_dev<<cNORMAL<<std::endl;


          }//for TB

            mean_dev_x/=n_tb;
            mean_dev_y/=n_tb;
            maxdev_ratio/=n_tb;



      }//if

  }

  if(kDebug) std::cout<<cYELLOW<<" Mean X dev : "<<mean_dev_x<<" Mean Y dev : "<<mean_dev_y<<" Ratio of TB with more than 5 "<<maxdev_ratio<<cNORMAL<<std::endl;

}


Int_t ATHoughSpaceCircle::GetDensityOfHits(std::vector<ATHit>* hits,Int_t index, Int_t tb_range)
{
  // This function assumes a reversed hit containers (descending TB value from beginning to end)
  int cnt = std::count_if(hits->begin(),hits->end(),[&index,&tb_range](ATHit& hit){return hit.GetTimeStamp()>index-tb_range;});
  return cnt;

}

std::vector<ATHit> ATHoughSpaceCircle::GetTBHitArray(Int_t TB,std::vector<ATHit> *harray)
{

        std::vector<ATHit> hitTBArray;
        std::copy_if(harray->begin(), harray->end(), std::back_inserter(hitTBArray),[&TB](ATHit& hit){return hit.GetTimeStamp()==TB;} );
        return hitTBArray;
}
