#include "MCMinimization.hh"
#include <iostream>
#include <algorithm>
#include "TRandom.h"
#include "TH1F.h"
#ifdef _OPENMP
#include <omp.h>
#endif


#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(MCMinimization)


MCMinimization::MCMinimization()
{
  fThetaMin=0.0;
  fEnerMin=0.0;
  fBrhoMin=0.0;
  fBMin=0.0;
  fPhiMin=0.0;
  fDensMin=0.0;
  fVertexEner=0.0;

  fThetaLorentz    = -6.6*TMath::Pi()/180.0;
  fThetaRot        = -13.0*TMath::Pi()/180.0;
  fThetaTilt       = 7.4*TMath::Pi()/180.0;
  fThetaPad        = 113.7*TMath::Pi()/180.0;
  fEntTB           = 280;
  fZk              = 1000.0;

}

MCMinimization::~MCMinimization()
{

}


void MCMinimization::ResetParameters()
{
  FitParameters.sThetaMin    = 0;
  FitParameters.sEnerMin     = 0;
  FitParameters.sPosMin.SetXYZ(0,0,0);
  FitParameters.sBrhoMin     = 0;
  FitParameters.sBMin        = 0;
  FitParameters.sPhiMin      = 0;
  FitParameters.sChi2Min     = 0;
  FitParameters.sVertexPos.SetXYZ(0,0,0);
  FitParameters.sVertexEner  = 0;
  FitParameters.sMinDistAppr = 0;
  FitParameters.sNumMCPoint  = 0;
  FitParameters.sNormChi2    = 0;

}

Bool_t MCMinimization::MinimizeOpt(Double_t* parameter,ATEvent *event){

                  std::vector<Double_t> xc;
                  std::vector<Double_t> xiter;
                  std::vector<Double_t> xinter;
                  std::vector<Double_t> yc;
                  std::vector<Double_t> yiter;
                  std::vector<Double_t> yinter;
                  std::vector<Double_t> zc;
                  std::vector<Double_t> ziter;
                  std::vector<Double_t> zinter;
                  std::vector<Int_t> TBInter;

                 Double_t xcmm[10000]={0};
                 Double_t ycmm[10000]={0};
                 Double_t zcmm[10000]={0};

                 Double_t xsol[10000]={0};
                 Double_t ysol[10000]={0};
                 Double_t zsol[10000]={0};

                 Double_t xdet[10000]={0};
                 Double_t ydet[10000]={0};
                 Double_t zdet[10000]={0};

                 Double_t xpad[10000]={0};
                 Double_t ypad[10000]={0};
                 Double_t zpad[10000]={0};

                 Double_t xTBCorr[10000];
                 Double_t yTBCorr[10000];
                 Double_t zTBCorr[10000];

                 std::fill_n(xTBCorr,-10000, -10000);
                 std::fill_n(yTBCorr,-10000, -10000);
                 std::fill_n(zTBCorr,-10000, -10000);

                 m                  = 1;
                 sm1                = m;
                 dzstep             = 5.20*80.0/1000.0;//  !unit cm
                 integrationsteps   = 10;
                 iz1                = 1;
                 z1                 = iz1;
                 restmass           = sm1*931.49432;
                 esm                = z1*1.75879e-3*0.510998918/restmass;// ![e/m electron cm**2/(Volt*nsec**2] this is not the energy/mass but charge/mass
                 B0                 = 1.66;  // !	magnetic field
                 B                  = B0*10000.; // !conversion of T en Gauss

                 Double_t ymax            = 0.0;
                 Double_t e0sm            = 0.0;
                 Double_t smprot          = 931.49432;
                 Double_t Bmin            = B;
                 Double_t chi2min         = 1E10;


                 /////////// Initial parameters///////////////
                 Double_t xmin= parameter[0]/10.0; //  ! at ztb=394 in cm
                 Double_t ymin= parameter[1]/10.0;
                 Double_t zmin= fZk/10.0 - (fEntTB-parameter[3])*dzstep;  //Micromegas Origin  at 100 cm of the entrance
                 Double_t TBmin = parameter[3]*dzstep;
                 Double_t phi0=TMath::Pi()-parameter[4]-115*TMath::Pi()/180.0;

                 Double_t bro=parameter[5]*B0/1000.0;// !Tm*/
                 Double_t theta0=parameter[6];
                 ////////////////////////////////////////////

                 Double_t phimin=phi0;
                 Double_t thetamin=theta0;
                 Double_t bromin=bro;
                 Double_t brotheta=bromin/TMath::Sin(thetamin);
                 Double_t tm=0.0;
                 Int_t iteration=0;

                 kVerbose = kFALSE;
                 kDebug   = kFALSE;

                 //if(kVerbose){
                 std::cout<<std::endl;
                 std::cout<<cGREEN<<" ============================"<<std::endl;
                 std::cout<<" Starting Monte Carlo event  "<<std::endl;
                 std::cout<<" X : "<<xmin<<" cm  - Y : "<<ymin<<" cm - Z : "<<zmin<<" cm "<<std::endl;
                 std::cout<<" Brho : "<<bro<<" Tm "<<std::endl;
                 std::cout<<" Radius of curvature : "<<parameter[5]<<" mm "<<std::endl;
                 std::cout<<" Scattering Angle : "<<theta0*180.0/TMath::Pi()<<" deg "<<std::endl;
                 std::cout<<" Azimutal Angle : "<<phi0*180.0/TMath::Pi()<<" deg "<<std::endl;
                 std::cout<<" Lenght of the experimental data : "<<parameter[7]<<cNORMAL<<std::endl;
              //  }

                std::vector<std::vector<ATHit>> *hitTBMatrix = new std::vector<std::vector<ATHit>>;

                std::vector<ATHit> *fHitArray = event->GetHitArray();
                Int_t numIntPoints=0;
                for(Int_t iexp=0;iexp<parameter[3];iexp++){
                std::vector<ATHit> hitTBArray;
                hitTBArray=GetTBHitArray(parameter[3]-iexp,fHitArray);
                if(hitTBArray.size()>0) numIntPoints++;
                hitTBMatrix->push_back(hitTBArray); // TB descends
                }

                GetEnergy(sm1,z1,brotheta,e0sm);
                if(kVerbose) std::cout<<cGREEN<<" Energy of the proton : "<<e0sm<<" MeV "<<cNORMAL<<std::endl;
                Double_t chimininit=1.e6;

                if(bro==0 || isnan(e0sm) || e0sm>100.0){
                      if(kVerbose) std::cout<<cRED<<" Invalid energy !"<<cNORMAL<<std::endl;
                    return kFALSE;
                  }

                  if(parameter[7]<700){


                        for(Int_t i=0;i<5;i++)
                        {


                            Float_t factstep=1.0/(TMath::Power(1.4,i));
                            Float_t step1=2*factstep;// !theta in deg
                            Float_t step2=2*factstep;//  !phi in deg
                            Float_t step3=0.2*factstep;//! broradius in realtive vaue
                            Float_t step4=0.3*factstep;//!x0 in cm
                            Float_t step5=0.3*factstep;// !y0
                            Float_t step6=0.5*factstep;//!z0
                            Float_t step7=0.0*factstep;//B
                            Float_t step8=0.0*factstep;//Density


                            for(Int_t j=0;j<400;j++) //MC
                            {
                              bro=bromin*(1.+(0.5-gRandom->Rndm())*step3); //!in Tm
                              theta0=thetamin+step1*(0.5-gRandom->Rndm())*0.01745;
                              brotheta=bro/TMath::Sin(theta0);//  !initial bro corrected for angle
                              //            phi0= (-20.+ step2*(0.5-rand(0)))*0.01745 +180.
                              phi0= phimin+ step2*(0.5-gRandom->Rndm())*0.01745;
                              B=Bmin*(1.+step7*(0.5-gRandom->Rndm()));




                              GetEnergy(sm1,z1,brotheta,e0sm);

                              // 	calcul vitesse initiale
                              Double_t e0ll=e0sm*sm1;
                              Double_t e0=e0ll*1000000.;// !conversion from MeV in eV kinetic energy
                              //	esm= 1.75879e-3
                              Double_t beta2=2.*e0ll/(sm1*931.49);
                              Double_t ekin=e0ll;
                              Double_t beta0=sqrt(beta2);//    ![cm/nsec]

                               //      recalcuate velocity for energy loss
                               //  	vsc=sqrt(dxdt**2+dydt**2+dzdt**2)/29.9792  !v/c

                               Double_t ecinsm=ekin/sm1;
                               Double_t eloss=ekin;
                              //	dedx=s*dens  !de/dx in [MeV/cm] !only defined after 1st tour
                                //          dzstep=0.221  !unit cm
                              //	boucle d'integration
                              Int_t  ipr=0;
                              Double_t  range=0.0;

                              //        define initial conditions

                              //         Transform initial parameters into lab
                              Double_t  x = xmin + step4*(gRandom->Rndm()-0.5); // ! ztb=394
                              Double_t  y = ymin + step5*(gRandom->Rndm()-0.5);
                              Double_t  z = zmin + step6*(gRandom->Rndm()-0.5);
                              Double_t x_buff =x;
                              Double_t y_buff =y;
                              Double_t z_buff =z;

                                        //std::cout<<" Xini "<<x<<" Yini "<<y<<" Zini "<<z<<std::endl;

                                        TVector3 PosIniCmm = TransformIniPos(x,y,z); //Arguments in cm
                                        x=PosIniCmm.X();
                                        y=PosIniCmm.Y();
                                        z=PosIniCmm.Z();
                                        Double_t zmin_trans = z;

                                        //For testing purposes!!!
                                        /*TVector3 InvPosIni = InvTransIniPos(x,y,z); //Arguments in cm
                                        x=InvPosIni.X();
                                        y=InvPosIni.Y();
                                        z=InvPosIni.Z();
                                        std::cout<<" Xt "<<x<<" Yt "<<y<<" Zt "<<z<<std::endl;*/
                                       /////////////////////////////////////////////////

                                        Double_t  x0=x;
                                        Double_t  y0=y;
                                        Double_t  z0=z;

                                        Double_t v0=beta0*29.9792;//  !v in cm/ns
                                        Double_t dt=dzstep/(v0*TMath::Cos(theta0));//![unite temps ns]
                                        dt=dt/(Float_t)integrationsteps; // NEW
                                        Double_t t=-dt;

                                        //           initial velocity vector
                                        //	v0=sqrt(v2)    ![cm/nsec]
                                                    Double_t dxdt=v0*TMath::Sin(theta0)*TMath::Cos(phi0);
                                                    Double_t dydt=v0*TMath::Sin(theta0)*TMath::Sin(phi0);
                                                    Double_t dzdt=v0*TMath::Cos(theta0);
                                                    Double_t iprinttr=1000;
                                                    Double_t dens=0.06363*18*(1+step8*(gRandom->Rndm()-0.5))/20.;//  !DENSITY ISOBUTANE AT 20 TORR corrected 18 torr
                                                    Double_t iterationmax=10000;
                                                    ipr=0;

                                        //******************************************************************

                                                     iteration=0;
                                                     Int_t iterd=0;
                                                     Int_t iterCorr=0;
                                                     Int_t iterCorr_0=0; //offset correction for TB
                                                     Int_t iterd0=0;
                                                     Int_t iterCorrNorm=0;

                                                     Int_t icnb;
                                                     Int_t num_MC_Point = 0;


                                                     for(Int_t k=0;k<iterationmax;k++)
                                                     {
                                                           iteration++;
                                                           Float_t factq=1.0;
                                                           //iterd=(Int_t) k/integrationsteps;//NEW
                                                           iterd=k;


                                                           xcmm[iterd] = x*10.0;
                                                           ycmm[iterd] = y*10.0;
                                                           zcmm[iterd] = z*10.0;

                                                            zcmm[iterd] = -zcmm[iterd]+ 2*zmin_trans*10.0;
                                                            xsol[iterd]=xcmm[iterd]-zcmm[iterd]*TMath::Sin(fThetaLorentz)*TMath::Sin(fThetaRot);
                                                            ysol[iterd]=ycmm[iterd]+zcmm[iterd]*TMath::Sin(fThetaLorentz)*TMath::Cos(fThetaRot);
                                                            zsol[iterd]=zcmm[iterd];

                                                            xdet[iterd] = xsol[iterd];
                                                            ydet[iterd] = -(fZk-zsol[iterd])*TMath::Sin(fThetaTilt) + ysol[iterd]*TMath::Cos(fThetaTilt);
                                                            zdet[iterd] = zsol[iterd]*TMath::Cos(fThetaTilt) - ysol[iterd]*TMath::Sin(fThetaTilt);

                                                            xpad[iterd] = xdet[iterd]*TMath::Cos(fThetaPad) - ydet[iterd]*TMath::Sin(fThetaPad);
                                                            ypad[iterd] = xdet[iterd]*TMath::Sin(fThetaPad) + ydet[iterd]*TMath::Cos(fThetaPad);
                                                            zpad[iterd] = zdet[iterd];




                                                           iterCorr = (Int_t) (zpad[iterd]/(dzstep*10) + 0.5);
                                                           if(k==0) iterCorr_0 = iterCorr; //Offset renomarlization
                                                           //std::cout<<" iterCorr : "<<iterCorr_0-iterCorr<<" iterd : "<<iterd<<std::endl;

                                                           iterCorrNorm = iterCorr_0-iterCorr;
                                                           if(iterCorrNorm<0) break;

                                                           xTBCorr[iterCorrNorm] = xpad[iterd];
                                                           yTBCorr[iterCorrNorm] = ypad[iterd];
                                                           zTBCorr[iterCorrNorm] = zpad[iterd];


                                                        //  if(iterCorrNorm!=icnb){
                                                           xiter.push_back(xTBCorr[iterCorrNorm]);
                                                           yiter.push_back(yTBCorr[iterCorrNorm]);
                                                           ziter.push_back(zTBCorr[iterCorrNorm]);
                                                      //    }
                                                          icnb=iterCorrNorm;



                                                           t=t+dt;
                                                           Double_t ddxddt=esm*B*10.*dydt*factq;//  !remember esm =charge/masse
                                                           Double_t ddyddt=-esm*(B*10.*dxdt)*factq;
                                                           Double_t ddzddt=0.;
                                                           x=x + dxdt*dt + 0.5*ddxddt*TMath::Power(dt,2);
                                                           y=y + dydt*dt + 0.5*ddyddt*TMath::Power(dt,2);
                                                           z=z + dzdt*dt + 0.5*ddzddt*TMath::Power(dt,2);
                                                           dxdt=dxdt+ddxddt*dt;
                                                           dydt=dydt+ddyddt*dt;
                                                           dzdt=dzdt+ddzddt*dt;
                                                           //std::cout<<" Iteration : "<<k<<" vx : "<<dxdt<<" vy : "<<dydt<<" vz : "<<dzdt<<std::endl;


                                                           Double_t help=TMath::Power((dxdt*dt + 0.5*ddxddt*TMath::Power(dt,2)),2 );
                                                           help=help+TMath::Power((dydt*dt + 0.5*ddyddt*TMath::Power(dt,2)),2);
                                                           help=help+TMath::Power((dzdt*dt + 0.5*ddzddt*TMath::Power(dt,2)),2);
                                                           help=TMath::Sqrt(help);
                                                           range=range+help;
                                                           //std::cout<<" Range : "<<range<<std::endl;
                                                           Double_t sloss = 0.0;

                                                           Double_t  c0;

                                                           if(iz1==1){
                                                              c0=ekin;
                                                              if(m==2) c0=c0/2.0;
                                                              sloss=6.98*(1./TMath::Power(c0,0.83))*(1./(20.+1.6/TMath::Power(c0,1.3)))+0.2*TMath::Exp(-30.*TMath::Power((c0-0.1),2)); //Old expression
                                                              //sloss = 0.3*TMath::Power((1./c0),0.78)*(1./(1.+0.023/TMath::Power(c0,1.37)));
                                                           }
                                                           if(iz1==6){
                                                              c0=ekin/6.;
                                                              sloss=36.*(1./TMath::Power(c0,0.83))*(1./(1.6+1.6/TMath::Power(c0,1.5)))+1.*TMath::Exp(TMath::Power(-(c0-0.5),2));
                                                           }
                                                           if(iz1==2){
                                                              c0=ekin;
                                                              sloss=11.95*(1./TMath::Power(c0,0.83))*(1./(2.5+1.6/TMath::Power(c0,1.5)))+ 0.05*TMath::Exp(TMath::Power(-(c0-0.5),2));
                                                            }

                                                            sloss= sloss*dens*help; //!energy loss with density and step

                                                            //if(y>ymax)ymax=y;
                                                            //slowing down by energy loss
                                                            Double_t vcin=sqrt(TMath::Power(dxdt,2)+TMath::Power(dydt,2)+TMath::Power(dzdt,2));// !v in cm/ns
                                                            Double_t vsc=vcin/29.979; // !v/c
                                                            Double_t beta=vsc;
                                                            //ecinsm=931.494/2.*(vsc)**2*1./sqrt(1.-vsc**2) !relativistic
                                                            Double_t ekindo=sm1*931.494*0.5*TMath::Power(vsc,2); //!nonrelativistic

                                                                help=ekin;
                                                                ekin=ekin-sloss;// !energy loss
                                                                //help=ekin/help
                                                                help=ekin/ekindo;
                                                                help=TMath::Sqrt(help);
                                                                Double_t help1=help;
                                                                ekindo=ekindo-sloss;
                                                                dxdt=dxdt*help; // ! this is the introduction energy loss!!!!
                                                                dydt=dydt*help;
                                                                dzdt=dzdt*help;
                                                                //dt=dzstep/(dzdt);
                                                                dt=dzstep/(dzdt)/(Float_t) integrationsteps;//NEW
                                                                //std::cout<<" z :"<<z<<" dt : "<<dt<<" dzstep : "<<dzstep<<" dzdt : "<<dzdt<<std::endl;
                                                                //test if still in detector supposing radius of 25cm and lenght 100cm
                                                                Double_t radp2=TMath::Power(x,2)+TMath::Power(y,2);
                                                                Double_t radp=TMath::Sqrt(radp2);
                                                                //std::cout<<cRED<<" dt : "<<dt<<cNORMAL<<std::endl;

                                                              //  if(radp>25.0) break;//  !this limits the radial distance of the trajectories taken into ccount
                                                                //	if(z.gt.ztot) go to 100
                                                                if(zTBCorr[iterCorrNorm]<0.0) break;
                                                                //std::cout<<" Ekin : "<<ekin<<std::endl;
                                                                if(ekin<0.01 || isnan(ekin)) break;
                                                                //if(ekin<0) std::cout<<" Ekin "<<std::endl;


                                                    }//k loop Spiral Integration

                                                    tm=tm+t;

                                                    //simangle->Draw("AP");



                                                    //CHI SQUARE CALCULATION
                                                    Int_t iterh = (Int_t)(iteration/integrationsteps);

                                                    Int_t imaxchi2=std::max(iterCorrNorm,(Int_t) parameter[7]); //NEW
                                                    if(kDebug){
                                                       std::cout<<cRED<<" Imaxchi2 : "<<imaxchi2<<std::endl;
                                                       std::cout<<" iterCorrNorm : "<<iterCorrNorm<<std::endl;
                                                       std::cout<<" iterh : "<<iterh<<std::endl;
                                                       std::cout<<" parameter[7] : "<<parameter[7]<<std::endl;
                                                       std::cout<<" Num of interpolated exp points : "<<numIntPoints<<cNORMAL<<std::endl;
                                                    }
                                                    //Int_t imaxchi2=std::max(iteration,(Int_t)parameter[7]);
                                                    Double_t sigma2 = 36.0;  //!error in mm2
                                                    Double_t chi2   = 0.0;
                                                    Bool_t kIsExp   = kTRUE;
                                                    Bool_t kIsSim   = kTRUE;

                                                      //NB : imaxchi2 must be limited

                                                  Double_t xposbuff[imaxchi2];
                                                  Double_t yposbuff[imaxchi2];
                                                  Double_t zposbuff[imaxchi2];
                                                  Double_t xTBbuff[imaxchi2];
                                                  Double_t yTBbuff[imaxchi2];
                                                  Double_t zTBbuff[imaxchi2];
                                                  Double_t chi2buff[imaxchi2];
                                                  Int_t TBShadow[imaxchi2];

                                                  std::fill_n(chi2buff,0,0);


                                                   for(Int_t iChi=0;iChi<imaxchi2;iChi++){

                                                      std::vector<ATHit> hitTBArray;
                                                      //hitTBArray=GetTBHitArray(parameter[3]-iChi,fHitArray); // Seach for Hits with the same TB
                                                      if(hitTBMatrix->size()>iChi) hitTBArray=hitTBMatrix->at(iChi);

                                                      //if(iChi<parameter[7])   hitTBArray=GetTBHitArray(parameter[3]-iChi,fHitArray); // Seach for Hits with the same TB
                                                     //if(event->GetEventID()==973) std::cout<<" iChi : "<<iChi<<" parameters 3 "<<parameter[3]<<" parameters 7 "<<parameter[7]<<" Hit TB Size "<<hitTBArray.size()<<std::endl;

                                          //NB: Simulated data progress in forward direction in the sd::vector while the experimental one is
                                          // the opposite
                                          //                                                  Initial Time Bucket
                                          //                                                          |
                                          //                                                       Simulation -> Increasing time bucket (std::vector position)
                                          //                           Increasing time bucket  <-  Experiment
                                          //Vector index: 0,1,2,3 ....

                                                          // TO compare Sim and Exp uncomment this

                                                        if(kDebug){
                                                        std::cout<<cRED<<"  ------------------------------------------------------------------------------------------- "<<cNORMAL<<std::endl;
                                                        std::cout<<" Array : "<<iChi<<" X_Sim : "<<xcmm[iChi]<<" Y_Sim : "<<ycmm[iChi]<<" Z_Sim : "<<zcmm[iChi]<<" TB Sim"<<floor(zcmm[iChi]/dzstep/10)<<std::endl;
                                                        std::cout<<cGREEN<<" Array Corr: "<<iChi<<" X_Sim_Corr : "<<xTBCorr[iChi]<<" Y_Sim_Corr : "<<yTBCorr[iChi]<<" Z_Sim_Corr : "<<zTBCorr[iChi]<<cNORMAL<<std::endl;
                                                        std::cout<<" Real Time bucket : "<<parameter[3]+iChi<<std::endl;
                                                        //std::cout<<" Corrected TIme Bucket : "<<(zcmm[iChi]/10 - 100)/dzstep + fEntTB <<std::endl;
                                                        //std::cout<<iChi<<" X_exp : "<<position.X()<<" Y_exp : "<<position.Y()<<" Hit TS : "<<parameter[3]-HitTS<<std::endl;
                                                        std::cout<<" Number of hits with same TB : "<<hitTBArray.size()<<std::endl;
                                                        std::cout<<" Experimental TB "<<parameter[3]-iChi<<std::endl;
                                                        }

                                                        if(xTBCorr[iChi]==(Double_t)-10000) kIsSim=kFALSE;
                                                        else kIsSim=kTRUE;


                    //std::cout<<" Experimental Time Bucket : "<<parameter[3]-iChi<<"  Simulated Time Bucket : "<<round( (zcmm[iChi]- fZk/10)/dzstep + parameter[3] )<<std::endl;


                                                        Double_t posx=0.0;
                                                        Double_t posy=0.0;
                                                        Double_t posz=0.0;
                                                        Int_t numHitsDist = 0;
                                                        Int_t TB = 0;
                                                        Double_t cmsHits_X =0.0;
                                                        Double_t cmsHits_Y =0.0;
                                                        Double_t totCharge=0.0;


                                                           if(hitTBArray.size()>0){

                                                             /*ATHit hit = hitTBArray.at(0);
                                                             TVector3 positiontest = hit.GetPosition();
                                                             Int_t HitTS = hit.GetTimeStamp();
                                                             std::cout<<" Position test Z : "<<positiontest.Z()<<std::endl;
                                                             std::cout<<" Hit Time Stamp HitTS : "<<HitTS<<std::endl;*/


                                                              for(Int_t iHitTB=0;iHitTB<hitTBArray.size();iHitTB++){// TODO: There no need to do this every chisquare loop
                                                                ATHit hitTB = hitTBArray.at(iHitTB);
                                                                TVector3 positionTB = hitTB.GetPosition();
                                                                //posz=positionTB.Z();


                                                                //Double_t hitDist = TMath::Sqrt( TMath::Power(positionTB.X()-xcmm[iChi],2) + TMath::Power(positionTB.Y()-ycmm[iChi],2) );
                                                                cmsHits_X+= positionTB.X()*hitTB.GetCharge();
                                                                cmsHits_Y+= positionTB.Y()*hitTB.GetCharge();
                                                                totCharge+=hitTB.GetCharge();
                                                                //dist_vs_TB->Fill(parameter[3]-iChi,hitDist);

                                                                  posx+=positionTB.X();
                                                                  posy+=positionTB.Y();
                                                                  TB=hitTB.GetTimeStamp();
                                                                  //Recalibration of Z position
                                                                  posz = fZk - (fEntTB-TB)*dzstep*10;  // Calibration of Z in mm
                                                                  //numHitsDist++;

                                                                // TO compare Sim and Exp uncomment this
                                                                if(kDebug) std::cout<<iChi<<" X_exp : "<<positionTB.X()<<" Y_exp : "<<positionTB.Y()<<" Z_exp : "<<posz<<" Hit TS : "<<hitTB.GetTimeStamp()<<std::endl;

                                                              }// Loop over hits with same TB

                                                              posx/=hitTBArray.size();
                                                              posy/=hitTBArray.size();

                                                              posx=cmsHits_X/totCharge;
                                                              posy=cmsHits_Y/totCharge;

                                                              kIsExp=kTRUE;
                                                                // TO compare Sim and Exp uncomment this
                                                              if(kDebug) std::cout<<cGREEN<<" Average X : "<<posx<<" Average Y : "<<posy<<" Average Z : "<<posz<<cNORMAL<<std::endl;

                                                           }else if(hitTBArray.size()==0){
                                                               posx=0.0;
                                                               posy=0.0;
                                                               kIsExp=kFALSE;
                                                           }// Size of container


                                                          xinter.push_back(posx);
                                                          yinter.push_back(posy);
                                                          zinter.push_back(posz);
                                                          TBInter.push_back(TB);

                                                          Double_t diffx= posx-xTBCorr[iChi];
                                                          Double_t diffy= posy-yTBCorr[iChi];

                                                          xposbuff[iChi]=posx;
                                                          yposbuff[iChi]=posy;
                                                          zposbuff[iChi]=posz;
                                                          xTBbuff[iChi]=xTBCorr[iChi];
                                                          yTBbuff[iChi]=yTBCorr[iChi];
                                                          zTBbuff[iChi]=zTBCorr[iChi];
                                                          TBShadow[iChi]=TB;


                                                          if(kIsExp){
                                                            Double_t chi2_buff = ( TMath::Power(diffx,2) + TMath::Power(diffy,2) )/sigma2;
                                                            if(i>2 && chi2_buff>10.0) chi2+=10.0;
                                                            else if(i<3 && chi2_buff>100.0) chi2+=100.0;
                                                            else chi2+=chi2_buff;
                                                            chi2buff[iChi]=chi2_buff;
                                                            if(kDebug) std::cout<<" Point chi Square : "<<chi2_buff<<" for Experimental Time bucket : "<<parameter[3]-iChi<<std::endl;
                                                            if(kDebug) std::cout<<" Total chi Square : "<<chi2<<std::endl;
                                                            fPosXinter=xinter;
                                                            fPosYinter=yinter;
                                                            fPosZinter=zinter;
                                                            fPosTBinter=TBInter;
                                                            num_MC_Point++;

                                                          }

                                                          if(kDebug && !kIsSim) std::cout<<cRED<<" Point without simulated value! "<<cNORMAL<<std::endl;
                                                          if(kDebug && !kIsExp) std::cout<<cRED<<" Point without experimental value! "<<cNORMAL<<std::endl;
                                                          if(kDebug) std::cout<<cRED<<"  ------------------------------------------------------------------------------------------- "<<cNORMAL<<std::endl;



                                                    }// Chi2 loop
                                                //  }// Temporary condition


                                                  if(chi2<chi2min){
                                                     chi2min = chi2;
                                                     fPosXmin=xiter;
                                                     fPosYmin=yiter;
                                                     fPosZmin=ziter;
                                                     fThetaMin=theta0;
                                                     fEnerMin=e0sm;
                                                     fPosMin.SetXYZ(x_buff,y_buff,z_buff);
                                                     fBrhoMin=bro;
                                                     fBMin=B;
                                                     fPhiMin=phi0;
                                                     fDensMin=dens;

                                                     bromin=bro; //Minimization variables
                                                     thetamin=theta0;
                                                     Bmin=B;
                                                     phimin=phi0;

                                                     FitParameters.sThetaMin = theta0;
                                                     FitParameters.sEnerMin=e0sm;
                                                     FitParameters.sPosMin.SetXYZ(x,y,z);
                                                     FitParameters.sBrhoMin=bro;
                                                     FitParameters.sBMin=B;
                                                     FitParameters.sPhiMin=phi0;
                                                     FitParameters.sChi2Min=chi2min;
                                                     FitParameters.sNumMCPoint=num_MC_Point;
                                                     FitParameters.sNormChi2=chi2min/num_MC_Point;

                                                   if(kVerbose){
                                                     std::cout<<cYELLOW<<" New Min chi2 : "<<chi2min<<" for MC iteration : "<<j<<" of the step "<<i<<std::endl;
                                                     std::cout<<" Number of MC points : "<<num_MC_Point<<std::endl;
                                                     std::cout<<" Reduced chi2 : "<<chi2min/num_MC_Point<<cNORMAL<<std::endl;
                                                     std::cout<<" Energy : "<<e0sm<<" Angle "<<theta0<<std::endl;
                                                   }




                                                   }

                                                  // Reset MC container for the next iteration
                                                  memset(xcmm, 0, sizeof(xcmm));
                                                  memset(ycmm, 0, sizeof(ycmm));
                                                  memset(zcmm, 0, sizeof(zcmm));
                                                  xinter.clear();
                                                  yinter.clear();
                                                  zinter.clear();
                                                  TBInter.clear();
                                                  xiter.clear();
                                                  yiter.clear();
                                                  ziter.clear();


                            }//j Loop MC Parameter Variation


                        } // i Loop MC Step redefinition

                    }// paramter[7] cut


                                                            for(Int_t ig=0;ig<fHitArray->size();ig++){
                                                                 ATHit hit = fHitArray->at(ig);
                                                                 TVector3 position = hit.GetPosition();
                                                                 fPosXexp.push_back(position.X());
                                                                 fPosYexp.push_back(position.Y());
                                                                 fPosTBexp.push_back(hit.GetTimeStamp());
                                                           }

                                                           BackwardExtrapolation();

                                                          // if(kVerbose){
                                                              std::cout<<cYELLOW<<" Minimization result : "<<std::endl;
                                                              std::cout<<" Scattering Angle : "<<fThetaMin*180.0/TMath::Pi()<<std::endl;
                                                              std::cout<<" Azimutal angle : "<<fPhiMin*180.0/TMath::Pi()<<std::endl;
                                                              std::cout<<" B : "<<fBMin<<std::endl;
                                                              std::cout<<" Brho : "<<fBrhoMin<<std::endl;
                                                              std::cout<<" Energy : "<<fEnerMin<<std::endl;
                                                              std::cout<<" Vertex Position (Backward extrapolation) - X : "<<fVertexPos.X()<<" - Y : "<<fVertexPos.Y()<<" - Z : "<<fVertexPos.Z()<<std::endl;
                                                              std::cout<<" Vertex Energy : "<<fVertexEner<<" MeV "<<std::endl;
                                                              std::cout<<" Reduced chi2 : "<<chi2min/FitParameters.sNumMCPoint<<std::endl;
                                                              std::cout<<" Minimum chi2 : "<<chi2min<<cNORMAL<<std::endl;
                                                        //  }


                                                            delete hitTBMatrix;
                                                            return kTRUE;




}

std::vector<ATHit> MCMinimization::GetTBHitArray(Int_t TB,std::vector<ATHit> *harray)
{

        std::vector<ATHit> hitTBArray;
        std::copy_if(harray->begin(), harray->end(), std::back_inserter(hitTBArray),[&TB](ATHit& hit){return hit.GetTimeStamp()==TB;} );
        return hitTBArray;
}

void MCMinimization::GetEnergy(Double_t M,Double_t IZ,Double_t BRO,Double_t &E){

  Float_t  AM=931.5;
  Float_t X=BRO/0.1439*IZ/M;
  X=pow(X,2);
  X=2.*AM*X;
  X=X+pow(AM,2);
  E=TMath::Sqrt(X)-AM;

}

TVector3 MCMinimization::TransformIniPos(Double_t x,Double_t y, Double_t z)
{

   TVector3 PosIniCmm;

   Double_t x_det = x*TMath::Cos(fThetaPad)  + y*TMath::Sin(fThetaPad);
   Double_t y_det = -x*TMath::Sin(fThetaPad) + y*TMath::Cos(fThetaPad);
   Double_t z_det = z;

   Double_t x_sol = x_det;
   Double_t z_sol = ( z_det*TMath::Cos(fThetaTilt) + y_det*TMath::Sin(fThetaTilt) + fZk/10.0*TMath::Power(TMath::Sin(fThetaTilt),2) ) ;
   Double_t y_sol = ( y_det + (fZk/10.0-z_sol)*TMath::Sin(fThetaTilt) )/ TMath::Cos(fThetaTilt);

   Double_t z_cmm = z_sol;
   Double_t x_cmm = x_sol + z_cmm*TMath::Sin(fThetaLorentz)*TMath::Sin(fThetaRot);
   Double_t y_cmm = y_sol - z_cmm*TMath::Sin(fThetaLorentz)*TMath::Cos(fThetaRot);

   PosIniCmm.SetXYZ(x_cmm,y_cmm,z_cmm);

  return PosIniCmm;


}

TVector3 MCMinimization::InvTransIniPos(Double_t x,Double_t y, Double_t z)
{

  TVector3 InvPosIni;

  Double_t xsol=x-z*TMath::Sin(fThetaLorentz)*TMath::Sin(fThetaRot);
  Double_t ysol=y+z*TMath::Sin(fThetaLorentz)*TMath::Cos(fThetaRot);
  Double_t zsol=z;

  Double_t xdet = xsol;
  Double_t ydet = -(fZk/10.0-zsol)*TMath::Sin(fThetaTilt) + ysol*TMath::Cos(fThetaTilt);
  Double_t zdet = zsol*TMath::Cos(fThetaTilt) - ysol*TMath::Sin(fThetaTilt);

  Double_t xpad = xdet*TMath::Cos(fThetaPad) - ydet*TMath::Sin(fThetaPad);
  Double_t ypad = xdet*TMath::Sin(fThetaPad) + ydet*TMath::Cos(fThetaPad);
  Double_t zpad = zdet;

  InvPosIni.SetXYZ(xpad,ypad,zpad);

  return InvPosIni;

}

void MCMinimization::BackwardExtrapolation()
{

              Double_t xcmm;
              Double_t ycmm;
              Double_t zcmm;
              Double_t xsol;
              Double_t ysol;
              Double_t zsol;
              Double_t xdet;
              Double_t ydet;
              Double_t zdet;
              Double_t xpad;
              Double_t ypad;
              Double_t zpad;

              Double_t x_origin = 0.0;  //Vertex origin, by default we assume the center of the micromegas.
              Double_t y_origin = 0.0;

              Double_t xTBCorr[200];
              Double_t yTBCorr[200];
              Double_t zTBCorr[200];

              Double_t minDist = 1E10; //Distance of minimum approach to the vertex origin

              Double_t e0ll=fEnerMin*sm1;
              Double_t beta2=2.*e0ll/(sm1*931.49);
  	          Double_t ekin=e0ll;
  	          Double_t beta0=sqrt(beta2);//    ![cm/nsec]
              Double_t v0=beta0*29.9792;//  !v in cm/ns
              Double_t dt=-dzstep/(v0*TMath::Cos(fThetaMin))/(Float_t)(integrationsteps);//![unite temps ns] negatif time
              Double_t t=0.;
              Double_t x= fPosMin.X();
              Double_t y= fPosMin.Y();
              Double_t z= fPosMin.Z();
              TVector3 PosIniCmm = TransformIniPos(x,y,z); // Transform initial position from pad plane to laboratory frame
              x=PosIniCmm.X();
              y=PosIniCmm.Y();
              z=PosIniCmm.Z();
              Double_t zmin_trans = z;
              t=-dt;
              Double_t dxdt=v0*TMath::Sin(fThetaMin)*TMath::Cos(fPhiMin);
              Double_t dydt=v0*TMath::Sin(fThetaMin)*TMath::Sin(fPhiMin);
              Double_t dzdt=v0*TMath::Cos(fThetaMin);
              Double_t factq=1.0;

              Double_t ddxddt = 0.0;
              Double_t ddyddt = 0.0;
              Double_t ddzddt = 0.;
              Int_t iterCorr = 0;
              Int_t iterCorr_0 = 0;
              Int_t iterCorrNorm = 0;

              Double_t z_vertex=0.0;

              Double_t dist;

                for(Int_t i=0;i<200;i++){

                  dist = TMath::Sqrt( TMath::Power(x - x_origin,2) + TMath::Power(y - y_origin,2)  );

                  //std::cout<<" X back : "<<x<<" Y back "<<y<<" Z back "<<z<<" zmin_trans : "<<zmin_trans<<std::endl;
                  //std::cout<<" Z Vertex : "<<z_vertex<<std::endl;
                  z_vertex = 2*zmin_trans - z;

                  if(dist<minDist) minDist=dist;
                  else{
                    fVertexPos.SetXYZ(x,y,z_vertex);
                    fVertexEner=ekin;
                    FitParameters.sVertexPos=fVertexPos;
                    FitParameters.sVertexEner=fVertexEner;
                    FitParameters.sMinDistAppr=dist;
                    break;
                  }


                  //Draw the extrapolated spiral in the micromegas pad plane
                  xcmm = x*10.0;
                  ycmm = y*10.0;
                  zcmm = z*10.0;
                  zcmm = -zcmm+ 2*zmin_trans*10.0;
                  //std::cout<<" zcmm : "<<zcmm[iterd]<<"  2*zmin*10.0  : "<<2*zmin_trans*10.0<<std::endl;
                  xsol = xcmm-zcmm*TMath::Sin(fThetaLorentz)*TMath::Sin(fThetaRot);
                  ysol = ycmm+zcmm*TMath::Sin(fThetaLorentz)*TMath::Cos(fThetaRot);
                  zsol = zcmm;

                  xdet = xsol;
                  ydet = -(fZk-zsol)*TMath::Sin(fThetaTilt) + ysol*TMath::Cos(fThetaTilt);
                  zdet = zsol*TMath::Cos(fThetaTilt) - ysol*TMath::Sin(fThetaTilt);

                  xpad = xdet*TMath::Cos(fThetaPad) - ydet*TMath::Sin(fThetaPad);
                  ypad = xdet*TMath::Sin(fThetaPad) + ydet*TMath::Cos(fThetaPad);
                  zpad = zdet;

                  iterCorr = (Int_t) (zpad/(dzstep*10) + 0.5);
                  if(i==0) iterCorr_0 = iterCorr; //Offset renomalization
                  iterCorrNorm = -iterCorr_0+iterCorr;


                  if(iterCorrNorm>=0){
                  xTBCorr[iterCorrNorm] = xpad;
                  yTBCorr[iterCorrNorm] = ypad;
                  zTBCorr[iterCorrNorm] = zpad;

                  fPosXBack.push_back(xTBCorr[iterCorrNorm]);
                  fPosYBack.push_back(yTBCorr[iterCorrNorm]);
                  fPosZBack.push_back(zTBCorr[iterCorrNorm]);
                  }

                  /////////////////////////////////////////////////////////

                  t=t+dt;
                  ddxddt=esm*B*10.*dydt*factq;//  !remember esm =charge/masse
                  ddyddt=-esm*(B*10.*dxdt)*factq;
                  ddzddt=0.;
                  x=x + dxdt*dt + 0.5*ddxddt*TMath::Power(dt,2);
                  y=y + dydt*dt + 0.5*ddyddt*TMath::Power(dt,2);
                  z=z + dzdt*dt + 0.5*ddzddt*TMath::Power(dt,2);
                  dxdt=dxdt+ddxddt*dt;
                  dydt=dydt+ddyddt*dt;
                  dzdt=dzdt+ddzddt*dt;

                  Double_t help=TMath::Power((dxdt*dt + 0.5*ddxddt*TMath::Power(dt,2)),2 );
                  help=help+TMath::Power((dydt*dt + 0.5*ddyddt*TMath::Power(dt,2)),2);
                  help=help+TMath::Power((dzdt*dt + 0.5*ddzddt*TMath::Power(dt,2)),2);
                  help=TMath::Sqrt(help);

                  Double_t sloss = 0.0;

                  Double_t  c0;


                  if(iz1==1){
                     c0=ekin;
                     if(m==2) c0=c0/2.0;
                     sloss=6.98*(1./TMath::Power(c0,0.83))*(1./(20.+1.6/TMath::Power(c0,1.3)))+0.2*TMath::Exp(-30.*TMath::Power((c0-0.1),2)); //Old expression
                     //sloss = 0.3*TMath::Power((1./c0),0.78)*(1./(1.+0.023/TMath::Power(c0,1.37)));
                  }
                  if(iz1==6){
                     c0=ekin/6.;
                     sloss=36.*(1./TMath::Power(c0,0.83))*(1./(1.6+1.6/TMath::Power(c0,1.5)))+1.*TMath::Exp(TMath::Power(-(c0-0.5),2));
                  }
                  if(iz1==2){
                     c0=ekin;
                     sloss=11.95*(1./TMath::Power(c0,0.83))*(1./(2.5+1.6/TMath::Power(c0,1.5)))+ 0.05*TMath::Exp(TMath::Power(-(c0-0.5),2));
                   }

                   sloss= sloss*fDensMin*help; //!energy loss with density and step

                   //if(y>ymax)ymax=y;
                   //slowing down by energy loss
                   Double_t vcin=sqrt(TMath::Power(dxdt,2)+TMath::Power(dydt,2)+TMath::Power(dzdt,2));// !v in cm/ns
                   Double_t vsc=vcin/29.979; // !v/c
                   Double_t beta=vsc;
                   //ecinsm=931.494/2.*(vsc)**2*1./sqrt(1.-vsc**2) !relativistic
                   Double_t ekindo=sm1*931.494*0.5*TMath::Power(vsc,2); //!nonrelativistic

                       help=ekin;
                       ekin=ekin+sloss;// !energy loss
                       //help=ekin/help
                       help=ekin/ekindo;
                       help=TMath::Sqrt(help);
                       Double_t help1=help;
                       ekindo=ekindo-sloss;
                       dxdt=dxdt*help; // ! this is the introduction energy loss!!!!
                       dydt=dydt*help;
                       dzdt=dzdt*help;
                       //dt=dzstep/(dzdt);
                       dt=-dzstep/(dzdt)/(Float_t) integrationsteps;

                       if(i==199)
                          if(kVerbose) std::cout<<cYELLOW<<" Warning: Minimum distance of approach not found ! "<<cNORMAL<<std::endl;



                }




}
