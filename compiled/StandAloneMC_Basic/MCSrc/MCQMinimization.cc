#include "MCQMinimization.hh"
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

ClassImp(MCQMinimization)

MCQMinimization::MCQMinimization()
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

MCQMinimization::~MCQMinimization()
{

}

Int_t MCQMinimization::GetMinimization()
{

}

void MCQMinimization::ResetParameters()
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


Bool_t MCQMinimization::MinimizeOptMapAmp(Double_t* parameter,ATEvent *event, TH2Poly* hPadPlane,multiarray PadCoord)
{


           double Qtrack[10000]={0.}; //simulated amplitude for track to analyse
           double ztrackq[10000]={0.}; //simulated amplitude for track to analyse *ztrack fo find center of gravity

            fPadPlane = hPadPlane;
            fHitArray = event->GetHitArray();
            for(Int_t i=0;i<fHitArray->size();i++){
               ATHit hit = fHitArray->at(i);
               TVector3 position = hit.GetPosition();
               Int_t hitTB = hit.GetTimeStamp();
               Int_t hitPadNum = hit.GetHitPadNum();
               Int_t hitcharge = hit.GetCharge();
               //std::cout<<cGREEN<<" Hit number : "<<i<<" X : "<<position.X()<<" Y : "<<position.Y()<<" Z : "<<position.Z()<<" TB : "<<hitTB<<" Pad Number : "<<hitPadNum<<" charge "<<hitcharge<<cNORMAL<<std::endl;
               double xhelp = position.X();
               double yhelp = position.Y();
               Int_t iplot= 10*sqrt(hitcharge/500.);
               int ipl;
               if (iplot>10)iplot=10;
               //for (ipl=0;ipl<(iplot);++ipl){
              //   fout1<<"  "<<yhelp;
              // }
              // fout1<< std::endl;
               Qtrack[hitPadNum] = hitcharge;
               ztrackq[hitPadNum] = position.Z() ; // values to be used in chi2
             }


             //This is the experimental hitpattern

              //std::fout1<<" Hit number : "<<i<<" X : "<<position.X()<<" Y : "<<position.Y()<<" Z : "<<position.Z()<<" TB : "<<hitTB<<" Pad Number : "<<hitPadNum<<" charge "<<hitcharge<<cNORMAL<<std::endl;

             /*fout1<<" ind " <<" xind "<<" yind "<<"  Qsim "<<" zcg 1 2 3 4 5 6 7 8 9 10"<<endl;
           for (size_t i = 0; i < npadtotal; i++) {
               //fout1<<"  "<<zsimq[i]<<endl;
               if(Qsim[i] > tracksimthreshold)
               {
                 //int  ihelp=fPadPlane->GetBinCenter(i);
                 double xhelp=PadCoord[i][0][0];
                 double yhelp=PadCoord[i][0][1];
                 ivalidsim++  ;
                  zsimq[i]  = zsimq[i]/Qsim[i];//normalized position
                  fout1<<"  "<<i<<"  "<< xhelp<<"  "<<yhelp<<"  "<<Qsim[i];


                     Int_t iplot= 10*sqrt((Qsim[i])/500.);
                     int ipl;
                     if (iplot>10)iplot=10;
                     for (ipl=0;ipl<(iplot);++ipl){
                       fout1<<"  "<<yhelp;
                     }
                     fout1<<"  "<<zsimq[i]<<endl;
               }// normalized*/

            //std::cout<<" Number of hits in the event : "<<HitArray->size()<<std::endl;

            //TODO: Pass these paramters with the fPar pointer
            Double_t ymax            = 0.0;
            Double_t e0sm            = 0.0;
            Double_t smprot          = 931.49432;
            Double_t Bmin            = B;
            Double_t chi2min         = 1E10;
            //////////////////////////////////////////
            /////////// Initial parameters///////////////
          /*  Double_t xmin= parameter[0]/10.0; //  ! at ztb=394 in cm
            Double_t ymin= parameter[1]/10.0;
            Double_t zmin= fZk/10.0 - (fEntTB-parameter[3])*dzstep;  //Micromegas Origin  at 100 cm of the entrance
            //Double_t zmin= parameter[3]*dzstep;
            Double_t TBmin = parameter[3]*dzstep; // Absolute TB to compare between exp and sim
            //Double_t phi0= (TMath::Pi()-5.0*TMath::Pi())-parameter[4]- fThetaPad; //RADIANS
            //Double_t phi0=parameter[4]+fThetaPad-10.0*TMath::Pi(); //RADIANS
            Double_t phi0=TMath::Pi()-parameter[4]-115*TMath::Pi()/180.0;

            //Double_t phi0=parameter[4];

            Double_t bro=parameter[5]*B0/1000.0;// !Tm
             Double_t theta0=parameter[6];
             */



            /////////// Initial parameters///////////////
            Double_t xmin;
            Double_t ymin;
            Double_t zmin;  //Micromegas Origin  at 1000 mm of the entrance
            Double_t TBmin; // Absolute TB to compare between exp and sim
            Double_t phi0;
            Double_t bro;// !Tm*/
            Double_t theta0;
            ////////////////////////////////////////////
            double x0MC  = xmin ; //mm
            double y0MC  = ymin;
            double z0MC  = zmin;
            double aMC   = theta0; //radians
            double phiMC = phi0;
            //double Bmin; //magnetic field in Tesla
            double dens = 0.0738 ; // value from Josh 0.06363*18./20. ; //gas density
            double romin;
            double Bminv;//B //after MC variation
            double densv  ; //gas density after MC variation
            double rominv; //magnetic radius after MC variation
            double xpad[10000]={0}; //these are the partial integral results in small steps
            double ypad[10000]={0};
            double zpad[10000]={0};
            double Qpad[10000]={0}; //wm
            double Qsim[10000]={0.}; //simulated amplitude
            double zsimq[10000]={0.};
            double Qtracktotal;
            double QMCtotal;
            double CHi2fit;
            int modevar=1;
            double rangeMC= 200.;  //simulation of track
            int iconvar;
            double x0MCv= x0MC ;
            double y0MCv =y0MC ;
            double z0MCv = z0MC;
            double aMCv = aMC;
            double phiMCv = phiMC;
            double rangeMCv = rangeMC ;  //simulation of track values for MC variation
            double sigmaq=0.2 ;  //defined as the fraction of sum Qsim+Qtrack
            double sigmaz= 4.0  ;  //defined as deviation of the center of gravity in mm modified from 5.4 on june 10 wm
            int imc1=0;
            int imc1max=10;
            iconvar=imc1;
            int imc2=0;
            int imc2max=100;
            int icontrol=1;


            MCvar(parameter, icontrol,iconvar,x0MC, y0MC, z0MC, aMC,phiMC, Bmin, dens,romin,x0MCv, y0MCv,z0MCv, aMCv, phiMCv, Bminv, densv, rominv); // for initialisation

                std::cout<<std::endl;
                std::cout<<cGREEN<<" ============================"<<std::endl;
                std::cout<<" Starting Monte Carlo event  "<<std::endl;
                std::cout<<" X : "<<x0MC<<" cm  - Y : "<<y0MC<<" cm - Z : "<<z0MC<<" cm "<<std::endl;
                std::cout<<" Brho : "<<(Bmin*romin)<<" Tm "<<std::endl;
                std::cout<<" Magnetic field : "<<Bmin<<" T "<<std::endl;
                std::cout<<" Radius of curvature : "<<parameter[5]<<" mm "<<std::endl;
                std::cout<<" Scattering Angle : "<<aMC*180.0/TMath::Pi()<<" deg "<<std::endl;
                std::cout<<" Azimutal Angle : "<<phiMC*180.0/TMath::Pi()<<" deg "<<std::endl;
                std::cout<<" Lenght of the experimental data : "<<parameter[7]<<cNORMAL<<std::endl;

                //aMCv=aMCv*0.8 ; //for test
                //x0MCv=x0MCv + 10.;
                /*std::cout << "  initiatialisation "<< std::endl;
                std::cout << "ouside mc icon " <<iconvar<< std::endl;
                std::cout << "outide mc mode " <<icontrol<< std::endl;
                std::cout << " out mc x0 " <<x0MCv<< " x0  "<<x0MC << std::endl;
                std::cout << " out mc y " <<y0MCv<< std::endl;
                std::cout << "outide z" <<z0MCv<< std::endl;
                std::cout << " out side mc a " <<aMCv<< std::endl;
                std::cout << " outideside mc  phi " << phiMCv<<"  "<<rangeMCv<< std::endl;*/
            double Chimin=10000000. ;




            for (imc1=0;imc1<imc1max;imc1++){
              iconvar = imc1; //this controls the step of MC in MCvar
                for (imc2=0;imc2<imc2max;imc2++){
                  icontrol=2;
                  //fPadPlane->Reset(0);
                  MCvar(parameter,icontrol,iconvar,x0MC, y0MC, z0MC, aMC,phiMC, Bmin, dens,romin,x0MCv, y0MCv,z0MCv, aMCv, phiMCv, Bminv, densv, rominv); // for MC variation with same starting value as before
                  QMCsim(parameter, Qsim, zsimq, QMCtotal,x0MCv, y0MCv, z0MCv,  aMCv, phiMCv, Bminv, densv,rominv,e0sm,PadCoord);
                  //std::cout<<cRED<<" After QMCsim x "<<x0MCv<<" y "<< y0MCv<< " z "<< z0MCv<<" theta "<< aMCv<<" phi "<< phiMCv<<" B " <<Bminv<<" dens "<< densv<< " e0sm "<<e0sm<<" ro "<<rominv<<cNORMAL;
                  Chi2MC(Qtrack,ztrackq,Qtracktotal,Qsim,zsimq,QMCtotal,CHi2fit,sigmaq,sigmaz);   //Chi2 to compare track and MC

                  if(CHi2fit<Chimin){

                      Chimin=CHi2fit;
                        //std: cout <<"**************************************Chi2min   "<<Chimin<<"  "<<CHi2fit<<endl;
                        icontrol=3;
                        //MCvar(icontrol,iconvar,x0MC, y0MC, z0MC, aMC,phiMC, rangeMC, x0MCv, y0MCv,z0MCv, aMCv, phiMCv, rangeMCv); // take the last value as new starting

                        MCvar(parameter,icontrol,iconvar,x0MC, y0MC, z0MC, aMC,phiMC, Bmin, dens,romin,x0MCv, y0MCv,z0MCv, aMCv, phiMCv, Bminv, densv, rominv);

                                    }
                }//imc2 loop
            }//imc1 loop

            //fPadPlane->Draw("zcol");


            QMCsim(parameter, Qsim, zsimq, QMCtotal,x0MC, y0MC, z0MC,  aMC, phiMC, Bmin, dens,romin,e0sm,PadCoord); // simulation with Chimin parameters

            //FitParameters.sThetaMin = aMC;
            //FitParameters.sEnerMin=e0sm;
            //FitParameters.sPosMin.SetXYZ(x0MC,y0MC,z0MC);
            //FitParameters.sBrhoMin=Bmin*romin;
            //FitParameters.sBMin=Bmin;
            //FitParameters.sPhiMin=phiMC;

            //std::cout<<cRED<<" final fit x "<<x0MC <<" y "<< y0MC << " z "<< z0MC <<" theta "<< aMC <<" phi "<< phiMC <<" B " <<Bmin <<" dens "<< dens <<" e0sm "<<e0sm<<" ro "<<romin <<cNORMAL<<std::endl;
            //Chi2MC(Qtrack,ztrackq,Qtracktotal,Qsim,zsimq,QMCtotal,CHi2fit,sigmaq,sigmaz);   //Chi2 to compare track and MC for Chimin parameters

            //FitParameters.sChi2Min=CHi2fit;
            //FitParameters.sNumMCPoint=num_MC_Point;
            //FitParameters.sNormChi2=chi2min/num_MC_Point;



            std::cout<<cYELLOW<<" Minimization result : "<<std::endl;
            std::cout<<" Scattering Angle : "<<aMC*180.0/TMath::Pi()<<std::endl;
            std::cout<<" Azimutal angle : "<<phiMC*180.0/TMath::Pi()<<std::endl;
            std::cout<<" B : "<<Bmin<<std::endl;
            //std::cout<<" Brho : "<<FitParameters.sBrhoMin<<std::endl; //August
            std::cout<<" Energy : "<<e0sm<<std::endl;
            std::cout<<" Vertex Position (Backward extrapolation) - X : "<<x0MC<<" - Y : "<<y0MC<<" - Z : "<<z0MC<<std::endl;
            //std::cout<<" Vertex Energy : "<<fVertexEner<<" MeV "<<std::endl;
            //std::cout<<" Reduced chi2 : "<<chi2min/FitParameters.sNumMCPoint<<std::endl;
            std::cout<<" Minimum chi2 : "<<CHi2fit<<cNORMAL<<std::endl;


            return kTRUE;

}


void MCQMinimization::MCvar( double* parameter, int & modevar,int & iconvar,double & x0MC, double & y0MC, double & z0MC,  double & aMC, double & phiMC, double & Bmin, double & dens, double & romin,
                             double & x0MCv,  double & y0MCv, double & z0MCv, double & aMCv, double & phiMCv,  double & Bminv, double & densv, double & rominv){


                            if(modevar==1){  //initiatialisation
                                    x0MC =parameter[0]/10.0;
                                    y0MC =parameter[1]/10.0;
                                    z0MC =fZk/10.0 - (fEntTB-parameter[3])*dzstep;
                                    aMC = parameter[6];
                                    phiMC = TMath::Pi()-parameter[4]-115*TMath::Pi()/180.0;
                                    //double Bmin= 17000. ; //magnetic field
                                    /////////// Initial parameters///////////////
                                    //Double_t xmin= parameter[0]/10.0; //  ! at ztb=394 in cm
                                    //Double_t ymin= parameter[1]/10.0;
                                    //Double_t zmin= fZk/10.0 - (fEntTB-parameter[3])*dzstep;  //Micromegas Origin  at 100 cm of the entrance
                                    //Double_t zmin= parameter[3]*dzstep;
                                    //Double_t TBmin = parameter[3]*dzstep; // Absolute TB to compare between exp and sim
                                    //Double_t phi0= (TMath::Pi()-5.0*TMath::Pi())-parameter[4]- fThetaPad; //RADIANS
                                    //Double_t phi0=parameter[4]+fThetaPad-10.0*TMath::Pi(); //RADIANS
                                    //Double_t phi0=TMath::Pi()-parameter[4]-115*TMath::Pi()/180.0;

                                    //Double_t phi0=parameter[4];

                                    Double_t bro=parameter[5]*Bmin/1000.0;// !Tm*/
                                    romin=parameter[5];
                                    Double_t theta0=parameter[6];



/*                                  std::cout << "indside mc modevar "<< modevar << "  iconvar " <<iconvar<<std::endl;
                                    std::cout << "indside mc modevar " <<modevar<< std::endl;
                                    std::cout << "indside mc x0 " <<x0MCv<< "  "<<x0MC << std::endl;
                                    std::cout << "indside mc y " <<y0MCv<<std::endl;
                                    std::cout << "indside z" <<z0MCv<<std::endl;
                                    std::cout << "indside mc a " <<aMCv<< std::endl;
                                    std::cout << "indside mc  phi " << phiMCv<<std::endl; */
                            }
                    if(modevar==2){   // MC variation
                              //   start  of MC
                              double factstep= std:: pow(1.4, -iconvar);///(TMath::Power(1.4,i));
                              double step1=4.*factstep;// angle in degres
                              double step2=4*factstep;//  !phi in deg
                              double step3=0.1*factstep;//! broradius in relativ
                              double step4=0.5*factstep;//!x0 in cm
                              double step5=0.5*factstep;// !y0
                              double step6=0.5*factstep;//!z0
                              double step7=0.0*factstep;//B
                              double step8=0.0*factstep;//Density
                              double step9=0.1*factstep;//to be used somewhere


                              aMCv= aMC + step1*(0.5-gRandom->Rndm())*0.01745;
                              phiMCv= phiMC + step2*(0.5-gRandom->Rndm())*0.01745;
                              rominv = romin*(1.+ step3*(0.5-gRandom->Rndm()));
                              x0MCv= x0MC + step4*(0.5-gRandom->Rndm());
                              y0MCv= y0MC + step5*(0.5-gRandom->Rndm());
                              z0MCv= z0MC + step6*(0.5-gRandom->Rndm());          //  std::cout<<" first loop"<<iterd<<" i "<<i<<"  j  "<<j << std::endl; //wm
                              Bminv = Bmin*(1.+step7*(0.5-gRandom->Rndm()));
                              densv = dens*(1.+ step8*(0.5-gRandom->Rndm()));
                              double helpv = step9*(0.5-gRandom->Rndm());



                            /*  std::cout << "indside mc modevar "<< modevar << "  iconvar " <<iconvar<<std::endl;
                              std::cout << "indside mc modevar " <<modevar<< std::endl;
                              std::cout << "indside mc x0 " <<x0MCv<< "  "<<x0MC << std::endl;
                              std::cout << "indside mc y " <<y0MCv<<std::endl;
                              std::cout << "indside z" <<z0MCv<<std::endl;
                              std::cout << "indside mc a " <<aMCv<< std::endl;
                              std::cout << "indside mc  phi " << phiMCv<<std::endl;
                              */



                      }

                      if(modevar==3){  //lower chi2 hence new starting condition
                        x0MC=x0MCv;
                        y0MC=y0MCv;
                        z0MC =z0MCv;
                        aMC =aMCv;
                        phiMC =phiMCv;
                        Bmin = Bminv;
                        dens = densv;
                        romin = rominv;

                      }

}

void MCQMinimization::QMCsim(double* parameter, double* Qsim,double *zsimq,double & QMCtotal,
       double x0MC,double y0MC, double z0MC, double aMC, double phiMC, double Bmin, double dens, double romin,double & e0sm, multiarray PadCoord) {


/*         std::cout << "indside sim  x0 " <<x0MC<< "  "<<x0MC << std::endl;
         std::cout << "indside sim y " <<y0MC <<std::endl;
         std::cout << "indside sim z " <<z0MC <<std::endl;
         std::cout << "indside sim a " <<aMC << std::endl;
         std::cout << "indside sim  phi " << phiMC <<std::endl;
         std::cout << "indside sim Bmin  " <<Bmin<< std::endl;
         std::cout << "indside sim dens  " << dens <<std::endl;
         std::cout << "indside sim romin   " << romin <<std::endl;  */



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


                                       Double_t xTBCorr[10000];
                                       Double_t yTBCorr[10000];
                                       Double_t zTBCorr[10000];

                                       std::fill_n(xTBCorr,-10000, -10000);
                                       std::fill_n(yTBCorr,-10000, -10000);
                                       std::fill_n(zTBCorr,-10000, -10000);


                                      //Parameters
                                      //Double_t e0sm            = 0.0;
                                      Double_t smprot          = 931.49432;
                                      //Double_t Bmin            = B;
                                      Double_t chi2min         = 1E10;
                                      Double_t thetaLorentz    = fThetaLorentz;
                                      Double_t thetaRot        = fThetaRot;
                                      Double_t thetaTilt       = fTiltAng;
                                      Double_t thetaPad        = fThetaPad;

                                      /////////// Initial parameters///////////////
                                      /* Double_t xmin= parameter[0]/10.0; //  ! at ztb=394 in cm
                                      Double_t ymin= parameter[1]/10.0;
                                      Double_t zmin= fZk/10.0 - (fEntTB-parameter[3])*dzstep;  //Micromegas Origin  at 100 cm of the entrance
                                      //Double_t zmin= parameter[3]*dzstep;
                                      Double_t TBmin = parameter[3]*dzstep; // Absolute TB to compare between exp and sim
                                      //Double_t phi0= (TMath::Pi()-5.0*TMath::Pi())-parameter[4]- fThetaPad; //RADIANS
                                      //Double_t phi0=parameter[4]+fThetaPad-10.0*TMath::Pi(); //RADIANS
                                      Double_t phi0=TMath::Pi()-parameter[4]-115*TMath::Pi()/180.0;

                                      //Double_t phi0=parameter[4];

                                      Double_t bro=parameter[5]*B0/1000.0;//
                                       Double_t theta0=parameter[6];
                                       */
                        //if (!fout1.is_open()) {
                           //cerr << "error: open output file failed" << endl;
                            //  }

                            double xpad[10000]={0}; //these are the partial intergal results in small steps
                            double ypad[10000]={0};
                            double zpad[10000]={0};
                            double Qpad[10000]={0};
                            /*   for(Int_t idef=0;idef<10000;idef++) {
                            Qsim[idef]={0.};
                            zsimq[idef]={0.};
                          } */
                            std::fill_n(Qsim,10000,0);
                            std::fill_n(zsimq,10000,0);
                            double Qpadplane[10000]={0.};   //wm
                            double zpadplane[10000]={0.};
                            Int_t npadtotal=10000  ;
                            double tracksimthreshold=5; // threshold for simulated track in number of primary electrons
                            double Qtotalsim=0.;  // total charge of the simulated track

                            //Qsim[][]={0.}
                            //double Qsim[10000]={0.}; //simulated amplitude
                          //double Qtrack[10000]={0.}; //simulated amplitude for track to analyse
                         //double ztrackq[10000]={0.}; //simulated amplitude for track to analyse *ztrack fo find center of gravity
                      //      double zsimq[10000]={0.}; //simulated amplitude for track to analyse *ztrack fo find center of gravity
                      //bro=Bmin*romin; //!in Tm
                      //double x0MC,double y0MC, double z0MC, double aMC, double phiMC, double Bmin, double dens, double romin
                      double theta0=aMC;
                      double rometer=0.001*romin;
                      double brotheta=Bmin*romin*1.e-7/TMath::Sin(theta0);//  !initial bro corrected for angle in Tm
                      //            phi0= (-20.+ step2*(0.5-rand(0)))*0.01745 +180.
                      double phi0= phiMC;
                      double B=Bmin;


                      GetEnergy(sm1,z1,brotheta,e0sm);
                      //std::cout << "indside sim e0sm   " << e0sm <<std::endl;

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
                      Double_t  x = x0MC; // ! ztb=394
                      Double_t  y = y0MC;
                      Double_t  z = z0MC;
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
                                            //Double_t dens=0.06363*18*(1+step8*(gRandom->Rndm()-0.5))/20.;//  !DENSITY ISOBUTANE AT 20 TORR corrected 18 torr
                                            Double_t integrationmax=10000;
                                            ipr=0;

                                //******************************************************************

                                             Int_t integration=0;
                                             Int_t iterd=0;
                                             Int_t iterCorr=0;
                                             Int_t iterCorr_0=0; //offset correction for TB
                                             Int_t iterd0=0;
                                             Int_t iterCorrNorm=0;

                                             Int_t icnb;
                                             Int_t num_MC_Point = 0;

                                             for(Int_t k=0;k<integrationmax;k++)
                                             {
                                                   integration++;
                                                   Float_t factq=1.0;
                                                   //iterd=(Int_t) k/integrationsteps;//NEW
                                                   iterd=k;
                                                                //fout1<<" x "<<x<<" y "<<y<<" z "<<z<<"  Q" << Qpad[iterd]<< endl;


                                                /////  if(iterd0==iterd) // Only takes the first one of hte iterd series
                                                   xcmm[iterd] = x*10.0;
                                                   ycmm[iterd] = y*10.0;
                                                   zcmm[iterd] = z*10.0;

                                                   //zcmm[iterd] = -z*10. + 2*zmin*10.0;
                                                  // iterd0++;


                                                   //Transform to pad plane before propagation
                                                /* xsol[iterd]=xcmm[iterd]-zcmm[iterd]*TMath::Sin(thetaLorentz)*TMath::Sin(thetaRot);
                                                   ysol[iterd]=ycmm[iterd]+zcmm[iterd]*TMath::Sin(thetaLorentz)*TMath::Cos(thetaRot);
                                                   zsol[iterd]=zcmm[iterd];

                                                   xdet[iterd] = xsol[iterd];
                                                   ydet[iterd] = (fZk-zsol[iterd])*TMath::Sin(thetaTilt) + ysol[iterd]*TMath::Cos(thetaTilt);
                                                   zdet[iterd] = zsol[iterd]*TMath::Cos(thetaTilt) + ysol[iterd]*TMath::Sin(thetaTilt);

                                                   xpad[iterd] = xdet[iterd]*TMath::Cos(thetaPad) - ydet[iterd]*TMath::Sin(thetaPad);
                                                   ypad[iterd] = xdet[iterd]*TMath::Sin(thetaPad) + ydet[iterd]*TMath::Cos(thetaPad);
                                                   zpad[iterd] = zdet[iterd];
                                                   //zpad[iterd] = -zdet[iterd] + 2*zmin*10.0;*/

                                                    //std::cout<<" k : "<<k<<std::endl;
                                                    //std::cout<<" zcmm : "<<zcmm[iterd]<<"  2*zmin*10.0  : "<<2*zmin_trans*10.0<<std::endl;
                                                    zcmm[iterd] = -zcmm[iterd]+ 2*zmin_trans*10.0;
                                                    //std::cout<<" zcmm : "<<zcmm[iterd]<<"  2*zmin*10.0  : "<<2*zmin_trans*10.0<<std::endl;
                                                    xsol[iterd]=xcmm[iterd]-zcmm[iterd]*TMath::Sin(thetaLorentz)*TMath::Sin(thetaRot);
                                                    ysol[iterd]=ycmm[iterd]+zcmm[iterd]*TMath::Sin(thetaLorentz)*TMath::Cos(thetaRot);
                                                    zsol[iterd]=zcmm[iterd];

                                                    xdet[iterd] = xsol[iterd];
                                                    ydet[iterd] = -(fZk-zsol[iterd])*TMath::Sin(thetaTilt) + ysol[iterd]*TMath::Cos(thetaTilt);
                                                    zdet[iterd] = zsol[iterd]*TMath::Cos(thetaTilt) - ysol[iterd]*TMath::Sin(thetaTilt);

                                                    xpad[iterd] = xdet[iterd]*TMath::Cos(thetaPad) - ydet[iterd]*TMath::Sin(thetaPad);
                                                    ypad[iterd] = xdet[iterd]*TMath::Sin(thetaPad) + ydet[iterd]*TMath::Cos(thetaPad);
                                                    zpad[iterd] = zdet[iterd];

                                                    //zpad[iterd] = -zdet[iterd]+ 2*zmin*10.0;



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
                                                      sloss=6.98*(1./TMath::Power(c0,0.83))*(1./(20.+1.6/TMath::Power(c0,1.3)))+0.45*TMath::Exp(-55.*TMath::Power((c0-0.025),2)); // expression modified June 14 2016
                                                      //sloss=6.98*(1./TMath::Power(c0,0.83))*(1./(20.+1.6/TMath::Power(c0,1.3)))+1.0*TMath::Exp(-55.*TMath::Power((c0-0.025),2)); // expression modified June 14 2016 low energy term changed
                                                      //

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
                                                        Qpad[iterd] = sloss/(3.e-5); //wm 30eV/e
                                                        //fout1<<" x "<<x<<" y "<<y<<" z "<<z<<"  Q   " << Qpad[iterd]<< endl;
                                                        Qtotalsim = + Qpad[iterd]; //total cumulative charge
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
                                                        //if(ekin<0.01 || isnan(ekin)) break;
                                                        if(ekin<0.01 || std::isnan(ekin)) break;
                                                        //if(ekin<0) std::cout<<" Ekin "<<std::endl;


                                                      }// loop over k


                                                      //std::cout<<" Integration : "<<integration<<std::endl;
                              fPadPlane->Reset(0);

                            for( iterd=0;iterd<integration;iterd++){
                              double costheta=cos(aMC);
                              double sintheta=sin(aMC);
                              double cosphi=cos(phiMC);
                              double sinphi=sin(phiMC);
                              //xpad[iterd]= iterd*step_iter*sinphi*costheta+x0MC;
                              //ypad[iterd]= iterd*step_iter*sinphi*sintheta + y0MC ;
                              //zpad[iterd]= iterd*step_iter*cosphi + z0MC;
                                //Qpad[iterd] = 100.; // energy loss NB to be replaced
                                //defintion of padplane with rectangular pads

                                //double dxpad=4.;
                                //double dypad=4.;
                                //double zstep=2.;
                                /*indexx=xpad[iterd]/dxpad;
                                if (indexx>>99) indexx=99 ;

                                indexy=ypad[iterd]/dypad;
                                    if (indexy>>99) indexy=99 ;
                                indexz=zpad[iterd]/zstep;
                                    if (indexz>>99) indexz=99 ;*/
                                //introduce straggling
                                int istrag=9;
                            /*          f=x*exp(-x**2/2.)
                            meanr  rmax dProbability  probability
                            0.379256755      0.758513510      0.250009149      0.379256755      0.568891168      0.250009149
                            0.967998266       1.17748296      0.250010014      0.588741541      0.588690937      0.500019193
                            1.42142344       1.66536391      0.250012964      0.453425169      0.416164398      0.750032187
                            6.66956806       11.6737719      0.249981672       5.24814463       2.98571382E-29   1.00001383
                            */
                              double sigstrtrans=0.010*sqrt(zpad[iterd]) ; //in cm
                              double sigstrlong=0.025*sqrt(zpad[iterd]) ; //in cm but zpad is in mm
                              double rstrag[4]={0.};
                                rstrag[0] =0.37925;
                                rstrag[1] =  0.968;
                                rstrag[2] =  1.421 ;
                                rstrag[3] = 6.69 ; //normalized radius for 4 values
                                    //rstrag[3]= 2.  ; // Not correct for test
                                int isig;
                               for (isig=0;isig<4;++isig)
                               {
                                  double rstr= rstrag[isig]*sigstrtrans;
                                  int iphi;

                                  for(iphi=0;iphi<8;++iphi) {

                                   double phi=0.7854*iphi;  //45 degrees *iphi
                                   //std:: cout<<"iphi  "<< iphi<<" phi  "<<phi<<" isig  "<<isig<<" sigstr  " << rstr<<endl;
                                   double xstr=cos(phi)*rstr;
                                   double ystr=sin(phi)*rstr;
                                   //int indexxs=(xpad[iterd]+xstr)/dxpad;
                                   //int indexys=(ypad[iterd]+ystr)/dypad;
                                   double X_str = xpad[iterd]+xstr;
                                   double Y_str = ypad[iterd]+ystr;

                                   // nota bene: we do not for the moment calculate the longitudinal straggling because only center of gravity used
                                   double Q_str = Qpad[iterd]/32.;
                                  //  fout1<<" xstr "<<X_str<<" ystr "<<Y_str<<" z "<<z<<"  Qstr   " << Q_str<< endl;
                                   //Qsim[indexxs +100*indexys] += Qpad[iterd]/32.;  //sum charge normalized 4 radius 8 angles
                                   //zsimq[indexxs +100*indexys] += zpad[iterd]*Qpad[iterd]/32.;  //sum charge normalized 4 radius 8 angles
                                   //std::cout<<"integration :  "<<integration<<" X_str : "<<X_str<<" Y_str : "<<Y_str<<" Q_str : "<<Q_str<<std::endl;
                                   Int_t pBin =  fPadPlane->Fill(X_str,Y_str,Q_str);
                                   pBin=pBin-1;
                                   //Double_t qBin = fPadPlane->GetBinContent(pBin);
                                   // calculate center of gravity for z
                                   if(pBin>0){
                                   Qsim[pBin] += Q_str;  //charge in pad

                                   zsimq[pBin] += Q_str*zpad[iterd]; //not yet normalized
                                 //std::cout<<"X_str : "<<X_str<<" Y_str : "<<Y_str<<"  Pad Number : "<<pBin+1<<" Bin content : "<<qBin<<"  table "<<Qsim[pBin]<<std::endl;
                                  } // end if
                                                          }// phi angle

                               }  //end of straggling
                               // put in memory the cumulated charge per pad

                            //          std:: cout<<" xpad "<<xpad[iterd]<<"  "<<iterd<<std::endl;
                            //          std:: cout<<" xind  "<<indexx<<" yind "<<indexy<<" Q "<<Qsim[indexx +100*indexy]<<endl;
                            //          fout1<<" xind  "<<indexx<<" yind "<<indexy<<" Q "<<Qsim[indexx +100*indexy]<<endl;
                          }// integration loop
                            // calculate normalized track and define the simulated track
                              Int_t ivalidsim=0;
                              //fout1<<" indexpad " <<" xind "<<" yind "<<" zqusim "<<"  Qsim "<<" ysim 1 2 3 4 5 6 7 8 9 10"<<endl;
                            for (Int_t i = 0; i < npadtotal; i++) {
                                //fout1<<"  "<<zsimq[i]<<endl;
                                  double qsimnorm=4.0;  //normalization of simulation to fit exp. amplitudes is str dependant
                                  Qsim[i] = Qsim[i]*qsimnorm; //noramlized: attention Zsim was maultiplied by the unnormalized
                                if(Qsim[i] > tracksimthreshold)  //threshold after renormalization
                                {
                                  //int  ihelp=fPadPlane->GetBinCenter(i);
                                  //double xhelp=PadCoord[i][0][0];
                                  //double yhelp=PadCoord[i][0][1];
                                  Float_t xhelp = (PadCoord[i][0][0] + PadCoord[i][1][0] + PadCoord[i][2][0])/3.;// x center of gravity of the pad
                                  Float_t yhelp= (PadCoord[i][0][1] + PadCoord[i][1][1] + PadCoord[i][2][1])/3.;// y center of gravity of the pad
                                  //double xhelp1= PadCoord[i][1][0];
                                  //double xhelp2= PadCoord[i][2][0];
                                  //double yhelp1=PadCoord[i][0][2];
                                  //fout1<<"  test of coordinates"<<xhelp<<"  1 "<<xhelp1<<" 2  "<<xhelp2<<"   y "<<yhelp<<"  "<<"   y1 "<<yhelp1<<endl;
                                  ivalidsim++  ;
                                    double help;
                                    help = zsimq[i];
                                   zsimq[i]  = qsimnorm*zsimq[i]/Qsim[i];//normalized position cuidad norm
                                //std::cout<<" Inner loop  "<< i << "  nn "<< help <<" z "<<zsimq[i]<<" Qsim "<< Qsim[i] <<std::endl;
                                   //fout1<<"  "<<i<<"  "<< xhelp<<"  "<<yhelp<<" " << zsimq[i]<< "  "<<Qsim[i];
                                   xiter.push_back(xhelp);
                                   yiter.push_back(yhelp);
                                   ziter.push_back(zsimq[i]);



                                      Int_t iplot= 10*sqrt((Qsim[i])/2000.);
                                      int ipl;
                                      if (iplot>10)iplot=10;
                                      //for (ipl=0;ipl<(iplot);++ipl){
                                      //  fout1<<"  "<<yhelp;
                                      //}
                                      //fout1<<"  "<<yhelp<<endl;
                                }// normalized



                              }//loop for tracksimthreshold

                              fPosXmin=xiter;
                              fPosYmin=yiter;
                              fPosZmin=ziter;

                              //std::cout<<" End of QMCsim : "<<Qsim[103]<<std::endl;


}

void MCQMinimization::Chi2MC(double*  Qtrack,double*  ztrackq,double & Qtracktotal,double* Qsim ,double*  zsimq ,double & QMCtotal,double & Chi2fit, double & sigmaq, double & sigmaz) {

                  double trackthreshold= 50. ;
                  int  npadtotal =10000;
                  int indxval[10000]={0};
                  int indyval[10000] = {0};
                  int indzval[10000] = {0}; //index for tracks >threshold
                  double Qtrackval=0.;
                  double Qsimval=0.;
                  //int npoints=0;
                  int indexx=0;
                  int indexy=0;
                  int indexz=0;


/*// first check normalization with missing pads (Qtrack<threshold)
// to speed up defined indexes for only valable track points
                  for (indexx=0;indexx<99;++indexx){  //renormalize for charge on the valid region= exper.region
                    for (indexy=0;indexy<99;++indexy){
                      if ((Qtrack[indexx +100*indexy])>trackthreshold){
                        npoints +=1;
                        indxval[npoints]= indexx;
                        indyval[npoints]= indexy;
                        //indzval[npoints]= ztrackq [indexx +100*indexy];
                        Qtrackval += Qtrack[indexx +100*indexy]; // could be done at only first iteration
                        Qsimval += Qsim[indexx +100*indexy]; // this must be redone at all iterations
                  }

              }

            }  */

                      double ratioq=1.  ;  //supposing normalization done
                      //std:: cout << "npoints  "<<npoints<<"  Qtrack  " <<  Qtrackval << " Qsim "<<  Qsimval<< "  ratio  " <<ratioq<<endl;
                      // chi2 calculation
                      Chi2fit=0.;
                      double Chi2fitq=0.;
                      int i;
                      double Chi2fitz=0.;
                      double Chi2number=0. ;
                      int npoints=0;
                      int npointsim=0;
                      double qsimthreshold= 50. ;
                      double qtrackthreshold= 50. ;
    for (i=0; i<npadtotal; i++){
          //std:: cout <<"  i "<< i << "   "<< Qsim[i]<<"  "<< Qtrack[i]<< endl;
            if(Qsim[i] > qsimthreshold) {
              npointsim= npointsim+1;
              if(Qtrack[i]>qtrackthreshold) {
       double dQ= (Qsim[i] -  ratioq*Qtrack[i]);
       double Qsum= (Qsim[i] + ratioq*Qtrack[i]); // for sigma variance calculation
       double dz= (zsimq [i] -ztrackq [i]);
       double sigq= dQ/(Qsum*sigmaq);
       double sigz=dz/sigmaz;
       npoints=npoints+1 ;

       Chi2fitq += sigq*sigq ;
       Chi2fitz += sigz*sigz;
       //std:: cout<< " sim   "<< Qsim[i] <<"  track  " << Qtrack[i]<< "  Chi2z " << Chi2fitz<<"  sigz  "<<sigz<<" zsim "<<zsimq[i]<< " ztrack= "<<ztrackq[i]<<" npoints  " <<npoints<<endl;
                                        }
                                      }
    }
 // normalized
      // to optimize a similar number of points simulated and in the final fit

    //Chi2number=  ((npointsim  - npoints)*(npointsim  - npoints)/float(npointsim + npoints))/10.;
    //Chi2number= Chi2number/float(npointsim + npoints);
    Chi2fitq = Chi2fitq/float(npoints) ;
    Chi2fitz = Chi2fitz/float(npoints) ;
    //if(npoints>0) Chi2fit= (Chi2fitq+Chi2fitz+Chi2number)/float(2*npoints);
    if(npoints>0) Chi2fit= (Chi2fitq+Chi2fitz)/float(2*npoints);
    if(npoints>0) Chi2fit= Chi2fit/float(2*npoints);   /// for z and Q and npoints
    if(npoints<5) Chi2fit= 10000.;  /// to avoid chi2 by too little number of points
//std:: cout<< " chi2fitnorm**3  "<<Chi2fit<<"  Chi2q  " << Chi2fitq<<"  Chi2z " << Chi2fitz<<" Chi2number "<< Chi2number<<" npoints"<< npoints <<endl;
    //std:: cout<< " chi2fitnorm**3  "<<Chi2fit<<"  Chi2q  " << Chi2fitq<<"  Chi2z " << Chi2fitz<<" npoints "<< npoints <<endl;

}

void MCQMinimization::GetEnergy(Double_t M,Double_t IZ,Double_t BRO,Double_t &E){

  Float_t  AM=931.5;
  Float_t X=BRO/0.1439*IZ/M;
  X=pow(X,2);
  X=2.*AM*X;
  X=X+pow(AM,2);
  E=TMath::Sqrt(X)-AM;

}

TVector3 MCQMinimization::TransformIniPos(Double_t x,Double_t y, Double_t z)
{

   TVector3 PosIniCmm;

   Double_t x_det = x*TMath::Cos(fThetaPad)  + y*TMath::Sin(fThetaPad);
   Double_t y_det = -x*TMath::Sin(fThetaPad) + y*TMath::Cos(fThetaPad);
   Double_t z_det = z;

   Double_t x_sol = x_det;
   Double_t z_sol = ( z_det*TMath::Cos(fTiltAng) + y_det*TMath::Sin(fTiltAng) + fZk/10.0*TMath::Power(TMath::Sin(fTiltAng),2) ) ;
   Double_t y_sol = ( y_det + (fZk/10.0-z_sol)*TMath::Sin(fTiltAng) )/ TMath::Cos(fTiltAng);

   Double_t z_cmm = z_sol;
   Double_t x_cmm = x_sol + z_cmm*TMath::Sin(fThetaLorentz)*TMath::Sin(fThetaRot);
   Double_t y_cmm = y_sol - z_cmm*TMath::Sin(fThetaLorentz)*TMath::Cos(fThetaRot);

   PosIniCmm.SetXYZ(x_cmm,y_cmm,z_cmm);

  return PosIniCmm;


}

TVector3 MCQMinimization::InvTransIniPos(Double_t x,Double_t y, Double_t z)
{

  TVector3 InvPosIni;

  Double_t xsol=x-z*TMath::Sin(fThetaLorentz)*TMath::Sin(fThetaRot);
  Double_t ysol=y+z*TMath::Sin(fThetaLorentz)*TMath::Cos(fThetaRot);
  Double_t zsol=z;

  Double_t xdet = xsol;
  Double_t ydet = -(fZk/10.0-zsol)*TMath::Sin(fTiltAng) + ysol*TMath::Cos(fTiltAng);
  Double_t zdet = zsol*TMath::Cos(fTiltAng) - ysol*TMath::Sin(fTiltAng);

  Double_t xpad = xdet*TMath::Cos(fThetaPad) - ydet*TMath::Sin(fThetaPad);
  Double_t ypad = xdet*TMath::Sin(fThetaPad) + ydet*TMath::Cos(fThetaPad);
  Double_t zpad = zdet;

  InvPosIni.SetXYZ(xpad,ypad,zpad);

  return InvPosIni;

}

Int_t MCQMinimization::GetTBHit(Int_t TB,std::vector<ATHit> *harray)
{

          auto it =  find_if( harray->begin(),harray->end(),[&TB](ATHit& hit) {return hit.GetTimeStamp()==TB;}   );
          if(it != harray->end()){
             auto hitInd = std::distance<std::vector<ATHit>::const_iterator>(harray->begin(),it);
             return hitInd;
          }
          else return 0;

}

std::vector<ATHit> MCQMinimization::GetTBHitArray(Int_t TB,std::vector<ATHit> *harray)
{

        std::vector<ATHit> hitTBArray;
        std::copy_if(harray->begin(), harray->end(), std::back_inserter(hitTBArray),[&TB](ATHit& hit){return hit.GetTimeStamp()==TB;} );
        return hitTBArray;
}
