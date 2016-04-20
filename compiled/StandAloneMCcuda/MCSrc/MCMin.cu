/**************************************************************
 * \file MCMin.cu
 *
 * \brief CUDA implementation of Monte Carlo Minimization
 * \author M.P. Kuchera and Y. Ayyad
 * \date created 19 March 2016
 *
 **************************************************************/



#include "MCMin.cuh"

#include <iostream>
#include <algorithm>
#include "TRandom.h"
#include "TH1F.h"
#ifdef _OPENMP
#include <omp.h>
#endif
#include <curand.h>
#include <curand_kernel.h>
#include <math_constants.h>

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

#define ITER 400
#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess) 
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}
//Global variables on the GPU
__device__ Double_t sm1;
__device__ Double_t m;
__device__ Double_t dzstep;
__device__ Int_t    integrationsteps;
__device__ Double_t restmass;
__device__ Double_t esm;
__device__ Double_t iz1;
__device__ Double_t z1;
__device__ Double_t B0;
//__device__ Double_t B;
__device__ Double_t fZk;
__device__ Double_t phimin[ITER];//=phi0;
__device__ Double_t thetamin[ITER];//=theta0;
__device__ Double_t bromin[ITER];//=bro;
__device__ Double_t Bmin[ITER];
//__device__ Double_t phi0;
//__device__ Double_t theta0;
//__device__ Double_t bro;
  /////////// Initial parameters///////////////
__device__ Double_t xmin;//= parameter[0]/10.0; //  ! at ztb=394 in cm
__device__ Double_t ymin;//= parameter[1]/10.0;
__device__ Double_t zmin;//= fZk/10.0 - (fEntTB-parameter[3])*dzstep;  //Micromegas Origin  at 100 cm of the entrance
__device__ Double_t TBmin;// = parameter[3]*dzstep;
__device__ Double_t fEntTB;
__device__ Double_t phi0[ITER];
__device__ Double_t bro[400];
__device__ Double_t theta0[400];
__device__ Double_t B[400];
__device__ Double_t brotheta[400];


__shared__ Double_t fThetaPad;
__shared__ Double_t fThetaTilt;
__shared__ Double_t fThetaLorentz;
__shared__ Double_t fThetaRot;

//_device__ Double_t e0sm[400];
//__device__  Double_t phi0=TMath::Pi()-parameter[4]-115*TMath::Pi()/180.0;
  
//__device__  Double_t bro=parameter[5]*B0/1000.0;// !Tm*/
//__device__  Double_t theta0=parameter[6];
  ////////////////////////////////////////////

typedef struct {
  Double_t X;
  Double_t Y;
  Double_t Z;
} vec3;

__global__ void setGlobalGPUVariables(){
  m                  = 1;
  sm1                = m;
  dzstep             = 5.20*80.0/1000.0;//  !unit cm
  integrationsteps   = 10;
  iz1                = 1;
  z1                 = iz1;
  restmass           = sm1*931.49432;
  esm                = z1*1.75879e-3*0.510998918/restmass;// ![e/m electron cm**2/(Volt*nsec**2] this is not the energy/mass but charge/mass
  B0                 = 1.66;  // !	magnetic field
  // B                  = B0*10000.; // !conversion of T en Gauss
  fZk              = 1000.0;
  fEntTB           = 280;
  // B                = B0*10000;
  fThetaLorentz    = -6.6*CUDART_PI_F/180.0;
  fThetaRot        = -13.0*CUDART_PI_F/180.0;
  fThetaTilt       = 7.4*CUDART_PI_F/180.0;
  fThetaPad        = 113.7*CUDART_PI_F/180.0;
}
__device__ vec3 setXYZ(Double_t x, Double_t y, Double_t z){
  vec3 vec;
  vec.X = x;
  vec.Y = y;
  vec.Z = z;
  return vec;
}
__device__ vec3 TransformIniPos(Double_t x,Double_t y, Double_t z)
{

   vec3 PosIniCmm;

   Double_t x_det = x*cos(fThetaPad)  + y*sin(fThetaPad);
   Double_t y_det = -x*sin(fThetaPad) + y*cos(fThetaPad);
   Double_t z_det = z;

   Double_t x_sol = x_det;
   Double_t z_sol = ( z_det*cos(fThetaTilt) + y_det*sin(fThetaTilt) + fZk/10.0*pow(sin(fThetaTilt),2.0) ) ;
   Double_t y_sol = ( y_det + (fZk/10.0-z_sol)*sin(fThetaTilt) )/ cos(fThetaTilt);

   Double_t z_cmm = z_sol;
   Double_t x_cmm = x_sol + z_cmm*sin(fThetaLorentz)*sin(fThetaRot);
   Double_t y_cmm = y_sol - z_cmm*sin(fThetaLorentz)*cos(fThetaRot);

   PosIniCmm = setXYZ(x_cmm,y_cmm,z_cmm);

   return PosIniCmm;


}
__device__ __host__ void MCMinimization::GetEnergy(Double_t M,Double_t IZ,Double_t BRO,Double_t &E) {
  
}

__device__ Double_t GetEnergy(Double_t M,Double_t IZ,Double_t BRO) {
  Float_t  AM=931.5;
  Float_t X=BRO/0.1439*IZ/M;
  X=powf(X,2.0);
  X=2.*AM*X;
  X=X+powf(AM,2.0);
  Double_t E=sqrt(X)-AM;
  return E;
}



__global__ void setInitialParameters(Double_t* parameter){
  int idx = blockIdx.x*blockDim.x+threadIdx.x;
  if(idx < 400){
  
    phimin[idx]=phi0[idx];
    thetamin[idx]=theta0[idx];
    bromin[idx]=bro[idx];
    
    Double_t tm=0.0;
    //Int_t iteration=0;

    /////////// Initial parameters///////////////
    xmin= parameter[0]/10.0; //  ! at ztb=394 in cm
    ymin= parameter[1]/10.0;
    zmin= fZk/10.0 - (fEntTB-parameter[3])*dzstep;  //Micromegas Origin  at 100 cm of the entrance
    TBmin = parameter[3]*dzstep;
    phi0[idx]=CUDART_PI_F-parameter[4]-115*CUDART_PI_F/180.0;
    B[idx]                  = B0*10000.; // !conversion of T en Gauss
    bro[idx]=parameter[5]*B0/1000.0;// !Tm*/
    theta0[idx]=parameter[6];
    brotheta[idx] = bromin[idx]/sin(thetamin[idx]);
      
    fThetaPad = 0;
    fThetaTilt = 0;
    fThetaLorentz = 0;
    fThetaRot = 0;
    
    //  brotheta[idx]=bromin[idx]/sin(thetamin[idx]);
    Double_t e0sm = GetEnergy(sm1,z1,brotheta[idx]);
    printf(" Energy of the proton : %f MeV\n",e0sm);
    Double_t chimininit=1.e6;
    if(bro==0 || isnan(e0sm) || e0sm>100.0){
      printf(" Invalid energy !\n\n");//<<cNORMAL<<std::endl;
      return;// kFALSE;
  }

  ////////////////////////////////////////////
  }
  if(idx==0){
    printf(cGREEN "\n\n ============================\n");
    printf(" Starting Monte Carlo event \n");
    printf(" X : %f cm  - Y : %f cm - Z : %f cm \n",xmin,ymin,zmin);//<<std::endl;
    printf(" Brho : %f Tm \n",bro[idx]);//<<std::endl;
    printf(" Radius of curvature : %f mm\n",parameter[5]);//<<" mm "<<std::endl;
    printf(" Scattering Angle : %f deg\n",theta0[idx]*180.0/CUDART_PI_F);//<<" deg "<<std::endl;
    printf(" Azimutal Angle : %f deg\n",phi0[idx]*180.0/CUDART_PI_F);
    printf(" Length of the experimental data : %f\n============================\n" cNORMAL,parameter[7]);//<<cNORMAL<<std::endl;
  }
  
}
__global__ void init_stuff(curandState* state)
{
int idx = blockIdx.x*blockDim.x+threadIdx.x;
curand_init(1337,idx,0,&state[idx]);
}

__global__ void make_rand(curandState* state,float* randArray)
{
int idx = blockIdx.x*blockDim.x+threadIdx.x;
randArray[idx]=curand_uniform(&state[idx]);
}


__global__ void calcInner(int i, curandState* state, Double_t* parameter) {
  int idx = blockIdx.x*blockDim.x+threadIdx.x;
  // printf(cRED "idx = %d\n" cNORMAL,idx);
  if(idx < 400) {
     Double_t xcmm = 0;
     Double_t ycmm = 0;
     Double_t zcmm = 0;
     
     Double_t xsol = 0;
     Double_t ysol = 0;
     Double_t zsol = 0;
     
     Double_t xdet = 0;
     Double_t ydet = 0;
     Double_t zdet = 0;
     
     Double_t xpad = 0;
     Double_t ypad = 0;
     Double_t zpad = 0;
     
     Double_t xTBCorr[100];
     Double_t yTBCorr[100];
     Double_t zTBCorr[100];
     
    //setInitialParameters();
    Float_t factstep=1.0/(pow(1.4,(double)i));
    Float_t step1=2*factstep;// !theta in deg
    Float_t step2=2*factstep;//  !phi in deg
    Float_t step3=0.2*factstep;//! broradius in realtive vaue
    Float_t step4=0.3*factstep;//!x0 in cm
    Float_t step5=0.3*factstep;// !y0
    Float_t step6=0.5*factstep;//!z0
    Float_t step7=0.0*factstep;//B
    Float_t step8=0.0*factstep;//Density
    Double_t e0sm            = 0.0;
    Double_t tm=0.0;


    Bmin[idx] = B[idx];
    bro[idx]=bromin[idx]*(1.+(0.5-curand_uniform(&state[idx]))*step3); //!in Tm
    theta0[idx]=thetamin[idx]+step1*(0.5-curand_uniform(&state[idx]))*0.01745;
    brotheta[idx]=bro[idx]/sin(theta0[idx]);//  !initial bro corrected for angle
    
    phi0[idx]= phimin[idx]+ step2*(0.5-curand_uniform(&state[idx]))*0.01745;
    B[idx]=Bmin[idx]*(1.+step7*(0.5-curand_uniform(&state[idx])));
    e0sm = GetEnergy(sm1,z1,brotheta[idx]);
    Double_t e0ll=e0sm*sm1;
    Double_t e0=e0ll*1000000.;// !conversion from MeV in eV kinetic energy
                              //	esm= 1.75879e-3
    Double_t beta2=2.*e0ll/(sm1*931.49);
    Double_t ekin=e0ll;
    Double_t beta0=sqrt(beta2);//    ![cm/nsec] 

    Double_t ecinsm=ekin/sm1;
    Double_t eloss=ekin;
    //	dedx=s*dens  !de/dx in [MeV/cm] !only defined after 1st tour
    //          dzstep=0.221  !unit cm
    //	boucle d'integration
    Int_t  ipr=0;
    Double_t  range=0.0;
    
    //        define initial conditions
    
    //         Transform initial parameters into lab
    Double_t  x = xmin + step4*(curand_uniform(&state[idx])-0.5); // ! ztb=394
    Double_t  y = ymin + step5*(curand_uniform(&state[idx])-0.5);
    Double_t  z = zmin + step6*(curand_uniform(&state[idx])-0.5);
    Double_t x_buff =x;
    Double_t y_buff =y;
    Double_t z_buff =z;  
    vec3 PosIniCmm;
    // Working here Mich //////TVector3 PosIniCmm = TransformIniPos(x,y,z); //Arguments in cm
    x=PosIniCmm.X;
    y=PosIniCmm.Y;
    z=PosIniCmm.Z;
    Double_t zmin_trans = z;
    Double_t  x0=x;
    Double_t  y0=y;
    Double_t  z0=z;
    
    Double_t v0=beta0*29.9792;//  !v in cm/ns
    Double_t dt=dzstep/(v0*cos(theta0[idx]));//![unite temps ns]
    dt=dt/(Float_t)integrationsteps; // NEW
    Double_t t=-dt;
    
    //           initial velocity vector
    //	v0=sqrt(v2)    ![cm/nsec]
    Double_t dxdt=v0*sin(theta0[idx])*cos(phi0[idx]);
    Double_t dydt=v0*sin(theta0[idx])*sin(phi0[idx]);
    Double_t dzdt=v0*cos(theta0[idx]);
    Double_t iprinttr=1000;
    Double_t dens=0.06363*18*(1+step8*(curand_uniform(&state[idx])-0.5))/20.;//  !DENSITY ISOBUTANE AT 20 TORR corrected 18 torr
    Double_t iterationmax=10000;
    ipr=0;
    
    //******************************************************************
    
    Int_t iteration=0;
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
	
	xcmm = x*10.0;
	ycmm = y*10.0;
	zcmm = z*10.0;
	
	zcmm = -zcmm + 2*zmin_trans*10.0;
	xsol=xcmm-zcmm*sin(fThetaLorentz)*sin(fThetaRot);
	ysol=ycmm+zcmm*sin(fThetaLorentz)*cos(fThetaRot);
	zsol=zcmm;
	
	xdet = xsol;
	ydet = -(fZk-zsol)*sin(fThetaTilt) + ysol*cos(fThetaTilt);
	zdet = zsol*cos(fThetaTilt) - ysol*sin(fThetaTilt);
	
	xpad = xdet*cos(fThetaPad) - ydet*sin(fThetaPad);
	ypad = xdet*sin(fThetaPad) + ydet*cos(fThetaPad);
	zpad = zdet;

	iterCorr = (Int_t) (zpad/(dzstep*10) + 0.5);
	if(k==0) iterCorr_0 = iterCorr; //Offset renomarlization
	//std::cout<<" iterCorr : "<<iterCorr_0-iterCorr<<" iterd : "<<iterd<<std::endl;
	
	iterCorrNorm = iterCorr_0-iterCorr;
	if(iterCorrNorm<0) break;
	
	xTBCorr[iterCorrNorm] = xpad;
	yTBCorr[iterCorrNorm] = ypad;
	zTBCorr[iterCorrNorm] = zpad;
	
	//  if(iterCorrNorm!=icnb){
	//	xiter.push_back(xTBCorr[iterCorrNorm]);
	//yiter.push_back(yTBCorr[iterCorrNorm]);
	//ziter.push_back(zTBCorr[iterCorrNorm]);
	//    }
	icnb=iterCorrNorm;

	t=t+dt;
	Double_t ddxddt=esm*B[idx]*10.*dydt*factq;//  !remember esm =charge/masse
	Double_t ddyddt=-esm*(B[idx]*10.*dxdt)*factq;
	Double_t ddzddt=0.;
	x=x + dxdt*dt + 0.5*ddxddt*pow(dt,2.0);
	y=y + dydt*dt + 0.5*ddyddt*pow(dt,2.0);
	z=z + dzdt*dt + 0.5*ddzddt*pow(dt,2.0);
	dxdt=dxdt+ddxddt*dt;
	dydt=dydt+ddyddt*dt;
	dzdt=dzdt+ddzddt*dt;

	Double_t help=pow((dxdt*dt + 0.5*ddxddt*pow(dt,2.0)),2.0 );
	help=help+pow((dydt*dt + 0.5*ddyddt*pow(dt,2.0)),2.0);
	help=help+pow((dzdt*dt + 0.5*ddzddt*pow(dt,2.0)),2.0);
	help=sqrt(help);
	range=range+help;
	//std::cout<<" Range : "<<range<<std::endl;
	Double_t sloss = 0.0;
	
	Double_t  c0;
	 
	if(iz1==1){
	  c0=ekin;
	  if(m==2) c0=c0/2.0;
	  sloss=6.98*(1./pow(c0,0.83))*(1./(20.+1.6/pow(c0,1.3)))+0.2*exp(-30.*pow((c0-0.1),2.0)); //Old expression
	  //sloss = 0.3*TMath::Power((1./c0),0.78)*(1./(1.+0.023/TMath::Power(c0,1.37)));
	}
	if(iz1==6){
	  c0=ekin/6.;
	  sloss=36.*(1./pow(c0,0.83))*(1./(1.6+1.6/pow(c0,1.5)))+1.*exp(pow(-(c0-0.5),2.0));
	}
	if(iz1==2){
	  c0=ekin;
	  sloss=11.95*(1./pow(c0,0.83))*(1./(2.5+1.6/pow(c0,1.5)))+ 0.05*exp(pow(-(c0-0.5),2.0));
	}
	
	sloss= sloss*dens*help; //!energy loss with density and step
	Double_t vcin=sqrt(pow(dxdt,2.0)+pow(dydt,2.0)+pow(dzdt,2.0));// !v in cm/ns
	Double_t vsc=vcin/29.979; // !v/c
	Double_t beta=vsc;
	//ecinsm=931.494/2.*(vsc)**2*1./sqrt(1.-vsc**2) !relativistic
	Double_t ekindo=sm1*931.494*0.5*pow(vsc,2.0); //!nonrelativistic
	help=ekin;
	ekin=ekin-sloss;// !energy loss
	//help=ekin/help
	help=ekin/ekindo;
	help=sqrt(help);
	Double_t help1=help;
	ekindo=ekindo-sloss;
	dxdt=dxdt*help; // ! this is the introduction energy loss!!!!
	dydt=dydt*help;
	dzdt=dzdt*help;
	//dt=dzstep/(dzdt);
	dt=dzstep/(dzdt)/(Float_t) integrationsteps;//NEW
	//std::cout<<" z :"<<z<<" dt : "<<dt<<" dzstep : "<<dzstep<<" dzdt : "<<dzdt<<std::endl;
	//test if still in detector supposing radius of 25cm and lenght 100cm
	Double_t radp2=pow(x,2.0)+pow(y,2.0);
	Double_t radp=sqrt(radp2);
	//std::cout<<cRED<<" dt : "<<dt<<cNORMAL<<std::endl;
	
	//  if(radp>25.0) break;//  !this limits the radial distance of the trajectories taken into ccount
	//	if(z.gt.ztot) go to 100
	if(zTBCorr[iterCorrNorm]<0.0) break;
                                                                //std::cout<<" Ekin : "<<ekin<<std::endl;
	if(ekin<0.01 || isnan(ekin)) break; 
	//if(ekin<0) std::cout<<" Ekin "<<std::endl;	
	
      } // k loop spiral integration
    tm=tm+t;
    Int_t iterh = (Int_t)(iteration/integrationsteps);
    
    Int_t imaxchi2=max(iterCorrNorm,(Int_t) parameter[7]); //NEW
    //if(true){  
  
    if(idx==0){
      printf(cRED " Imaxchi2 : %d \n",imaxchi2);//<<std::endl;
      printf(" iterCorrNorm : %d\n", iterCorrNorm);//<<std::endl;
      printf(" iterh : %d\n",iterh);//<<std::endl;
      printf(" parameter[7] : %f\n" cNORMAL,parameter[7]);//<<std::endl;
      //printf(" Num of interpolated exp points : %d" cNORMAL,numIntPoints);//<<cNORMAL<<std::endl;
    }
      //std::cout<<" iterCorrNorm : "<<iterCorrNorm<<std::endl;
      //std::cout<<" iterh : "<<iterh<<std::endl;
      //std::cout<<" parameter[7] : "<<parameter[7]<<std::endl;
      //std::cout<<" Num of interpolated exp points : "<<numIntPoints<<cNORMAL<<std::endl;
      //}
  }
}

MCMinimization::MCMinimization()
{
  setGlobalGPUVariables<<<1,1>>>();
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
  cudaSetDevice(0);
  cudaDeviceSynchronize();
  cudaThreadSynchronize();
  curandState* d_state;
  Double_t* d_params;
  //int nThreads = 400;
  int nThreads = 400;
  int nBlocks = 1;
  cudaMalloc(&d_state,nThreads*nBlocks);
  cudaMalloc(&d_params,sizeof(Double_t)*8);
  cudaMemcpy(d_params,parameter,sizeof(Double_t)*8,cudaMemcpyHostToDevice);
  init_stuff<<<nBlocks,nThreads>>>(d_state);
  //make_rand<<<nblocks,nthreads>>>(d_state,randArray);
  setInitialParameters<<<1,1>>>(d_params);
  gpuErrchk(cudaPeekAtLastError());
  gpuErrchk(cudaDeviceSynchronize());
  //std::cout << "Initial parameters set\n";
  /////////// Initial parameters///////////////
  // Double_t xmin= parameter[0]/10.0; //  ! at ztb=394 in cm
  // Double_t ymin= parameter[1]/10.0;
  // Double_t zmin= fZk/10.0 - (fEntTB-parameter[3])*dzstep;  //Micromegas Origin  at 100 cm of the entrance
  // Double_t TBmin = parameter[3]*dzstep;
  // Double_t phi0=TMath::Pi()-parameter[4]-115*TMath::Pi()/180.0;
  
  // Double_t bro=parameter[5]*B0/1000.0;// !Tm*/
  // Double_t theta0=parameter[6];
  // ////////////////////////////////////////////
  
  // Double_t phimin=phi0;
  // Double_t thetamin=theta0;
  // Double_t bromin=bro;
  // Double_t brotheta=bromin/TMath::Sin(thetamin);
  // Double_t tm=0.0;
  // Int_t iteration=0;
  
  kVerbose = kTRUE;
  kDebug   = kFALSE;
  
  for(int i=0;i<5;i++){
    calcInner<<<nBlocks,nThreads>>>(0,d_state,d_params);
    gpuErrchk( cudaPeekAtLastError() );
    gpuErrchk(cudaDeviceSynchronize());
  }
  cudaFree(d_state);
  cudaFree(d_params);
}

std::vector<ATHit> MCMinimization::GetTBHitArray(Int_t TB,std::vector<ATHit> *harray)
{

     
}

/*void MCMinimization::GetEnergy(Double_t M,Double_t IZ,Double_t BRO,Double_t &E){


  }*/
TVector3 MCMinimization::TransformIniPos(Double_t x,Double_t y, Double_t z)
{
}

TVector3 MCMinimization::InvTransIniPos(Double_t x,Double_t y, Double_t z)
{
}
void MCMinimization::BackwardExtrapolation()
{
}
