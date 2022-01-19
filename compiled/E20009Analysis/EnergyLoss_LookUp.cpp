///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////Code: EnergyLoss_LookUp.cpp//////////////////////////////////
/////////////////////////////Date: November 2021///////////////////////////////////////
////////////////////////////Author: Nabin Rijal ///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////// Standalone Usage or to use shared object (.so) //////////////////////
//////////////////////  : root -l                     /////////////////////////////////
/////////////////////   :[].L EnergyLoss_LookUp.cpp++  ////////////////////////////////
////////// It creates EnergyLoss_LookUp.cpp.so file,   ////////////////////////////////
///////  which will be used in analysis script such as e20009Ana.cc ///////////////////
///////////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <string.h>
#include <TGraph.h> 

#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <cmath>

#include <string>
#include <math.h>   
#include <TRandom3.h>
#include <TPolyLine3D.h>
#include <TPolyMarker3D.h>
#include <TCanvas.h>
#include <TMath.h> 
#include <TROOT.h> 
#include <vector> 
#include <TLorentzVector.h>
#include <TVector3.h>
#include <TTree.h>

#include "EnergyLoss_LookUp.h"
using namespace std;
///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// Energy Loss Class  /////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// LINEAR INTERPOLATION //////////////////////////
// Get the energy loss of the gas for an initial ion energy and a certain distance through the target.

Double_t LookUp::GetEnergyLossLinear(Double_t energy /*MeV*/, Double_t distance ){
  int i = -1;
  if(energy >= 0.01){
    // Look for two points for which the initial energy lays in between.  This for-loop should find the points 
    //unless there was a big jump from the energy used in the last point and the energy used now.
    
    for(int p=0; p<points-1; p++){
      
      if(energy>=IonEnergy[p]  && energy<IonEnergy[p+1]){
	
	i = p+1;
	last_point = p;
	
	break;
      }
    }
    
    // If after this two loop i is still -1 it means the energy was out of range.
    if(i==-1){
      //cout << "*** EnergyLoss Error: energy not within range: " << energy << endl;
      Energy_in_range = 0;
      return 0;
    }
    
    // If the initial energy is within the range of the function, 
    //get the stopping power for the initial energy.  
    
    Double_t E1 = IonEnergy[i-1];  
    Double_t E2 = IonEnergy[i];
    
    Double_t dEdx_e1 = dEdx_e[i-1];     
    Double_t dEdx_e2 = dEdx_e[i];  
    
    Double_t dEdx_n1 = dEdx_n[i-1];     
    Double_t dEdx_n2 = dEdx_n[i]; 
    
    // Interpolating the electric stopping power (from point 1 to 'e').
    Double_t dEdx_e1e = dEdx_e1 + (energy - E1)*(dEdx_e2 - dEdx_e1)/(E2 - E1);
    
    // Interpolating the nuclear stopping power (usually negligable).
    Double_t dEdx_n1e = dEdx_n1 + (energy - E1)*(dEdx_n2 - dEdx_n1)/(E2 - E1);
    
    // The stopping power units are in MeV/mm so we multiply by 10 to convert to MeV/cm.
    return ( (dEdx_e1e+dEdx_n1e)*10*distance ); 
  } //end of if(energy > 0.1){
  else{
    return 0;
  }
  
}
 /////////////////////////////////// SPLINE INTERPOLATION ////////////////////////////////////////////
double LookUp::GetEnergyLoss(double energy /*MeV*/, double distance /*cm*/)
{
  Float_t a11=0.0, a12=0.0, a21=0.0, a22=0.0, a23=0.0, a32=0.0, a33=0.0;
  Float_t b11=0.0, b22=0.0, b33=0.0;
  Float_t a1=0.0, a2=0.0, b1=0.0, b2=0.0;
  Float_t K0=0.0, K1=0.0, K2=0.0; 
  Float_t N1=0.0, N2=0.0, N3=0.0;
  Float_t T1=0.0,T2=0.0,q1=0.0,q2=0.0;
  
  int i = -1;
  if(energy < 0.01)
    return(0);
  
  //if(energy < 0.01){
  //break;
  //}
  
  //Look for two points for which the initial energy lies in between.  
  //This for-loop should find the points 
  //unless there was a big jump from the energy used in the 
  //last point and the energy used now.
  
  for(int p=0; p<points-1; p++){
    
    if(energy>=IonEnergy[p]  && energy<IonEnergy[p+1]){
      
      i = p+1;
      last_point = p;
      
      break;
    }
  } 
  // If after this two loop i is still -1 it means the energy was out of range.
  
  if(i==-1){
    
    cout << "*** EnergyLoss Error: energy not within range: " << energy << endl;
    
    Energy_in_range = 0;
    
    return 0;
  } 
  
  //Ion Energy
  double x0=IonEnergy[i-1];
  double x1=IonEnergy[i];
  double x2=IonEnergy[i+1];
  
    //Total Energy Loss (electric + nuclear) for one step
  double y0=dEdx_e[i-1]+dEdx_n[i-1];
  double y1=dEdx_e[i]+dEdx_n[i];
  double y2=dEdx_e[i+1]+dEdx_n[i+1];
  
  a11=2/(x1-x0);
  a12=1/(x1-x0);
  a21=1/(x1-x0);
  a22=2*((1/(x1-x0))+(1/(x2-x1)));
  a23=1/(x2-x1);
  a32=1/(x2-x1);
  a33=2/(x2-x1);
  
  b11=3*((y1-y0)/((x1-x0)*(x1-x0)));
  b22=3*(((y1-y0)/((x1-x0)*(x1-x0)))+((y2-y1)/((x2-x1)*(x2-x1))));
  b33=3*((y2-y1)/((x2-x1)*(x2-x1)));  
  
  //mathematical terms to calculate curvatures.
  N1=(a21*a33*a12-a11*(a22*a33-a23*a32))/(a33*a12);
  N2=(b22*a33-a23*b33)/a33;
  N3=b11*(a22*a33-a23*a32)/(a33*a12);
  //cout<<"N1="<<N1<<" N2="<<N2<<" N3="<<N3<<endl;
  
  //curvatures
  K0=(N2-N3)/N1;
  K1=(b11-a11*K0)/a12;
  K2=(b33-a32*K1)/a33;
  // cout<<"K0="<<K0<<" K1="<<K1<<" K2="<<K2<<endl;
  
  a1=K0*(x1-x0)-(y1-y0);  
  b1=-K1*(x1-x0)+(y1-y0);  
  a2=K1*(x2-x1)-(y2-y1);  
  b2=-K2*(x2-x1)+(y2-y1);  
  //cout<<"a1="<<a1<<" b1="<<b1<<" a2="<<a2<<" b2="<<b2<<endl;
  
  T1=(energy-x0)/(x1-x0);
  T2=(energy-x1)/(x2-x1);
  
  //polynomials //which gives the value of energy loss for given energy.
  q1=(1-T1)*y0+T1*y1+T1*(1-T1)*(a1*(1-T1)+b1*T1);
  q2=(1-T2)*y1+T2*y2+T2*(1-T2)*(a2*(1-T2)+b2*T2);
  
  //cout<<"q1="<<q1<<" q2="<<q2<<endl; 
  
  return (q1*10*distance);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
Double_t LookUp::GetInitialEnergy(Double_t FinalEnergy /*MeV*/, Double_t PathLength /*cm*//*dist*/, Double_t StepSize/*cm*/)
{
  Double_t Energy = FinalEnergy;
  int Steps = (int)floor(PathLength/StepSize);
  last_point = 0;

  // The function starts by assuming FinalEnergy is within the energy range, 
  //but this could be changed in the GetEnergyLoss() function.

  Energy_in_range = 1;

  for (int s=0; s<Steps; s++) {

    Energy = Energy + GetEnergyLoss(Energy,PathLength/Steps);

    if (!Energy_in_range){
      break;
    } 
  }
  Energy = Energy + GetEnergyLoss(Energy,PathLength-Steps*StepSize);

  if (!Energy_in_range)
    Energy = -1000; // Return an unrealistic value. 

  return Energy;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
Double_t LookUp::GetFinalEnergy(Double_t InitialEnergy /*MeV*/, Double_t PathLength /*cm*/, Double_t StepSize/*cm*/)
{


  Double_t Energy = InitialEnergy;
  int Steps = (int)floor(PathLength/StepSize);

  // The function starts by assuming InitialEnergy is within the energy range, but
  // this could be changes in the GetEnergyLoss() function.

  Energy_in_range = 1;

  for (int s=0; s<Steps; s++) {

    Energy = Energy - GetEnergyLoss(Energy,PathLength/Steps);

    if (!Energy_in_range){
      break;
    }
    //cout<<"1: ="<<Energy<< "  "<<s*StepSize<<endl;
  }  

  Energy = Energy - GetEnergyLoss(Energy,PathLength-Steps*StepSize);

  if (!Energy_in_range){
    Energy = -1000;
  }
  
  //  if (PathLength == 6.0){
  //  cout << " Get Final Energy :"<< InitialEnergy << " " << PathLength << " " << Energy << endl;
  //}
  return Energy;

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Double_t LookUp::GetDistance(Double_t InitialE, Double_t FinalE, Double_t StepSize, int MaxSteps)
Double_t LookUp::GetDistance(Double_t InitialE, Double_t FinalE, Double_t StepSize)
{
  
  Double_t dist = 0;
  Double_t E = 0, Elast=0;
  //Double_t  Tolerance = 0.01;

  E = InitialE;
  
  
  while(E>FinalE){
    dist += StepSize;
    Elast=E;
    E = E - GetEnergyLoss(E,StepSize);
  }

  return ((dist-StepSize)-(StepSize*(Elast-FinalE)/(E-Elast)));
}
////////////////////////////////////////////////////////////////////////////////////////
// Calulates the ion's path length in cm.
////////////////////////////////////////////////////////////////////////////////////////
Double_t LookUp::GetPathLength(Float_t InitialEnergy /*MeV*/, Float_t FinalEnergy /*MeV*/, Float_t DeltaT /*ns*/)
{
  Double_t L = 0, DeltaX = 0;
  Double_t Kn = InitialEnergy;
  Double_t Kn1 = InitialEnergy;
  Int_t n=0;


  if (IonMass==0)
    cout << "*** EnergyLoss Error: Path length cannot be calculated for IonMass = 0." << endl;
  else {

    //The path length (L) is proportional to sqrt(Kn). 
    //After the sum, L will be multiplied by the proportionality factor.

    while (Kn > FinalEnergy && Kn1 > FinalEnergy && n < (int)pow(10.0,6)){

      //L += sqrt(Kn); // 2016-06-15 changed
      L += sqrt((Kn+Kn1)/2)*sqrt(2/IonMass)*DeltaT*c;  // DeltaL going from point n to n+1.
      //cout<<"1 = "<<Kn<<"  "<<Kn1<<endl;
      
      ////DeltaX = sqrt(2*Kn/IonMass)*DeltaT*c; // dx = v*dt// 2016-06-15 changed
      DeltaX = sqrt((Kn+Kn1)/IonMass)*DeltaT*c;

      Kn1 = Kn;
      //Kn -= GetEnergyLoss(Kn,DeltaX); // 2016-06-15 changed
      Kn -= GetEnergyLoss((Kn+Kn1)/2, DeltaX);     // After L is incremented the kinetic energy at n+1 is calculated.
      //cout<<"2 = "<<Kn<<"  "<<Kn1<<endl;
      //outfile4 << Kn << "\t" << L << endl;
      n++;    
    }
    if (n>=(int)pow(10.0,6)) {
      cout << "*** EnergyLoss Warning: Full path length wasn't reached after 10^6 iterations." << endl;
      L = 0;
    }
    else
      //L *= sqrt(2/IonMass)*DeltaT*c;
      L *= 1.0;
  }
  return L;
}
///////////////////////////////////////////////////////////////////////////////////////
Double_t LookUp::LoadRange(Float_t energy1)
{
  Int_t i = -1;
  Double_t Range1=0;

  if(energy1 >= 0.01){ //greater than= 10 keV

    for(Int_t p=last_point1-1; p<points1-1; p++){
      if (last_point1 >=0)
	if(energy1>=IonEnergy[p]  && energy1<IonEnergy[p+1]){
	  i = p+1;
	  last_point1 = p;
	  break;
	}
    }

    if (i==-1) {
      for(Int_t p=0; p<last_point1-1; p++){
	if(energy1>=IonEnergy[p]  && energy1<IonEnergy[p+1]){
	  i = p+1;
	  last_point1 = p;
	  break;
	}
      }
    }
 
    if(i==-1){
      cout << "*** EnergyLoss Error: energy not within range: " << energy1 << endl;
      Energy_in_range = 0;
      return 0;
    }   
    Range1= Range[i];
    return Range1;
  }
  return(0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Calulates the ion's time of flight in ns.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

Double_t LookUp::GetTimeOfFlight(Double_t InitialEnergy, Double_t PathLength, Double_t StepSize)
{
  Double_t TOF = 0;
  Double_t Kn = InitialEnergy;
  int Steps = (int)(PathLength/StepSize);
  
  if (IonMass==0) {
    cout << "Error: Time of flight cannot be calculated because mass is zero." << endl;
  }

  else {

    for (int n=0; n<Steps; n++) {

      TOF += sqrt(IonMass/(2*Kn))*StepSize/c;               // DeltaT going from point n to n+1.
      Kn -= GetEnergyLoss(Kn, StepSize); //After the TOF is added the K.E. at point n+1 is calc.} 
    }
    return TOF;
  }
  return(0);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LookUp::SetIonMass(Double_t IonMass)
{
  this->IonMass = IonMass;
  return;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Lookup Table Extension 
void LookUp::InitializeLookupTables(Double_t MaximumEnergy, Double_t MaximumDistance, 
				    Double_t DeltaE, Double_t DeltaD){

  int noE = (int)ceil(MaximumEnergy / DeltaE );
  int noD = (int)ceil(MaximumDistance / DeltaD );

  this->MaximumEnergy = MaximumEnergy;
  this->MaximumDistance = MaximumDistance;
  this->DeltaD = DeltaD;
  this->DeltaE = DeltaE; 
  
  if (!(EtoDtab = new Double_t[noE])||
      !(DtoEtab = new Double_t[noD])){
    cerr << "Could not allocate memory for " << noE << " " << noD << endl;
  }

  //Double_t D;
  int i;
  //-----------------------------------------------------------
  DtoEtab[0] = MaximumEnergy;
  cout << " Number of distance entries " << noD << endl;
  
  for (i=1; i<noD; i++){
    //DtoEtab[i] = GetFinalEnergy(InitialEnergy,D,50000);
    
    //DtoEtab[i] = GetFinalEnergy(DtoEtab[i-1],DeltaD,0.05*DeltaE);
    DtoEtab[i] = GetFinalEnergy(DtoEtab[i-1],DeltaD,0.05*DeltaD);
    
    //cout<<"2: "<<DtoEtab[i]<< " D="<<D<<endl;
    if (i%1000 == 0 ){
      //cout << " Passed  " << i << endl;
    }
  }
  
  //Double_t E;
  int j;
  
  cout << " Number of Energy entries " << noE << endl;
  
  EtoDtab[0] = 0.;
  for (j=1;j<noE;j++){
    //EtoDtab[j] = GetDistance(MaximumEnergy,E,50000);
    //EtoDtab[j] = GetDistance(MaximumEnergy,MaximumEnergy-j*DeltaE,0.1,5500);
    EtoDtab[j] = EtoDtab[j-1] + GetDistance((MaximumEnergy-(j-1)*DeltaE),
					    (MaximumEnergy-(j)*DeltaE),
					    (0.05*DeltaD)); 
    if (j%1000 == 0 ){
      //cout << " Passed  " << j << endl;
    }
    
  }
  //cout << " Passed  " << j << endl;
  //-----------------------------------------------------------

  /*
    for (i=0; i<noD; i++){
    //DtoEtab[i] = GetFinalEnergy(InitialEnergy,D,50000);
    DtoEtab[i] = GetFinalEnergy(MaximumEnergy,DeltaD*i,0.01);
    //cout<<"2: "<<DtoEtab[i]<< " D="<<D<<endl;
    }
    
    //Double_t E;
    int j;
    
    for (j=0;j<noE;j++){
    //EtoDtab[j] = GetDistance(MaximumEnergy,E,50000);
    //EtoDtab[j] = GetDistance(MaximumEnergy,MaximumEnergy-j*DeltaE,0.1,5500);
    EtoDtab[j] = GetDistance(MaximumEnergy,MaximumEnergy-j*DeltaE,0.1);
    }  
  */
  
}
//=======================================================
void LookUp::PrintLookupTables(){
  int noE = (int)ceil(MaximumEnergy / DeltaE );
  int noD = (int)ceil(MaximumDistance / DeltaD );
  int i;
  cout << "Maximum Energy = " << MaximumEnergy << " " << DeltaE << noE << endl;
  for (i=0;i<noE;i++){
    cout << "E,D= "<< MaximumEnergy - DeltaE*i << "," <<EtoDtab[i] <<endl; 
  }
  cout << "Maximum Distance = "<< MaximumDistance << " " << DeltaD << noD << endl;
  for (i=0;i<noD;i++){
    cout << "D,E= "<< DeltaE*i << "," <<DtoEtab[i]<<endl; 
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Double_t LookUp::GetLookupEnergy(Double_t InitialEnergy, Double_t distance){
Double_t LookUp::GetLookupEnergy(Double_t InitialEnergy, Double_t distance){

  Double_t D1,D2,D;
  Double_t E1,E2,E;
  int index;
  int imhere;

  if (InitialEnergy<0 || InitialEnergy>MaximumEnergy){
    return(-1.);
  }

  // Find the distance for which the initial energy is matched, interpolating 
  index = (int)floor((MaximumEnergy-InitialEnergy) / DeltaE) ;

  D1=EtoDtab[index];D2=EtoDtab[index+1];

  E1=MaximumEnergy-index*DeltaE; E2=MaximumEnergy-(index+1)*DeltaE; 

  D = ( InitialEnergy - E1 ) / ( E2 - E1 ) * (D2-D1) + D1 ;
  //cout << " 1:  "<< index << ' ' <<E1 << ' ' << E2 << ' ' << D1 << ' ' << D2 << ' ' << D << endl;


  //Still in the table ?
  if(((D+distance <=0)) || ((D+distance)> MaximumDistance)){
    //if (((D+distance)> MaximumDistance)){
    //if (D+distance <=0){
    cout<<"i m here"<<endl;
    //imhere++;
    return(0.);
  }

  // Lookup what energy is reached for (D + distance) and interpolate
  index = (int)floor((D + distance) / DeltaD );

  E1=DtoEtab[index];E2=DtoEtab[index+1];

  D1=index*DeltaD;D2=(index+1)*DeltaD;

  E = ( (D+distance) - D1 ) / ( D2 - D1 ) * (E2-E1) + E1 ;
  //cout <<" 2:   "<< E1 << ' ' << E2 << ' ' << D1 << ' ' << D2 << ' ' << E << endl;

  //cout<<"E == "<<E<<endl;
  //cout<<imhere<<endl;
  return(E);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Analyzer::Analyzer(string FileELoss1,string FileELoss2,string FileELoss3){
		   
  
  ConvUtoMeV = 931.494061;

  //initializing mass of atoms in amu
  mB1=0, mB2=0, mB3=0;
  //mass of atoms in MeV
  MB1=0, MB2=0, MB3=0;

  //pointers
  IonInGas1 = new LookUp(FileELoss1,MB1); 
  // cout << "loaded beam ion1 in gas"<<endl ; 
  IonInGas2 = new LookUp(FileELoss2,MB2); 
  // cout << "loaded beam ion2 in gas"<<endl ; 
  IonInGas3 = new LookUp(FileELoss3,MB3);
  // cout << "loaded beam ion3 in gas"<<endl ; 
  

  //Length of the AT-TPC
  La=100.0;//cm  
}
//-------------------------------------
Analyzer::~Analyzer(){  //Destructor:
  //---
}

//-------------------------------------
bool Analyzer::SetMasses(Double_t mB,Double_t mB2,Double_t mB3){

  this->mB1 = mB1;
  this->mB2 = mB2;
  this->mB3 = mB3;
 
  MB1 = mB1*ConvUtoMeV;
  MB2 = mB2*ConvUtoMeV;
  MB3 = mB3*ConvUtoMeV;

  IonInGas1->SetIonMass(MB1);
  IonInGas2->SetIonMass(MB2);
  IonInGas3->SetIonMass(MB3); 

  return 1;
}
//-------------------------------------
