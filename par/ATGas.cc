// This class header
#include "ATGas.hh"

// ROOT class headers
#include "TRandom.h"

// C/C++ class headers
#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <iostream>

using namespace std;

ClassImp(ATGas)

ATGas::ATGas(TString GasFileName)
: fGasFileName(GasFileName)
{
  InitializeParameters();
}

void ATGas::InitializeParameters()
{
  ifstream gasFile(fGasFileName.Data(), std::fstream::in);
  if(!gasFile) cerr << "Gas file " << fGasFileName.Data() << " not found!!" << endl;

  string line;
  string data;
  string name;
  string format;
  string val;

  while(getline(gasFile,line)) {
    istringstream ss_line(line);
    if(ss_line >> data >> val && data[0]!='#' && data[0]!='[') {
      name   = data.substr(0,data.find(":"));
      //name   = strtok(data.c_str(),":");
      //format = strtok(NULL,":");
      if(name=="EIonizeP10")         fEIonize            = atof(val.c_str());
      if(name=="DriftVelocity")      fDriftVelocity      = atof(val.c_str());
      if(name=="CoefAttachment")     fCoefAttachment     = atof(val.c_str());
      if(name=="CoefDiffusionLong")  fCoefDiffusionLong  = atof(val.c_str());
      if(name=="CoefDiffusionTrans") fCoefDiffusionTrans = atof(val.c_str());
      if(name=="Gain")               fGain               = atoi(val.c_str());
    }
  }
}

ATGas::~ATGas()
{
}

void ATGas::operator=(const ATGas& GasToCopy) 
{ 
  fEIonize            = GasToCopy.fEIonize; 
  fDriftVelocity      = GasToCopy.fDriftVelocity; 
  fCoefAttachment     = GasToCopy.fCoefAttachment; 
  fCoefDiffusionLong  = GasToCopy.fCoefDiffusionLong; 
  fCoefDiffusionTrans = GasToCopy.fCoefDiffusionTrans; 
  fGain               = GasToCopy.fGain; 
}

Double_t ATGas::GetEIonize()            { return fEIonize; }
Double_t ATGas::GetDriftVelocity()      { return fDriftVelocity; }
Double_t ATGas::GetCoefAttachment()     { return fCoefAttachment; }
Double_t ATGas::GetCoefDiffusionLong()  { return fCoefDiffusionLong; }
Double_t ATGas::GetCoefDiffusionTrans() { return fCoefDiffusionTrans; }
   Int_t ATGas::GetGain()               { return fGain; }
  UInt_t ATGas::GetRandomCS()           
{
  UInt_t CS = (UInt_t)(gRandom -> Gaus(50,20));
  if(CS<=0) CS=1;
  return CS;
}
