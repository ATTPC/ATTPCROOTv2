// This class header
#include "AtGas.h"

#include <cstdlib>
#include <fstream> // IWYU pragma: keep
#include <iostream>
#include <sstream> // IWYU pragma: keep
#include <string>
#include <utility>

// ROOT class headers
#include <Rtypes.h>
#include <TRandom.h>
#include <TString.h>

using namespace std;

ClassImp(AtGas)

   AtGas::AtGas(TString GasFileName)
   : fGasFileName(std::move(GasFileName))
{
   InitializeParameters();
}

void AtGas::InitializeParameters()
{
   ifstream gasFile(fGasFileName.Data(), std::fstream::in);
   if (!gasFile)
      cerr << "Gas file " << fGasFileName.Data() << " not found!!" << endl;

   string line;
   string data;
   string name;
   string format;
   string val;

   while (getline(gasFile, line)) {
      istringstream ss_line(line);
      if (ss_line >> data >> val && data[0] != '#' && data[0] != '[') {
         name = data.substr(0, data.find(":"));
         // name   = strtok(data.c_str(),":");
         // format = strtok(NULL,":");
         if (name == "EIonizeP10")
            fEIonize = atof(val.c_str());
         if (name == "DriftVelocity")
            fDriftVelocity = atof(val.c_str());
         if (name == "CoefAttachment")
            fCoefAttachment = atof(val.c_str());
         if (name == "CoefDiffusionLong")
            fCoefDiffusionLong = atof(val.c_str());
         if (name == "CoefDiffusionTrans")
            fCoefDiffusionTrans = atof(val.c_str());
         if (name == "Gain")
            fGain = atoi(val.c_str());
      }
   }
}

void AtGas::operator=(const AtGas &GasToCopy)
{
   fEIonize = GasToCopy.fEIonize;
   fDriftVelocity = GasToCopy.fDriftVelocity;
   fCoefAttachment = GasToCopy.fCoefAttachment;
   fCoefDiffusionLong = GasToCopy.fCoefDiffusionLong;
   fCoefDiffusionTrans = GasToCopy.fCoefDiffusionTrans;
   fGain = GasToCopy.fGain;
}

Double_t AtGas::GetEIonize()
{
   return fEIonize;
}
Double_t AtGas::GetDriftVelocity()
{
   return fDriftVelocity;
}
Double_t AtGas::GetCoefAttachment()
{
   return fCoefAttachment;
}
Double_t AtGas::GetCoefDiffusionLong()
{
   return fCoefDiffusionLong;
}
Double_t AtGas::GetCoefDiffusionTrans()
{
   return fCoefDiffusionTrans;
}
Int_t AtGas::GetGain()
{
   return fGain;
}
UInt_t AtGas::GetRandomCS()
{
   auto CS = (UInt_t)(gRandom->Gaus(50, 20));
   if (CS <= 0)
      CS = 1;
   return CS;
}
