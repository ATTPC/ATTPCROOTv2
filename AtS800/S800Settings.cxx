#include "S800Settings.h"

#include <TEnv.h>
#include <TString.h>

S800Settings::S800Settings() {}

S800Settings::S800Settings(const char *filename)
{
   SetFile(filename);
   ReadSettings();
}

S800Settings::~S800Settings() {}

void S800Settings::ReadSettings()
{
   char *defaultfile = (char *)"~/analysis/settings/nocal.dat";

   TEnv *set = new TEnv(fInputFile.data());
   fCalFile = set->GetValue("Crdc.File", defaultfile);
   fPedestalFile = set->GetValue("Crdc.Ped.File", defaultfile);
   for (int i = 0; i < 2; i++) {
      fXFit = set->GetValue("Crdc.X.Fit", 0);
      fXFitFunc = set->GetValue("Crdc.X.FitFunc", 1);
      fxOffset[i] = set->GetValue(Form("Crdc.X.Offset.%d", i), 0.0);
      fxSlope[i] = set->GetValue(Form("Crdc.X.Slope.%d", i), 1.0);
      fyOffset[i] = set->GetValue(Form("Crdc.Y.Offset.%d", i), 0.0);
      fySlope[i] = set->GetValue(Form("Crdc.Y.Slope.%d", i), 1.0);
   }
   fBadFile = set->GetValue("BadPad.File", defaultfile);
}

// void Settings::PrintSettings(){
//   std::cout << "Crdc.File\t"     << fCalFile      << std::endl;
//   std::cout << "Crdc.Ped.File\t" << fPedestalFile << std::endl;
//   std::cout << "Crdc.X.Fit\t"    << fXFit         << std::endl;
//   for(int i=0;i<2;i++){
//     std::cout << Form("Crdc.X.Offset.%d\t",i) << fxOffset[i] << std::endl;
//     std::cout << Form("Crdc.X.Slope.%d\t",i)  << fxSlope[i]  << std::endl;
//     std::cout << Form("Crdc.Y.Offset.%d\t",i) << fyOffset[i] << std::endl;
//     std::cout << Form("Crdc.Y.Slope.%d\t",i)  << fySlope[i]  << std::endl;
//   }
//   std::cout << "BadPad.File\t"         << fBadFile    << std::endl;
// }
