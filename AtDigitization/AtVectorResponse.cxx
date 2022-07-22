#include "AtVectorResponse.h"

#include "AtDigiPar.h"

#include <FairRunAna.h>
#include <FairRuntimeDb.h>
#include <FairTask.h>

#include <Rtypes.h>
#include <TString.h>

Double_t AtVectorResponse::fPeakingTime = 0;
Double_t AtVectorResponse::fTBTime = 0;
std::vector<Double_t> AtVectorResponse::fWaveSample = {};

AtVectorResponse::AtVectorResponse() {}

double AtVectorResponse::ResponseFunction(double reducedTime)
{
   double response = fWaveSample[std::floor(reducedTime * fPeakingTime / fTBTime)];
   // std::cout << "response: " << response << std::endl;
   return response;
}

void AtVectorResponse::SetFile(TString filename)
{
   std::ifstream input(filename);
   if (!input.is_open())
      std::cout << "wave input not open" << std::endl;
   if (input.is_open()) {
      std::cout << "wave input is open" << std::endl;
      while (!input.eof()) {
         double value = 0;
         input >> value;
         fWaveSample.push_back(value);
      }
   }
   input.close();
   std::cout << "fWaveSample size: " << fWaveSample.size() << std::endl;
   for (int i = 0; i < fWaveSample.size(); i++) {
      std::cout << "fWaveSample[" << i << "]: " << fWaveSample[i] << std::endl;
   }
   setParameters();
   std::cout << "fPeakingTime: " << fPeakingTime << "; fTBTime: " << fTBTime << std::endl;
}

void AtVectorResponse::setParameters()
{
   auto fPar = dynamic_cast<AtDigiPar *>(FairRunAna::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));
   fPeakingTime = fPar->GetPeakingTime() / 1000.;
   fTBTime = fPar->GetTBTime() / 1000.; // in us
}
