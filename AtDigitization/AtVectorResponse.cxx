#include "AtVectorResponse.h"

#include "AtDigiPar.h"
#include "AtElectronicResponse.h"

#include <FairLogger.h>
#include <FairParSet.h> // for FairParSet
#include <FairRunAna.h>
#include <FairRuntimeDb.h>

#include <string>

double AtVectorResponse::fPeakingTime = 0;
double AtVectorResponse::fTBTime = 0;
std::unique_ptr<ElectronicResponse::AtFileResponse> AtVectorResponse::fResponse = nullptr;

double AtVectorResponse::ResponseFunction(double reducedTime)
{
   return (*fResponse)(reducedTime * fPeakingTime + 0.5 * fTBTime);
}

void AtVectorResponse::SetFile(TString filename)
{
   setParameters();
   LOG(info) << "fPeakingTime: " << fPeakingTime << "; fTBTime: " << fTBTime;
   fResponse = std::make_unique<ElectronicResponse::AtFileResponse>(fTBTime, std::string(filename.Data()));
}

void AtVectorResponse::setParameters()
{
   auto fPar = dynamic_cast<AtDigiPar *>(FairRunAna::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));
   fPeakingTime = fPar->GetPeakingTime() / 1000.;
   fTBTime = fPar->GetTBTime() / 1000.;
}
