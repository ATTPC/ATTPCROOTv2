#ifndef ATVECTORRESPONSE_H
#define ATVECTORRESPONSE_H

#include <TString.h>

#include <memory> // for unique_ptr

namespace ElectronicResponse {
class AtFileResponse;
}

class [[deprecated("Use ElectronicResponse::AtFileResponse instead")]] AtVectorResponse
{
protected:
   static double fPeakingTime; //! Electronic peaking time in us
   static double fTBTime;      //! Time bucket size in us
   static std::unique_ptr<ElectronicResponse::AtFileResponse> fResponse;

public:
   AtVectorResponse() = default;
   static double ResponseFunction(double reducedTime);
   static void SetFile(TString filename);

protected:
   static void setParameters();
};
#endif
