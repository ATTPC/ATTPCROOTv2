#include "AtROOTresponse.h"

#include "AtElectronicResponse.h"
#include "AtPad.h"

#include <TFile.h>
#include <TObject.h>
using namespace ElectronicResponse;

AtRootResponse::AtRootResponse(double tbTime, std::string filePath, std::string objectName) : fTBTime(tbTime)
{
   TFile file(filePath.data(), "READ");
   if (file.IsZombie())
      throw std::invalid_argument("filePath");
   auto *event = dynamic_cast<AtRawEvent *>(file.FindObject(objectName.data()));
   if (event == nullptr)
      throw std::invalid_argument("objectName");
   fResponse = *event;
}

double AtRootResponse::GetResponse(int padNum, double time) const
{
   int tb = time / fTBTime;
   auto pad = fResponse.GetPad(padNum);
   if (pad == nullptr)
      throw std::invalid_argument("padNum");

   return pad->GetADC(tb);
}
