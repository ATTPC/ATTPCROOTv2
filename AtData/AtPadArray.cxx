#include "AtPadArray.h"

#include <TH1.h> // for TH1D

std::unique_ptr<AtPadBase> AtPadArray::Clone() const
{
   return std::make_unique<AtPadArray>(*this);
}

std::unique_ptr<TH1D> AtPadArray::GetHist(std::string name) const
{
   auto hist = std::make_unique<TH1D>(name.c_str(), name.c_str(), fArray.size(), 0, fArray.size() - 1);
   hist->SetDirectory(nullptr); // Pass ownership to the pointer instead of current ROOT directory
   for (int i = 0; i < fArray.size(); ++i)
      hist->SetBinContent(i + 1, fArray[i]);
   return hist;
}
ClassImp(AtPadArray);
