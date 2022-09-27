/*********************************************************************
 *   AtTPC Pad Class	AtPad                                         *
 *   Author: Y. Ayyad                                                 *
 *   Log: 05-03-2015 19:24 JST                                        *
 *                                                                    *
 *                                                                    *
 *********************************************************************/

#include "AtPad.h"

#include <FairLogger.h>

#include <TH1.h>

#include <memory>
#include <string>
#include <utility>
ClassImp(AtPad);

AtPad::AtPad(Int_t PadNum) : fPadNum(PadNum) {}

void swap(AtPad &a, AtPad &b) noexcept
{
   using std::swap; // Enable ADL

   swap(a.fPadNum, b.fPadNum);
   swap(a.fSizeID, b.fSizeID);
   swap(a.fPadCoord, b.fPadCoord);
   swap(a.fIsValid, b.fIsValid);
   swap(a.fIsPedestalSubtracted, b.fIsPedestalSubtracted);
   swap(a.fRawAdc, b.fRawAdc);
   swap(a.fAdc, b.fAdc);
   swap(a.fPadAugments, b.fPadAugments);
}
AtPad &AtPad::operator=(AtPad obj)
{
   swap(*this, obj);
   return *this;
}
AtPad::AtPad(const AtPad &o)
   : fPadNum(o.fPadNum), fSizeID(o.fSizeID), fPadCoord(o.fPadCoord), fIsValid(o.fIsValid),
     fIsPedestalSubtracted(o.fIsPedestalSubtracted), fRawAdc(o.fRawAdc), fAdc(o.fAdc)
{
   for (const auto &pair : fPadAugments)
      fPadAugments[pair.first] = pair.second->Clone();
}
std::unique_ptr<AtPadBase> AtPad::Clone() const
{
   return std::make_unique<AtPad>(*this);
}

/**
 * @brief Clone this pad (including augments).
 */
std::unique_ptr<AtPad> AtPad::ClonePad() const
{
   return std::make_unique<AtPad>(*this);
}

/**
 * Add an augment (ie AtPadFFT) to the pad with given name. If it exists, log an error and replace it.
 */
AtPadBase *AtPad::AddAugment(std::string name, std::unique_ptr<AtPadBase> augment)
{
   if (fPadAugments.find(name) != fPadAugments.end())
      LOG(error) << "AtPad augment " << name
                 << " already exists in pad! If replacement is intentional use Atpad::ReplaceAugment() instead!";

   return ReplaceAugment(name, std::move(augment));
}
/**
 * Adds or replaces an augment (ie AtPadFFT) to the pad with given name.
 */
AtPadBase *AtPad::ReplaceAugment(std::string name, std::unique_ptr<AtPadBase> augment)
{
   fPadAugments[name] = std::move(augment);
   return fPadAugments[name].get();
}
/**
 * Get augment to pad of given name (nullptr if pad doesn't contain the augment).
 */
AtPadBase *AtPad::GetAugment(std::string name)
{
   return const_cast<AtPadBase *>(const_cast<const AtPad *>(this)->GetAugment(name)); // NOLINT
}
/**
 * Get augment to pad of given name (nullptr if pad doesn't contain the augment).
 */
const AtPadBase *AtPad::GetAugment(std::string name) const
{
   if (fPadAugments.find(name) == fPadAugments.end())
      return nullptr;
   else
      return fPadAugments.at(name).get();
}

const AtPad::trace &AtPad::GetADC() const
{
   if (!fIsPedestalSubtracted)
      LOG(debug) << "Pedestal subtraction was not done on pad " << fPadNum;

   return fAdc;
}

Double_t AtPad::GetADC(Int_t idx) const
{
   return GetADC()[idx];
}

std::unique_ptr<TH1D> AtPad::GetADCHistrogram() const
{
   auto histName = "adc" + std::to_string(GetPadNum());
   auto histTitle = "ADC " + std::to_string(GetPadNum());
   auto hist = std::make_unique<TH1D>(histName.data(), histTitle.data(), fAdc.size(), 0, fAdc.size() - 1);
   hist->SetDirectory(nullptr); // Pass ownership to the pointer instead of current ROOT directory
   for (int i = 0; i < fAdc.size(); ++i)
      hist->SetBinContent(i + 1, fAdc[i]);
   return hist;
}
