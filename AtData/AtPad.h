/*********************************************************************
 *   AtTPC Pad Class	AtPad                                            *
 *   Author: Y. Ayyad            				                     *
 *   Log: 05-03-2015 19:24 JST					                     *
 *   Adapted from SPiRITROOT STPad by G. Jhang                        *
 *								                                     *
 *********************************************************************/

#ifndef AtPAD_H
#define AtPAD_H
#include "AtPadBase.h"

#include <Math/Point2D.h>
#include <Math/Point2Dfwd.h>
#include <Rtypes.h>

#include <array>
#include <map>
#include <memory>
#include <string>

class TBuffer;
class TClass;
class TMemberInspector;
class TH1D;

/**
 * @brief Container class for AtPadBase objects
 *
 * This class contains the information associated with a pad (or channel). In addition to the
 * information required to define a pad (adc value, pad number, etc) additional information can be
 * added through pad "augments" (this follows the compostion design pattern). All augments should be
 * listed in the group Pads, which has more documentation in this system.
 *
 * @ingroup Pads
 */
class AtPad : public AtPadBase {
public:
   using rawTrace = std::array<Int_t, 512>;
   using trace = std::array<Double_t, 512>;
   using XYPoint = ROOT::Math::XYPoint;

protected:
   Int_t fPadNum; // Pad reference number in AtMap
   Int_t fSizeID = -1000;
   XYPoint fPadCoord{-9999, -9999};
   Bool_t fIsValid = true;
   Bool_t fIsPedestalSubtracted = false;

   rawTrace fRawAdc{};
   trace fAdc{};
   std::map<std::string, std::unique_ptr<AtPadBase>> fPadAugments;

public:
   AtPad(Int_t PadNum = -1);
   AtPad(const AtPad &obj);
   AtPad &operator=(AtPad obj);
   AtPad(AtPad &&) = default;
   virtual ~AtPad() = default;
   friend void swap(AtPad &a, AtPad &b) noexcept;

   virtual std::unique_ptr<AtPadBase> Clone() const override;
   virtual std::unique_ptr<AtPad> ClonePad() const;

   AtPadBase *AddAugment(std::string name, std::unique_ptr<AtPadBase> augment);
   AtPadBase *ReplaceAugment(std::string name, std::unique_ptr<AtPadBase> augment);
   AtPadBase *GetAugment(std::string name);
   const AtPadBase *GetAugment(std::string name) const;
   template <typename T, typename std::enable_if_t<std::is_base_of<AtPadBase, T>::value> * = nullptr>
   T *GetAugment(std::string name)
   {
      return dynamic_cast<T *>(GetAugment(name));
   }
   template <typename T, typename std::enable_if_t<std::is_base_of<AtPadBase, T>::value> * = nullptr>
   const T *GetAugment(std::string name) const
   {
      return dynamic_cast<const T *>(GetAugment(name));
   }

   const std::map<std::string, std::unique_ptr<AtPadBase>> &GetAugments() const { return fPadAugments; }
   void SetValidPad(Bool_t val = kTRUE) { fIsValid = val; }
   void SetPadNum(Int_t padNum) { fPadNum = padNum; }
   void SetSizeID(Int_t sizeID) { fSizeID = sizeID; }

   void SetPedestalSubtracted(Bool_t val = kTRUE) { fIsPedestalSubtracted = val; }
   void SetPadCoord(const XYPoint &point) { fPadCoord = point; }

   void SetRawADC(const rawTrace &val) { fRawAdc = val; }
   void SetRawADC(Int_t idx, Int_t val) { fRawAdc[idx] = val; }
   void SetADC(const trace &val) { fAdc = val; }
   void SetADC(Int_t idx, Double_t val) { fAdc[idx] = val; }

   Bool_t IsPedestalSubtracted() const { return fIsPedestalSubtracted; }

   Int_t GetPadNum() const { return fPadNum; }
   Bool_t GetValidPad() const { return fIsValid; }
   Int_t GetSizeID() const { return fSizeID; }

   const trace &GetADC() const;
   Double_t GetADC(Int_t idx) const;
   std::unique_ptr<TH1D> GetADCHistrogram() const;

   const rawTrace &GetRawADC() const { return fRawAdc; }
   Int_t GetRawADC(Int_t idx) const { return fRawAdc[idx]; }

   XYPoint GetPadCoord() const { return fPadCoord; }

   friend class AtGRAWUnpacker;
   ClassDefOverride(AtPad, 3);
};

#endif
