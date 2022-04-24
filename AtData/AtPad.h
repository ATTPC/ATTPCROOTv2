/*********************************************************************
 *   AtTPC Pad Class	AtPad                                            *
 *   Author: Y. Ayyad            				                     *
 *   Log: 05-03-2015 19:24 JST					                     *
 *   Adapted from SPiRITROOT STPad by G. Jhang                        *
 *								                                     *
 *********************************************************************/

#ifndef AtPAD_H
#define AtPAD_H

#include <Math/Point2D.h>
#include <Math/Point2Dfwd.h>
#include <Rtypes.h>
#include <TObject.h>

#include <array>
#include <memory>

class TBuffer;
class TClass;
class TMemberInspector;
class TH1D;

class AtPad : public TObject {
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

public:
   AtPad(Int_t PadNum = -1);
   AtPad(const AtPad &obj) = default;
   virtual ~AtPad() = default;
   virtual std::unique_ptr<AtPad> Clone(); // Create a copy of sub-type

   void SetValidPad(Bool_t val = kTRUE) { fIsValid = val; }
   void SetPadNum(Int_t padNum) { fPadNum = padNum; }
   void SetSizeID(Int_t sizeID) { fSizeID = sizeID; }

   void SetPedestalSubtracted(Bool_t val = kTRUE) { fIsPedestalSubtracted = val; }
   void SetPadCoord(const XYPoint &point) { fPadCoord = point; }
   // void SetIsAux(Bool_t val) { fIsAux = val; }
   // void SetAuxName(std::string val) { fAuxName = std::move(val); }

   void SetRawADC(const rawTrace &val) { fRawAdc = val; }
   void SetRawADC(Int_t idx, Int_t val) { fRawAdc[idx] = val; }
   void SetADC(const trace &val) { fAdc = val; }
   void SetADC(Int_t idx, Double_t val) { fAdc[idx] = val; }

   Bool_t IsPedestalSubtracted() const { return fIsPedestalSubtracted; }
   // Bool_t IsAux() const { return fIsAux; }

   Int_t GetPadNum() const { return fPadNum; }
   Bool_t GetValidPad() const { return fIsValid; }
   // std::string GetAuxName() const { return fAuxName; }
   Int_t GetSizeID() const { return fSizeID; }

   const trace &GetADC() const;
   Double_t GetADC(Int_t idx) const;
   std::unique_ptr<TH1D> GetADCHistrogram() const;

   const rawTrace &GetRawADC() const { return fRawAdc; }
   Int_t GetRawADC(Int_t idx) const { return fRawAdc[idx]; }

   XYPoint GetPadCoord() const { return fPadCoord; }

   ClassDef(AtPad, 2);
};

#endif
