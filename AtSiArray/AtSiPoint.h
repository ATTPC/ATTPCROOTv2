#ifndef ATSIPOINT_H
#define ATSIPOINT_H

#include "AtMCPoint.h"

#include <Rtypes.h>
#include <TString.h>
#include <TVector3.h>

class TBuffer;
class TClass;
class TMemberInspector;

class AtSiPoint : public AtMCPoint {

protected:
   Double32_t fX_out{}, fY_out{}, fZ_out{};
   Double32_t fPx_out{}, fPy_out{}, fPz_out{};

public:
   /** Default constructor **/
   AtSiPoint();

   /** Constructor with arguments
    *@param trackID  Index of MCTrack
    *@param detID    Detector ID
    *@param pos      Ccoordinates at entrance to active volume [cm]
    *@param mom      Momentum of track at entrance [GeV]
    *@param tof      Time since event start [ns]
    *@param length   Track length since creation [cm]
    *@param eLoss    Energy deposit [GeV]
    **/
   AtSiPoint(Int_t trackID, Int_t detID, TVector3 pos, TVector3 mom, Double_t tof, Double_t length, Double_t eLoss);

   AtSiPoint(Int_t trackID, Int_t detID, TVector3 posIn, TVector3 posOut, TVector3 momIn, TVector3 momOut, Double_t tof,
             Double_t length, Double_t eLoss, TString VolName, Int_t detCopyID, Double_t EIni, Double_t AIni, Int_t A,
             Int_t Z);

   /** Destructor **/
   virtual ~AtSiPoint() = default;

   /** Accessors **/
   Double_t GetXIn() const { return fX; }
   Double_t GetYIn() const { return fY; }
   Double_t GetZIn() const { return fZ; }
   Double_t GetXOut() const { return fX_out; }
   Double_t GetYOut() const { return fY_out; }
   Double_t GetZOut() const { return fZ_out; }
   Double_t GetPxOut() const { return fPx_out; }
   Double_t GetPyOut() const { return fPy_out; }
   Double_t GetPzOut() const { return fPz_out; }
   void PositionIn(TVector3 &pos) { pos.SetXYZ(fX, fY, fZ); }
   void PositionOut(TVector3 &pos) { pos.SetXYZ(fX_out, fY_out, fZ_out); }
   void MomentumOut(TVector3 &mom) { mom.SetXYZ(fPx_out, fPy_out, fPz_out); }

   /** Point coordinates at given z from linear extrapolation **/
   Double_t GetX(Double_t z) const;
   Double_t GetY(Double_t z) const;

   /** Modifiers **/
   void SetPositionOut(TVector3 pos);
   void SetMomentumOut(TVector3 mom);

   /** Output to screen **/
   virtual void Print(const Option_t *opt) const;

private:
   /** Copy constructor **/
   AtSiPoint(const AtSiPoint &point);
   AtSiPoint operator=(const AtSiPoint &point);

   ClassDef(AtSiPoint, 1)
};

inline void AtSiPoint::SetPositionOut(TVector3 pos)
{
   fX_out = pos.X();
   fY_out = pos.Y();
   fZ_out = pos.Z();
}

inline void AtSiPoint::SetMomentumOut(TVector3 mom)
{
   fPx_out = mom.Px();
   fPy_out = mom.Py();
   fPz_out = mom.Pz();
}
#endif //#ifndef ATSIPOINT_H
