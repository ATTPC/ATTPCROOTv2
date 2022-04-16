/*********************************************************************
 *   AtTools Class for kinematics calculations                       *
 *   Author: Y. Ayyad yassid.ayyad@usc.es            	             *
 *   Log: 03/28/2022 					             *
 *								     *
 *********************************************************************/

#ifndef ATKINEMATICS_H
#define ATKINEMATICS_H

#include <Rtypes.h> // for Double_t, THashConsistencyHolder, Int_t, ClassDef

#include "TObject.h" // for TObject

#include <tuple> // for tuple
class TBuffer;
class TClass;
class TMemberInspector;

namespace AtTools {

class AtKinematics : public TObject {

public:
   AtKinematics();
   ~AtKinematics() = default;

   void SetVerbosity(Int_t verbosity) { fVerbosity = verbosity; }

   std::tuple<Double_t, Double_t>
   GetMomFromBrho(Double_t A, Double_t Z,
                  Double_t brho); ///< Returns momentum (in GeV) from Brho assuming M (amu) and Z;
   Double_t
   TwoBodyEx(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject);
   Double_t omega(Double_t x, Double_t y, Double_t z);

private:
   Int_t fVerbosity;

   ClassDef(AtKinematics, 1);
};

} // namespace AtTools

#endif
