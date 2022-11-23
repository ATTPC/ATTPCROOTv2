#include "AtPSAIterDeconv.h"

#include "AtDigiPar.h"
#include "AtHit.h"
#include "AtPad.h"
#include "AtPadArray.h"
#include "AtPadBase.h" // for AtPadBase
#include "AtPadFFT.h"

#include <FairLogger.h>
#include <FairParSet.h> // for FairParSet
#include <FairRun.h>
#include <FairRuntimeDb.h>

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <Rtypes.h>          // for Int_t, Double_t
#include <TComplex.h>
#include <TVirtualFFT.h>

#include <cmath> // for sqrt
#include <numeric>
#include <stdexcept> // for runtime_error
#include <utility>   // for move, pair

using XYZPoint = ROOT::Math::XYZPoint;

AtPSAIterDeconv::AtPSAIterDeconv() : AtPSADeconv() {};

AtPSAIterDeconv::HitVector AtPSAIterDeconv::AnalyzePad(AtPad *pad)
{
   // If this pad does not contains FFT information, then add FFT data to this pad.
   if (dynamic_cast<AtPadFFT *>(pad->GetAugment("fft")) == nullptr) {
      fFFT->SetPoints(pad->GetADC().data());
      fFFT->Transform();
      pad->AddAugment("fft", AtPadFFT::CreateFromFFT(fFFT.get()));
   }

   // Now process the pad with its fourier transform
   AnalyzeFFTpad(*pad);

   return chargeToHits(*pad);
}