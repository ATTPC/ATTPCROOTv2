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

// AtPSAIterDeconv::AtPSAIterDeconv() : AtPSADeconv() {};

AtPSAIterDeconv::HitVector AtPSAIterDeconv::AnalyzePad(AtPad *pad)
{
   RunPad(pad);

   // If saving the charge from the iterations to a different augment, make that augment
   if (fQName != "Qreco") {
      auto charge = std::make_unique<AtPadArray>();
      // Fill the charge pad
      for (int i = 0; i < 512; ++i)
         charge->SetArray(i, dynamic_cast<AtPadArray *>(pad->GetAugment("Qreco"))->GetArray(i));
      pad->AddAugment(fQName, std::move(charge));
   }

   auto respPad = GetResponse(pad->GetPadNum());
   AtPad *diffPad = new AtPad(pad->GetPadNum());

   for (int i = 0; i < fIterations; i++) {
      for (int r = 0; r < 512; r++) {
         double reconSig = 0;
         for (int a = 0; a < r + 1; a++) {
            reconSig += dynamic_cast<AtPadArray *>(pad->GetAugment(fQName))->GetArray(a) * respPad.GetADC(r - a);
         }
         diffPad->SetADC(r, pad->GetADC(r) - reconSig);
      }
      auto *testPad = new AtPad(pad->GetPadNum());
      testPad->SetADC(diffPad->GetADC());
      RunPad(testPad);

      for (int r = 0; r < 512; r++) {
         auto charge = dynamic_cast<AtPadArray *>(pad->GetAugment(fQName))->GetArray(r);
         charge += dynamic_cast<AtPadArray *>(testPad->GetAugment("Qreco"))->GetArray(r);
         dynamic_cast<AtPadArray *>(pad->GetAugment(fQName))->SetArray(r, charge);
      }
      testPad->Delete();
   }
   diffPad->Delete();

   return chargeToHits(*pad, fQName);
}

void AtPSAIterDeconv::RunPad(AtPad *pad)
{
   // If this pad does not contains FFT information, then add FFT data to this pad.
   if (dynamic_cast<AtPadFFT *>(pad->GetAugment("fft")) == nullptr) {
      fFFT->SetPoints(pad->GetADC().data());
      fFFT->Transform();
      pad->AddAugment("fft", AtPadFFT::CreateFromFFT(fFFT.get()));
   }

   // Now process the pad
   AnalyzeFFTpad(*pad);
}
