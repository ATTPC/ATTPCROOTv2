#include "AtContainerManip.h"
#include "AtSiEvent.h"

#include <Rtypes.h>

#include <algorithm>
#include <string> // for string

ClassImp(AtSiEvent);

AtSiEvent::AtSiEvent() : AtBaseEvent("AtSiEvent") {}

AtSiEvent::AtSiEvent(const AtSiEvent &copy)
   : AtBaseEvent(copy), fMultiplicityFront1(copy.fMultiplicityFront1), fMultiplicityBack1(copy.fMultiplicityBack1),
     fMultiplicityFront2(copy.fMultiplicityFront2), fMultiplicityBack2(copy.fMultiplicityBack2)
{
   for (int i = 0; i < 4; i++) {
      fEFront1[i] = copy.fEFront1[i];
      fEBack1[i] = copy.fEBack1[i];
      fEFront2[i] = copy.fEFront2[i];
      fEBack2[i] = copy.fEBack2[i];

      fADCMaxFront1[i] = copy.fADCMaxFront1[i];
      fADCMaxBack1[i] = copy.fADCMaxBack1[i];
      fADCMaxFront2[i] = copy.fADCMaxFront2[i];
      fADCMaxBack2[i] = copy.fADCMaxBack2[i];

      fStripFront1[i] = copy.fStripFront1[i];
      fStripBack1[i] = copy.fStripBack1[i];
      fStripFront2[i] = copy.fStripFront2[i];
      fStripBack2[i] = copy.fStripBack2[i];
   }
}

AtSiEvent &AtSiEvent::operator=(AtSiEvent object)
{
   swap(*this, object);
   return *this;
}

void AtSiEvent::Clear(Option_t *opt)
{
   AtBaseEvent::Clear(opt);
   for (int i = 0; i < 4; i++) {
      fEFront1[i] = -1;
      fEBack1[i] = -1;
      fEFront2[i] = -1;
      fEBack2[i] = -1;

      fADCMaxFront1[i] = -1;
      fADCMaxBack1[i] = -1;
      fADCMaxFront2[i] = -1;
      fADCMaxBack2[i] = -1;

      fStripFront2[i] = -1;
      fStripBack2[i] = -1;
      fStripFront2[i] = -1;
      fStripBack2[i] = -1;
   }

   fMultiplicityFront1 = 0;
   fMultiplicityBack1 = 0;
   fMultiplicityFront2 = 0;
   fMultiplicityBack2 = 0;
}
