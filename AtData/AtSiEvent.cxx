#include "AtSiEvent.h"

#include "AtContainerManip.h"

#include <Rtypes.h>

#include <algorithm>
#include <string> // for string

ClassImp(AtSiEvent);

AtSiEvent::AtSiEvent() : AtBaseEvent("AtSiEvent") {}

AtSiEvent::AtSiEvent(const AtSiEvent &copy) : AtBaseEvent(copy),
                                              fMultiplicityFront1(copy.fMultiplicityFront1), fMultiplicityBack1(copy.fMultiplicityBack1),
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

      fStripFront1[i] = -1;
      fStripBack1[i] = -1;
      fStripFront2[i] = -1;
      fStripBack2[i] = -1;
   }

   fMultiplicityFront1 = 0;
   fMultiplicityBack1 = 0;
   fMultiplicityFront2 = 0;
   fMultiplicityBack2 = 0;
}

void AtSiEvent::BuildHits() { //Returning strips to build physical map of Si detectors. For now focusing on multiplicity of 1 or 2 
	bool valid1 = true;
	if (fMultiplicityFront1 == 1) { //To-Do: there are many strips within the reaction runs that have a multiplicity>2. Might be beneficial to implement other combinations of multiplicities so we are not throwing out data.
		//everything is easy
		fXStrip1 = fStripFront1[0]%128;
		fEnergyFront1 = fEFront1[0];
	}
	else if (fMultiplicityFront1 == 2)  {
		if (abs(fStripFront1[0] - fStripFront1[1]) == 1) { //neighbours
			fEnergyFront1 = fEFront1[0] + fEFront1[1];
			//take strip from whichever has larger energy
			if (fEFront1[0] > fEFront1[1]) { fXStrip1 = fStripFront1[0]%128; }
			else { fXStrip1 = fStripFront1[1]%128; }
		}
		else { valid1 = false; }
	}
	else { valid1 = false; }

	if (fMultiplicityBack1 == 1) {
		//everything is easy
		fYStrip1 = 128-fStripBack1[0]%128;
		fEnergyBack1 = fEBack1[0];
	}
	else if (fMultiplicityBack1 == 2)  {
		if (abs(fStripBack1[0] - fStripBack1[1]) == 1) { //neighbours
			fEnergyBack1 = fEBack1[0] + fEBack1[1];
			//take strip from whichever has larger energy
			if (fEBack1[0] > fEBack1[1]) { fYStrip1 = 128-fStripBack1[0]%128; }
			else { fYStrip1 = 128-fStripBack1[1]%128; }
		}
		else { valid1 = false; }
	}
	else { valid1 = false; }

	bool valid2 = true;
	if (fMultiplicityFront2 == 1) {
		//everything is easy
		fXStrip2 = fStripFront2[0]%128;
		fEnergyFront2 = fEFront2[0];
	}
	else if (fMultiplicityFront2 == 2)  {
		if (abs(fStripFront2[0] - fStripFront2[1]) == 1) { //neighbours
			fEnergyFront2 = fEFront2[0] + fEFront2[1];
			//take strip from whichever has larger energy
			if (fEFront2[0] > fEFront2[1]) { fXStrip2 = fStripFront2[0]%128; }
			else { fXStrip2 = fStripFront2[1]%128; }
		}
		else { valid2 = false; }
	}
	else { valid2 = false; }

	if (fMultiplicityBack2 == 1) {
		//everything is easy
		fYStrip2 = 128-fStripBack2[0]%128;
		fEnergyBack2 = fEBack2[0];
	}
	else if (fMultiplicityBack2 == 2)  {
		if (abs(fStripBack2[0] - fStripBack2[1]) == 1) { //neighbours
			fEnergyBack2 = fEBack2[0] + fEBack2[1];
			//take strip from whichever has larger energy
			if (fEBack2[0] > fEBack2[1]) { fYStrip2 = 128-fStripBack2[0]%128; }
			else { fYStrip2 = 128-fStripBack2[1]%128; }
		}
		else { valid2 = false; }
	}
	else { valid2 = false; }

	if (valid1 == false) {
		fYStrip1 = -1;
		fXStrip1 = -1;
		fEnergyFront1 = -1;
		fEnergyBack1 = -1;
	}
	if (valid2 == false) {
		fYStrip2 = -1;
		fXStrip2 = -1;
		fEnergyFront2 = -1;
		fEnergyBack2 = -1;
	}
}

