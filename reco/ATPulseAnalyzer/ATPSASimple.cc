
#include "ATPSASimple.hh"

// STL
#include <algorithm>

using std::max_element;
using std::min_element;
using std::distance;

ClassImp(ATPSASimple) ATPSASimple::ATPSASimple()
{
}

ATPSASimple::~ATPSASimple()
{
}

void
 ATPSASimple::Analyze(ATRawEvent * rawEvent, ATEvent * event)
{
    Int_t numPads = rawEvent->GetNumPads();
    Int_t hitNum = 0;

    for (Int_t iPad = 0; iPad < numPads; iPad++) {
	ATPad *pad = rawEvent->GetPad(iPad);

	Double_t xPos = pad->GetPadXCoord();
	Double_t yPos = pad->GetPadYCoord();
	Double_t zPos = 0;
	Double_t charge = 0;

	if (pad->IsPedestalSubtracted()) {	//TODO after pedestal subtraction
	    Double_t *adc = pad->GetADC();
	    Int_t maxAdcIdx =
		distance(adc, max_element(adc + 4, adc + fNumTbs - 5));

	    zPos = CalculateY(maxAdcIdx);
	    charge = adc[maxAdcIdx];

	    if (fThreshold > 0 && charge < fThreshold)
		continue;
	} else {
	    Int_t *rawAdc = pad->GetRawADC();
	    Int_t minAdcIdx =
		distance(rawAdc, min_element(rawAdc + 4, rawAdc + fNumTbs - 5));

	    zPos = CalculateZ(minAdcIdx);
	    charge = rawAdc[minAdcIdx];

	    if (fThreshold > 0 && charge > fThreshold)
		continue;
	}

	ATHit *hit = new ATHit(hitNum, xPos, yPos, zPos, charge);
	event->AddHit(hit);
	delete hit;

	hitNum++;
    }
}
