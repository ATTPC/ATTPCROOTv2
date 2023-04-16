#include "AtSimulatedLine.h"

#include <utility>

using XYZVector = ROOT::Math::XYZVector;

AtSimulatedLine::AtSimulatedLine() : AtSimulatedPoint(), fSigmaLongDiffusion(-1), fSigmaTransDiffusion(-1) {}

AtSimulatedLine::AtSimulatedLine(std::size_t mcPointID, Int_t clusterID, Int_t charge, const XYZVector &posIn,
                                 XYZVector posOut, Double_t longitudalDiffusionSigma, Double_t transverseDiffusionSigma)
   : AtSimulatedPoint(mcPointID, clusterID, charge, posIn), fSigmaTransDiffusion(transverseDiffusionSigma),
     fSigmaLongDiffusion(longitudalDiffusionSigma), fPositionFinal(std::move(posOut))
{
}

AtSimulatedLine &AtSimulatedLine::operator=(AtSimulatedLine other)
{
   swap(*this, other);
   return *this;
}

void AtSimulatedLine::SetFinalPosition(Double_t x, Double_t y, Double_t zTime)
{
   fPositionFinal.SetCoordinates(x, y, zTime);
}

void AtSimulatedLine::SetInitialPosition(Double_t x, Double_t y, Double_t zTime)
{
   fPosition.SetCoordinates(x, y, zTime);
}
void AtSimulatedLine::SetLongitudinalDiffusion(Double_t sigma)
{
   fSigmaLongDiffusion = sigma;
}

void AtSimulatedLine::SetTransverseDiffusion(Double_t sigma)
{
   fSigmaTransDiffusion = sigma;
}

XYZVector AtSimulatedLine::GetPosition()
{
   return (fPositionFinal + fPosition) / 2.0;
}
XYZVector AtSimulatedLine::GetFinalPosition()
{
   return fPositionFinal;
}
XYZVector AtSimulatedLine::GetInitialPosition()
{
   return fPosition;
}
Double_t AtSimulatedLine::GetTransverseDiffusion()
{
   return fSigmaTransDiffusion;
}
Double_t AtSimulatedLine::GetLongitudinalDiffusion()
{
   return fSigmaLongDiffusion;
}
