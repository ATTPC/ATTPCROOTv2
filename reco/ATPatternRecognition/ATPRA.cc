#include "ATPRA.hh"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

// STL
#include <algorithm>

ClassImp(ATPATTERN::ATPRA)


ATPATTERN::ATPRA::~ATPRA()
{
}

void ATPATTERN::ATPRA::SetTrackCurvature(ATTrack& track)
{
        std::cout<<" new track  "<<"\n";
        std::vector<double> radius_vec;
        std::vector<ATHit>* hitArray = track.GetHitArray();
        int nstep = 0.60*hitArray->size(); //20% of the hits to calculate the radius of curvature with less fluctuations
        ATHit iHit;

        for(Int_t iHit=0; iHit<(hitArray->size()-nstep); iHit++){

          ATHit hitA = hitArray->at(iHit);
          ATHit hitB = hitArray->at((int)(iHit+(nstep/2.0)));
          ATHit hitC = hitArray->at((int)(iHit+nstep));

          //std::cout<<nstep<<" "<<iHit<<"  "<<(int)(iHit+(nstep/2.0))<<"  "<<(int)(iHit+nstep)<<"\n";

          TVector3 posA = hitA.GetPosition();
          TVector3 posB = hitB.GetPosition();
          TVector3 posC = hitC.GetPosition();

          double slopeAB = ( posB.Y()-posA.Y() )/( posB.X()-posA.X() ); //m1
          double slopeBC = ( posC.Y()-posB.Y() )/( posC.X()-posB.X() ); //m2

          double centerX = ( slopeAB*slopeBC*( posA.Y()-posC.Y() ) + slopeBC*( posB.X()+posA.X() )
             - slopeAB*(posB.X()+posC.X()) )/ ( 2.0*(slopeBC - slopeAB) );

          double centerY = (-1/slopeAB)*( centerX - ( posB.X()+posA.X() )/2.0 )  + ( posB.Y()+posA.Y() )/2.0;

          std::cout<<" Center "<<centerX<<" - "<<centerY<<"\n";

          double radiusA = TMath::Sqrt( TMath::Power(posA.X() - centerX,2) + TMath::Power(posA.Y() - centerY,2)   );
          radius_vec.push_back(radiusA);
          double radiusB = TMath::Sqrt( TMath::Power(posB.X() - centerX,2) + TMath::Power(posB.Y() - centerY,2)   );
          radius_vec.push_back(radiusB);
          double radiusC = TMath::Sqrt( TMath::Power(posC.X() - centerX,2) + TMath::Power(posC.Y() - centerY,2)   );
          radius_vec.push_back(radiusC);




        }


}
