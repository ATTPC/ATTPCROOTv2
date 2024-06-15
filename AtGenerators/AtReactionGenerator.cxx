#include "AtReactionGenerator.h"

#include "AtVertexPropagator.h"

Bool_t AtReactionGenerator::ReadEvent(FairPrimaryGenerator *primGen)
{
   bool isBeamEvent = AtVertexPropagator::Instance()->IsBeamEvent();
   if (kIsFinalGen)
      AtVertexPropagator::Instance()->EndEvent(); // End the event if this is the final generator

   if (!isBeamEvent || kAlwaysRun)
      return GenerateReaction(primGen);
   return true;
}