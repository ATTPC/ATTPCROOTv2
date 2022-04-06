#include "AtRunAna.h"

#include <FairRunAna.h>

AtRunAna::AtRunAna() : FairRunAna() {}

Bool_t AtRunAna::GetMarkFill()
{
   return fMarkFill;
}

ClassImp(AtRunAna);
