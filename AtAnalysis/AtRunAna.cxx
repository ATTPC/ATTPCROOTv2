#include "AtRunAna.h"

AtRunAna::AtRunAna() : FairRunAna() {}

Bool_t AtRunAna::GetMarkFill()
{
   return fMarkFill;
}

ClassImp(AtRunAna);
