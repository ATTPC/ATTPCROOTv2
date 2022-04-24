#include "AtAuxPad.h"

#include <Rtypes.h>

AtAuxPad::AtAuxPad(std::string auxName) : AtPad(-1), fAuxName(std::move(auxName)) {}
std::unique_ptr<AtPad> AtAuxPad::Clone()
{
   return std::make_unique<AtAuxPad>(*this);
}
ClassImp(AtAuxPad);
