#include "AtAuxPad.h"

#include "AtPadBase.h"

#include <Rtypes.h>

AtAuxPad::AtAuxPad(std::string auxName) : AtPad(-1), fAuxName(std::move(auxName)) {}

std::unique_ptr<AtPad> AtAuxPad::ClonePad() const
{
   return std::make_unique<AtAuxPad>(*this);
}
std::unique_ptr<AtPadBase> AtAuxPad::Clone() const
{
   return std::make_unique<AtAuxPad>(*this);
}
ClassImp(AtAuxPad);
