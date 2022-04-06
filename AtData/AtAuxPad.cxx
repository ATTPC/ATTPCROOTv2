#include "AtAuxPad.h"

#include <Rtypes.h>

AtAuxPad::AtAuxPad(std::string auxName) : AtPad(-1), fAuxName(std::move(auxName)) {}

ClassImp(AtAuxPad);
