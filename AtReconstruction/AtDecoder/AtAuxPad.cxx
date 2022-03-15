#include "AtAuxPad.h"

AtAuxPad::AtAuxPad(std::string auxName) : AtPad(-1), fAuxName(std::move(auxName)) {}

ClassImp(AtAuxPad);
