#include "AtPadReference.h"

#include <iostream>

std::ostream &operator<<(std::ostream &os, const AtPadReference &t)
{
   os << "[" << t.cobo << "," << t.asad << "," << t.aget << "," << t.ch << "]";
   return os;
}

bool operator<(const AtPadReference &l, const AtPadReference &r)
{
   return std::hash<AtPadReference>()(l) < std::hash<AtPadReference>()(r);
}

bool operator==(const AtPadReference &l, const AtPadReference &r)
{
   return l.cobo == r.cobo && l.asad == r.asad && l.aget == r.aget && l.ch == r.ch;
}

ClassImp(AtElectronicReference);
