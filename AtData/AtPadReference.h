#ifndef _ATPADREFERENCE_H
#define _ATPADREFERENCE_H

#include "AtPadBase.h"

#include <Rtypes.h>

#include <cstddef>
#include <functional> // IWYU pragma: keep
#include <iosfwd>
#include <memory>

class TBuffer;
class TClass;
class TMemberInspector;

// The definition of this struct, and the operator overloads have to
// be before AtMap where an unordered_map using this as a key is
// instatiated.
struct AtPadReference {
   Int_t cobo;
   Int_t asad;
   Int_t aget;
   Int_t ch;
};
bool operator<(const AtPadReference &l, const AtPadReference &r);
bool operator==(const AtPadReference &l, const AtPadReference &r);
std::ostream &operator<<(std::ostream &os, const AtPadReference &t);

namespace std {
template <>
struct hash<AtPadReference> {
   inline size_t operator()(const AtPadReference &x) const
   {
      return x.ch + x.aget * 100 + x.asad * 10000 + x.cobo * 1000000;
   }
};
} // namespace std

class AtElectronicReference : public AtPadBase {
private:
   AtPadReference fRef;

public:
   AtElectronicReference(AtPadReference ref = {}) : fRef(ref) {}
   virtual std::unique_ptr<AtPadBase> Clone() const override { return std::make_unique<AtElectronicReference>(*this); }
   AtPadReference GetReference() { return fRef; }
   ClassDefOverride(AtElectronicReference, 1);
};
#endif //#ifndef _PADREFERENCE_H
