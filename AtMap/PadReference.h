#ifndef _PADREFERENCE_H
#define _PADREFERENCE_H

#include <Rtypes.h>

#include <cstddef>

// The definition of this struct, and the operator overloads have to
// be before AtMap where an unordered_map using this as a key is
// instatiated.
struct PadReference {
   Int_t cobo;
   Int_t asad;
   Int_t aget;
   Int_t ch;
};
bool operator<(const PadReference &l, const PadReference &r);
bool operator==(const PadReference &l, const PadReference &r);
namespace std {
template <>
struct hash<PadReference> {
   inline size_t operator()(const PadReference &x) const
   {
      return x.ch + x.aget * 100 + x.asad * 10000 + x.cobo * 1000000;
   }
};
} // namespace std

#endif //#ifndef _PADREFERENCE_H
