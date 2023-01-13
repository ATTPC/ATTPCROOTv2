#ifndef ATTABINFOBASE_H
#define ATTABINFOBASE_H

#include <Rtypes.h>

class AtTabInfoBase {
public:
   AtTabInfoBase() = default;
   virtual ~AtTabInfoBase() = default;

   virtual void Init() = 0;
   virtual void Update() = 0;

   ClassDef(AtTabInfoBase, 1);
};

#endif
