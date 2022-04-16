// S800Event.h
//
// A class prepared for symmetry with respect to the DDASEvent class
//
// Author: Shumpei Noji
// Date  : 5/15/2014
//

#ifndef S800EVENT_H
#define S800EVENT_H

#include <Rtypes.h>
#include <TObject.h>

#include "S800.h"

class TBuffer;
class TClass;
class TMemberInspector;

class S800Event : public TObject {
public:
   S800Event() {}
   void Clear(Option_t * /*option*/ = "") { fS800.Clear(); };

   S800 *GetS800() { return &fS800; }
   void SetS800(S800 s800) { fS800 = s800; }

protected:
   S800 fS800;

   ClassDef(S800Event, 1);
};

#endif
