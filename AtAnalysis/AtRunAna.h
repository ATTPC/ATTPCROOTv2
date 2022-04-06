#ifndef ATRUNANA_H
#define ATRUNANA_H

#include <Rtypes.h>
#include <RtypesCore.h>

#include <FairRunAna.h>

class TBuffer;
class TClass;
class TMemberInspector;

class AtRunAna : public FairRunAna {
public:
   AtRunAna();
   Bool_t GetMarkFill();

   ClassDefOverride(AtRunAna, 1);
};

#endif //#ifndef ATRUNANA_H
