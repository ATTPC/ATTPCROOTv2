#ifndef ATRUNANA_H
#define ATRUNANA_H

#include "FairRunAna.h"

class AtRunAna : public FairRunAna {
public:
   AtRunAna();
   Bool_t GetMarkFill();

   ClassDefOverride(AtRunAna, 1);
};

#endif //#ifndef ATRUNANA_H
