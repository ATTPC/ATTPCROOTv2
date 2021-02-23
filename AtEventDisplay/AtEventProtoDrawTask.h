#ifndef AtEVENTPROTODRAWTASK_H
#define AtEVENTPROTODRAWTASK_H

#include "AtEventDrawTask.h"

class AtEventProtoDrawTask : public AtEventDrawTask
{
  public :

   AtEventProtoDrawTask();
   ~AtEventProtoDrawTask();

   void DrawPadPlane();

   ClassDef(AtEventProtoDrawTask,1);

};

#endif
