#ifndef ATEVENTPROTODRAWTASK_H
#define ATEVENTPROTODRAWTASK_H

#include "ATEventDrawTask.hh"

class ATEventProtoDrawTask : public ATEventDrawTask
{
  public :

   ATEventProtoDrawTask();
   ~ATEventProtoDrawTask();

   void DrawPadPlane();

   ClassDef(ATEventProtoDrawTask,1);

};

#endif
