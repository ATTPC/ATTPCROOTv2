/*****************************************************************/
/*    AtClusterizeTask: Simulates the ionized electrons that are */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*****************************************************************/
#ifndef ATCLUSTERIZELINETASK_H
#define ATCLUSTERIZELINETASK_H

#include "AtClusterizeLine.h"
#include "AtClusterizeTask.h"

#include <Rtypes.h>

#include <memory> // for make_shared, shared_ptr

class TBuffer;
class TClass;
class TMemberInspector;

class [[deprecated(
   "AtClusterizeLineTask is deprecated. Use AtClusterizeTask with AtClusterizeLine instead.")]] AtClusterizeLineTask
   : public AtClusterizeTask
{
   AtClusterizeLineTask() : AtClusterizeTask(std::make_shared<AtClusterizeLine>()) {}

   ClassDefOverride(AtClusterizeLineTask, 2);
};

#endif //#define ATCLUSTERIZELINETASK_H
