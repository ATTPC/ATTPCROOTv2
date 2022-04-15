/*****************************************************************/
/*    AtClusterizeTask: Simulates the ionized electrons that are */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*****************************************************************/
#ifndef ATCLUSTERIZELINETASK_H
#define ATCLUSTERIZELINETASK_H

#include <FairTask.h>
#include <Rtypes.h>

#include "AtClusterizeTask.h"

class TBuffer;
class TClass;
class TMemberInspector;

class AtClusterizeLineTask : public AtClusterizeTask {
private:
   Double_t fTBTime{}; //!< Width of a time bucket [us]

protected:
   virtual void getParameters() override;
   virtual void processPoint(Int_t mcPointID) override;

public:
   AtClusterizeLineTask();
   ~AtClusterizeLineTask();

   virtual InitStatus Init() override; //!< Initiliazation of task at the beginning of a run.

   ClassDefOverride(AtClusterizeLineTask, 1);
};

#endif //#define ATCLUSTERIZELINETASK_H
