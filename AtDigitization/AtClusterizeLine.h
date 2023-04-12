/*****************************************************************/
/*    AtClusterizeTask: Simulates the ionized electrons that are */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*****************************************************************/
#ifndef ATCLUSTERIZELINE_H
#define ATCLUSTERIZELINE_H

#include "AtClusterize.h"

class AtClusterizeLine : public AtClusterize {
private:
   double fTBTime{}; //!< Width of a time bucket [us]

public:
   virtual void GetParameters(AtDigiPar *fPar) override;
   virtual void FillTClonesArray(TClonesArray &array, std::vector<SimPointPtr> &vec) override;

protected:
   virtual std::vector<SimPointPtr> processPoint(AtMCPoint &mcPoint, int pointID) override;
   virtual std::string GetSavedClassName() const override { return "AtSimulatedLine"; }
};

#endif //#define ATCLUSTERIZELINETASK_H
