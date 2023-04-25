/*****************************************************************/
/*    AtClusterizeTask: Simulates the ionized electrons that are */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*****************************************************************/
#ifndef ATCLUSTERIZELINE_H
#define ATCLUSTERIZELINE_H

#include "AtClusterize.h"

#include <memory> // for make_shared, shared_ptr
#include <string> // for allocator, string
#include <vector> // for vector
class AtDigiPar;
class AtMCPoint;
class TClonesArray;

class AtClusterizeLine : public AtClusterize {
private:
   double fTBTime{}; //!< Width of a time bucket [us]

public:
   virtual void GetParameters(const AtDigiPar *fPar) override;
   virtual void FillTClonesArray(TClonesArray &array, std::vector<SimPointPtr> &vec) override;
   virtual std::shared_ptr<AtClusterize> Clone() const override { return std::make_shared<AtClusterizeLine>(*this); }

protected:
   virtual std::vector<SimPointPtr> processPoint(AtMCPoint &mcPoint, int pointID) override;
   virtual std::string GetSavedClassName() const override { return "AtSimulatedLine"; }
};

#endif //#define ATCLUSTERIZELINETASK_H
