/*****************************************************************/
/*    AtClusterizeTask: Simulates the ionized electrons which are    */
/*    drifted and amplified by avalanche when reaching the mesh. */
/*    Log: Created in 24-10-2016     				 */
/*    Author: Nathan Watwood (NSCL) 				 */
/*    ayyadlim@nscl.msu.edu                                      */
/*****************************************************************/
#ifndef AtClusterizeTask_H
#define AtClusterizeTask_H

#include <FairTask.h>

#include <Rtypes.h>
#include <TClonesArray.h>
#include <TString.h> // for TString

#include <memory>
class AtClusterize;
class AtDigiPar;
class TBuffer;
class TClass;
class TMemberInspector;

class AtClusterizeTask : public FairTask {
protected:
   Int_t fEventID{0}; //!< EventID

   AtDigiPar *fPar{}; //!< Base parameter container.

   // IO stuff
   TString fMCPointName{"AtTpcPoint"};
   TClonesArray *fMCPointArray{};

   std::unique_ptr<TClonesArray> fSimulatedPointArray{nullptr}; //!< Primary cluster array
   Bool_t fIsPersistent{false};                                 //!< If true, save container

   std::shared_ptr<AtClusterize> fClusterize; //<!Cluster Task

public:
   AtClusterizeTask(std::shared_ptr<AtClusterize> clusterize = std::make_shared<AtClusterize>(),
                    const char *name = "AtClusterizeTask");
   ~AtClusterizeTask();

   void SetPersistence(Bool_t val) { fIsPersistent = val; }
   void SetClusterizeMethod(std::shared_ptr<AtClusterize> cluster) { fClusterize = cluster; }

   virtual InitStatus Init() override;        //!< Initiliazation of task at the beginning of a run.
   virtual void Exec(Option_t *opt) override; //!< Executed for each event.
   virtual void SetParContainers() override;  //!< Load the parameter container from the runtime database.

   ClassDefOverride(AtClusterizeTask, 2);
};

#endif
