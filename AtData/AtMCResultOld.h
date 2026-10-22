#ifndef ATMCRESULTOLD_H
#define ATMCRESULTOLD_H

#include <Rtypes.h> // for Double_t, THashConsistencyHolder, ClassDefOverride
#include <TObject.h>

#include <map>
#include <string> // for string
class TBuffer;
class TClass;
class TMemberInspector;

namespace MCFitter {

/**
 * Class for storing the result of an iteration in the AtMCFitter method.
 */
class [[deprecated]] AtMCResultOld : public TObject {
public:
   using ParamMap = std::map<std::string, Double_t>;

   Double_t fObjective;  //< Value f the objective function for this iteration
   ParamMap fParameters; //< Parameters used in simulation
   Int_t fIterNum;       //< Iteration number. Used to map with the simulated event ID in the TTree.

   AtMCResultOld() = default;

   void Print() const;

   ClassDefOverride(AtMCResultOld, 1);
};

} // namespace MCFitter

#endif // #ifndef ATMCRESULT_H
