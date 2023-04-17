#ifndef ATMCRESULT_H
#define ATMCRESULT_H

#include <TObject.h>

#include <map>
namespace MCFitter {

/**
 * Class for storing the result of an iteration in the AtMCFitter method.
 */
class AtMCResult : public TObject {
public:
   using ParamMap = std::map<std::string, Double_t>;

   Double_t fObjective;  //< Value f the objective function for this iteration
   ParamMap fParameters; //< Parameters used in simulation
   Int_t fIterNum;       //< Iteration number. Used to map with the simulated event ID in the TTree.

   AtMCResult() = default;

   void Print() const;

   ClassDefOverride(AtMCResult, 1);
};

} // namespace MCFitter

#endif //#ifndef ATMCRESULT_H
