#ifndef ATMCRESULT_H
#define ATMCRESULT_H

#include "AtFitTrackMetadata.h"

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
class AtMCResult : public AtFitTrackMetadata {
public:
   using ParamMap = std::map<std::string, Double_t>;

   ParamMap fParameters; //< Parameters used in simulation
   Int_t fIterNum;       //< Iteration number. Used to map with the simulated event ID in the TTree.

   AtMCResult() = default;
   ~AtMCResult() = default;

   void Print() const override;

   ClassDefOverride(AtMCResult, 2);
};

} // namespace MCFitter

#endif // #ifndef ATMCRESULT_H
