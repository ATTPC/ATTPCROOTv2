#ifndef ATREACTIONGENERATOR_H
#define ATREACTIONGENERATOR_H

#include <FairGenerator.h>

/**
 * This class represents a generator for reaction-like events.
 * When the ReadEvent method is called, this generator can assume
 * that it's parent vertex is stored in the AtVertexPropagator singleton.
 *
 */
class AtReactionGenerator : public FairGenerator {
protected:
   bool kIsFinalGen{true}; ///< Flag to indicate if this generator is the final one in the chain
   bool kAlwaysRun{false}; ///< Flag to indicate if this generator should always run
public:
   virtual Bool_t ReadEvent(FairPrimaryGenerator *primGen) final;
   void SetSequentialDecay(Bool_t var) { kIsFinalGen = !var; }
   void SetAlwaysRun(Bool_t var) { kAlwaysRun = var; }

protected:
   virtual bool GenerateReaction(FairPrimaryGenerator *primGen) = 0;
};

#endif // ATREACTIONGENERATOR_H
