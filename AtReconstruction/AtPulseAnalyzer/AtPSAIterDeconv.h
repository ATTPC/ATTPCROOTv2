#ifndef AtPSAITERDECONV_H
#define AtPSAITERDECONV_H

#include "AtPSA.h"
#include "AtPSADeconv.h"

#include <Rtypes.h> // for Bool_t, THashConsistencyHolder, ClassDefOverride

#include <limits>
#include <memory> // for make_unique, unique_ptr

class AtPad;
class TBuffer;
class TClass;
class TMemberInspector;

/**
 * @brief Modifies AtPSADeconv to make iterative corrections to the output current.
 *
 */
class AtPSAIterDeconv : public AtPSADeconv {
private:
   int fIterations{0}; //< Number of iterations

public:
   AtPSAIterDeconv();
   virtual HitVector AnalyzePad(AtPad *pad) override;
   void SetIterations(int iterations) { fIterations = iterations; }

   ClassDefOverride(AtPSAIterDeconv, 1)
};

#endif
