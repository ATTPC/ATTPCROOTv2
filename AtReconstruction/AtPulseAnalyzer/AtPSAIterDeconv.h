#ifndef AtPSAITERDECONV_H
#define AtPSAITERDECONV_H

#include "AtPSA.h"
#include "AtPSADeconv.h"

#include <string>

class AtPad;

/**
 * @brief Modifies AtPSADeconv to make iterative corrections to the output current.
 *
 */
class AtPSAIterDeconv : public AtPSADeconv {
private:
   int fIterations{0};          //< Number of iterations
   std::string fQName{"Qreco"}; //< Name of the augment for the charge from iterations

public:
   virtual HitVector AnalyzePad(AtPad *pad) override;
   void RunPad(AtPad *pad);
   void SetIterations(int iterations) { fIterations = iterations; }
   void SetIterQName(std::string name) { fQName = name; }

   int GetIterations() { return fIterations; }
};

#endif
