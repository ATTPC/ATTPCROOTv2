#include "AtMCResult.h"

#include <iostream>
ClassImp(MCFitter::AtMCResult);
namespace MCFitter {

void AtMCResult::Print() const

{
   std::cout << "Objective: " << fObjective << " Iteration: " << fIterNum << std::endl;
   for (auto &[name, val] : fParameters)
      std::cout << name << ": " << val << std::endl;
}
} // namespace MCFitter
