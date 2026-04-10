#include "AtMCResultOld.h"

#include <iostream>
ClassImp(MCFitter::AtMCResultOld);
namespace MCFitter {

void AtMCResultOld::Print() const

{
   std::cout << "Objective: " << fObjective << " Iteration: " << fIterNum << std::endl;
   for (auto &[name, val] : fParameters)
      std::cout << name << ": " << val << std::endl;
}
} // namespace MCFitter
