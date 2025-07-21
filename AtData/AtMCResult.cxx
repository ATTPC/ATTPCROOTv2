#include "AtMCResult.h"

#include <iostream>
ClassImp(MCFitter::AtMCResult);
namespace MCFitter {

void AtMCResult::Print() const

{
   AtFitTrackResult::Print();
   std::cout << " MC fit specifics: " << std::endl;

   std::cout << "   Iteration = " << fIterNum << std::endl;
   std::cout << "   Parameters:" << std::endl;
   for (auto &[name, val] : fParameters)
      std::cout << "      " << name << " = " << val << std::endl;
}
} // namespace MCFitter
