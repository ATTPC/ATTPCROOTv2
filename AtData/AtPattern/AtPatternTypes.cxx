#include "AtPatternTypes.h"

#include "AtPatternCircle2D.h"
#include "AtPatternLine.h"

using namespace AtPatterns;

std::unique_ptr<AtPattern> AtPatterns::CreatePattern(PatternType type)
{

   switch (type) {
   case (PatternType::kLine): return std::make_unique<AtPatternLine>();
   case (PatternType::kCircle2D): return std::make_unique<AtPatternCircle2D>();
   default: return nullptr;
   }
}
