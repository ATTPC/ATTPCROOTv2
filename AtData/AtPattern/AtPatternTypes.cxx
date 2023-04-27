#include "AtPatternTypes.h"

#include "AtPatternCircle2D.h"
#include "AtPatternFission.h"
#include "AtPatternLine.h"
#include "AtPatternRay.h"
#include "AtPatternY.h"

using namespace AtPatterns;

std::unique_ptr<AtPattern> AtPatterns::CreatePattern(PatternType type)
{

   switch (type) {
   case (PatternType::kLine): return std::make_unique<AtPatternLine>();
   case (PatternType::kRay): return std::make_unique<AtPatternRay>();
   case (PatternType::kCircle2D): return std::make_unique<AtPatternCircle2D>();
   case (PatternType::kY): return std::make_unique<AtPatternY>();
   case (PatternType::kFission): return std::make_unique<AtPatternFission>();
   default: return nullptr;
   }
}
