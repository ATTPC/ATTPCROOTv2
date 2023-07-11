#ifndef ATPATTERNTYPES_H
#define ATPATTERNTYPES_H

#include <memory>

namespace AtPatterns {

/**
 * @brief Supported patterns.
 *
 * Can be created with the static factory AtPatterns::CreatePattern(PatternType type)
 * @ingroup AtPattern
 */
enum class PatternType { kLine, kRay, kCircle2D, kY, kFission };

class AtPattern;
/**
 * @brief Factory for AtPattern.
 *
 * Factory method for creating instances of AtPattern based on type
 * @ingroup AtPattern
 */
std::unique_ptr<AtPattern> CreatePattern(PatternType type);
/*{

   switch (type) {
   case (PatternType::kLine): return std::make_unique<AtPatternLine>();
   case (PatternType::kCircle2D): return std::make_unique<AtPatternCircle2D>();
   default: return nullptr;
   }
}
*/

} // namespace AtPatterns
#endif //#ifndef ATPATTERNTYPES_H
