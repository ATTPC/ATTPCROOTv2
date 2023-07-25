#ifndef ATPATTERNFISSION_H
#define ATPATTERNFISSION_H

#include "AtPatternY.h"

#include <Rtypes.h> // for THashConsistencyHolder, ClassDefOverride

#include <vector> // for vector
class TBuffer;
class TClass;
class TMemberInspector;

namespace AtPatterns {

/**
 * @brief Describes a fission event
 *
 * Is the same as AtPatternY but with a different minimization function for fitting.
 *
 * @ingroup AtPattern
 */
class AtPatternFission : public AtPatternY {
protected:
   virtual void FitPattern(const std::vector<XYZPoint> &points, const std::vector<double> &charge) override;

   ClassDefOverride(AtPatternFission, 1)
};
} // namespace AtPatterns
#endif
