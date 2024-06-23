#ifndef ATKNN_H
#define ATKNN_H

#include "AtDataCleaner.h"

namespace AtTools {

namespace DataCleaning {

/**
 * kNN algorithm as implemented in AtPRA class.
 *
 * Will reject any point whose average distance to its k nearest neighbors is greater than a threshold.
 */
class AtkNN : public AtDataCleaner {
protected:
   int fkNN;        //<! Number of nearest neighbors to consider
   double fkNNDist; //<! Distance threshold for outlier rejection in kNN
public:
   AtkNN(int kNN, double kNNDist) : fkNN(kNN), fkNNDist(kNNDist) {}
   HitCloud CleanData(const HitCloud &hits) override;

   bool kNN(const HitCloud &hits, AtHit &hitRef);
};

} // namespace DataCleaning

} // namespace AtTools

#endif // ATKNN_HH
