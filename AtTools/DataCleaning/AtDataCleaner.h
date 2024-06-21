#ifndef ATKNN_H
#define ATKNN_H

#include "AtHit.h"

#include <memory>
#include <vector>
class AtHit;

namespace AtTools {

namespace DataCleaning {

using HitCloud = std::vector<std::unique_ptr<AtHit>>;

/**
 * @brief Interface for data cleaning algorithms.
 * They take in a hit cloud and output a hit cloud.
 */
class AtDataCleaner {
public:
   virtual ~AtDataCleaner() = default;
   /**
    * @brief Clean the data.
    * @param hits The input hit cloud.
    * @return The cleaned hit cloud.
    */
   virtual HitCloud CleanData(const HitCloud &hits) = 0;
};

} // namespace DataCleaning

} // namespace AtTools

#endif // ATKNN_H