#ifndef ATKNN_H
#define ATKNN_H

#include "AtHit.h"
#include <vector>
#include <memory>
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
        

            virtual HitCloud CleanData(const HitCloud &hits) = 0;
        };
            
        }

    }

#endif //ATKNN_H