/*********************************************************************
 *
 *********************************************************************/

#ifndef _AtFRIBHDFUNPACKER_H_
#define _AtFRIBHDFUNPACKER_H_

#include "AtHDFUnpacker.h"
#include "AtUnpacker.h" // for mapPtr

#include <Rtypes.h> // for THashConsistencyHolder, ClassDefOverride

#include <cstddef> // for size_t
#include <cstdint> // for int16_t
#include <string>  // for string
#include <vector>  // for vector

class TBuffer;
class TClass;
class TMemberInspector;

class AtFRIBHDFUnpacker : public AtHDFUnpacker {

public:
   AtFRIBHDFUnpacker(mapPtr map);
   ~AtFRIBHDFUnpacker() = default;

   void Init() override;

protected:
   std::size_t open(char const *file) override;
   void setFirstAndLastEventNum() override;
   void processData() override;
   void processPad(std::size_t padIndex) override;
   std::size_t n_pads(std::string i_raw_event) override;
   std::vector<int16_t> pad_raw_data(std::size_t i_pad) override;

   ClassDefOverride(AtFRIBHDFUnpacker, 1);
};

#endif
