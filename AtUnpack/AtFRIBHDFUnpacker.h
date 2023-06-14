/*********************************************************************
 *
 *********************************************************************/

#ifndef _AtFRIBHDFUNPACKER_H_
#define _AtFRIBHDFUNPACKER_H_

#include "AtHDFUnpacker.h"

class AtFRIBHDFUnpacker : public AtHDFUnpacker {

private:
public:
   AtFRIBHDFUnpacker(mapPtr map);
   ~AtFRIBHDFUnpacker() = default;

   void Init() override;
   std::size_t open(char const *file) override;
   void setFirstAndLastEventNum() override;
   void processData() override;
   void processPad(std::size_t padIndex) override;
   std::size_t n_pads(std::string i_raw_event) override;
   std::vector<int16_t> pad_raw_data(std::size_t i_pad) override;
   void setAdc(AtPad *pad, const std::vector<int16_t> &data) override;

   ClassDefOverride(AtFRIBHDFUnpacker, 1);
};

#endif
