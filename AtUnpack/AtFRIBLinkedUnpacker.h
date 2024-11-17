#ifndef ATFRIBLINKEDHDFUNPACKER_H
#define ATFRIBLINKEDHDFUNPACKER_H

#include "AtHDFUnpacker.h"
/**
 * @brief  Unpacker for HDF5 files with FRIB already linked by timestamp.
 *
 * This class is used to unpack data from HDF5 files that have already been linked by timestamp
 * and are using the FRIB DAQ to load in any "aux" channels with the 2024 sample digitizer.
 */

class AtFRIBLinkedHDFUnpacker : public AtHDFUnpacker {
public:
   AtFRIBLinkedHDFUnpacker(mapPtr map) : AtHDFUnpacker(map) {};
   ~AtFRIBLinkedHDFUnpacker() = default;

   void Init() override;

protected:
   virtual std::size_t open(char const *file) override;
   virtual void setFirstAndLastEventNum() override;
   // virtual void processData() override;
   // virtual void processPad(std::size_t padIndex) override;
   // virtual std::size_t n_pads(std::string i_raw_event) override;
   // virtual std::vector<int16_t> pad_raw_data(std::size_t i_pad) override;

   ClassDefOverride(AtFRIBLinkedHDFUnpacker, 1);
};

#endif // ATFRIBLINKEDHDFUNPACKER_H
