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

protected:
   std::string fFribPath = "/frib_physics/1903";

public:
   AtFRIBLinkedHDFUnpacker(mapPtr map) : AtHDFUnpacker(map){};
   ~AtFRIBLinkedHDFUnpacker() = default;

protected:
   virtual std::size_t open(char const *file) override;
   virtual void setFirstAndLastEventNum() override;
   virtual void setEventIDAndTimestamps() override;
   virtual void processData() override;
   // virtual void processPad(std::size_t padIndex) override;
   virtual std::size_t n_pads(std::string i_raw_event) override;
   std::size_t n_aux(std::string i_raw_event);
   void processAux(std::size_t auxIndex);
   // virtual std::vector<int16_t> pad_raw_data(std::size_t i_pad) override;

   ClassDefOverride(AtFRIBLinkedHDFUnpacker, 1);
};

#endif // ATFRIBLINKEDHDFUNPACKER_H
