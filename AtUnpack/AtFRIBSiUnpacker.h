#ifndef ATFRIBSILUNPACKER_H
#define ATFRIBSILUNPACKER_H

#include "AtFRIBLinkedUnpacker.h"
/**
 * @brief  Unpacker for HDF5 files with FRIB already linked by timestamp with si channels.
 *
 * This class is used to unpack data from HDF5 files that have already been linked by timestamp
 * and are using the FRIB DAQ to load in any "aux" channels with the 2028 sample digitizer.
 *
 * It provides additional support for processing the Si channels added to the `get` tag in the
 * HDF5 file that are not part of the standard FRIB linked unpacker file ouput.
 */

class AtFRIBSiUnpacker : public AtFRIBLinkedHDFUnpacker {
protected:
   std::vector<std::string> fSiPaths = {"si_downstream_back", "si_downstream_front", "si_upstream_back",
                                        "si_upstream_front"}; // Paths to the FRIB DAQ channels
public:
   AtFRIBSiUnpacker(mapPtr map) : AtFRIBLinkedHDFUnpacker(map)
   {
      fFribPaths = {"977", "1903", "1904", "1905", "1906"};
      fGetPath = "/get";
   };
   ~AtFRIBSiUnpacker() = default;

protected:
   // virtual std::size_t open(char const *file) override;
   // virtual void setFirstAndLastEventNum() override;
   // virtual void setEventIDAndTimestamps() override;
   virtual void processData() override;
   //  virtual void processPad(std::size_t padIndex) override;
   virtual std::size_t n_pads(std::string i_raw_event) override;
   // std::size_t n_aux(std::string i_raw_event);
   // void processAux(std::size_t auxIndex);
   void processSiChannel(std::size_t chIndex);

   ClassDefOverride(AtFRIBSiUnpacker, 1);
};

#endif // ATFRIBSILUNPACKER_H
