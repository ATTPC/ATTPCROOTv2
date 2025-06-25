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
   std::vector<std::string> fFribPaths = {"1903"};
   std::string fGetPath = "/get_traces"; // Path to the get object containing meta data and traces

public:
   AtFRIBLinkedHDFUnpacker(mapPtr map) : AtHDFUnpacker(map){};
   ~AtFRIBLinkedHDFUnpacker() = default;

   void SetFribPaths(const std::vector<std::string> &paths) { fFribPaths = paths; }
   void AddFribPath(const std::string &path) { fFribPaths.push_back(path); }
   const std::vector<std::string> &GetFribPaths() const { return fFribPaths; }

protected:
   virtual std::size_t open(char const *file) override;
   virtual void setFirstAndLastEventNum() override;
   virtual void setEventIDAndTimestamps() override;
   virtual void processData() override;

   // virtual void processPad(std::size_t padIndex) override;
   virtual std::size_t n_pads(std::string i_raw_event) override;

   /**
    * @brief Get the number of auxiliary (FRIB_DAQ) channels in the event.
    *
    * These are the channels that live in the frib_physics group of the HDF5 file.
    * All groups specified in fFribPaths will be checked.
    */
   std::size_t n_aux(std::string i_raw_event);

   /**
    * @brief Process an auxiliary channel in the frib_physics group.
    * @param auxIndex Index of the auxiliary channel to process.
    * @param nTB Number of time bins in the auxiliary channel.
    */
   void processAux(std::size_t auxIndex, std::size_t nTB);

   /**
    * @brief Process an SIS digitizer in the frib_physics group.
    *
    * @param name Name of the SIS digitizer to process eg 1903.
    *
    * This will save the data as generic traces in the AtRawEvent.
    */
   void processSIS(std::string i_raw_event, std::string name);
   // virtual std::vector<int16_t> pad_raw_data(std::size_t i_pad) override;

   ClassDefOverride(AtFRIBLinkedHDFUnpacker, 1);
};

#endif // ATFRIBLINKEDHDFUNPACKER_H
