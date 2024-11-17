/*********************************************************************
 *
 *********************************************************************/

#ifndef _AtHDFUNPACKER_H_
#define _AtHDFUNPACKER_H_

#include "AtUnpacker.h"

#include <Rtypes.h>

#include <H5Cpp.h> // for hsize_t, herr_t
#include <H5Dpublic.h>
#include <H5Ipublic.h>
#include <H5Lpublic.h>
#include <H5Ppublic.h>
#include <H5Spublic.h>
#include <stdint.h>

#include <cstddef>
#include <string>
#include <tuple>
#include <vector>

class AtRawEvent;
class AtPad;
struct AtPadReference;
class TBuffer;
class TClass;
class TMemberInspector;

class AtHDFUnpacker : public AtUnpacker {

protected:
   Int_t fNumberTimestamps{};
   Bool_t fIsBaseLineSubtraction{};

   std::size_t fFirstEvent{};
   std::size_t fLastEvent{};

public:
   AtHDFUnpacker(mapPtr map);
   ~AtHDFUnpacker() = default;

   void SetBaseLineSubtraction(Bool_t value) { fIsBaseLineSubtraction = value; }
   void SetNumberTimestamps(int numTimestamps) { fNumberTimestamps = numTimestamps; };

   void Init() override;
   void FillRawEvent(AtRawEvent &event) override;
   bool IsLastEvent() override;
   Long64_t GetNumEvents() override;

protected:
   enum class IO_MODE { READ, WRITE };

   virtual std::size_t open(char const *file);
   virtual void setFirstAndLastEventNum();
   virtual void processData();
   virtual void processPad(std::size_t padIndex);
   virtual std::size_t n_pads(std::string i_raw_event);
   virtual std::vector<int16_t> pad_raw_data(std::size_t i_pad);
   hid_t open_file(char const *file, IO_MODE mode);
   std::tuple<hid_t, hsize_t> open_group(hid_t fileId, char const *group);
   std::tuple<hid_t, std::vector<hsize_t>>
   open_dataset(hid_t locId, char const *dataset); /// Returns the id of the dataspace and the dimesnions in the vector
   virtual void setEventIDAndTimestamps();

   void close_file(hid_t file);
   void close_group(hid_t group);
   void close_dataset(hid_t dataset);
   void end_raw_event();
   Float_t getBaseline(const std::vector<int16_t> &data);

   template <typename T>
   void read_slab(hid_t dataset, hsize_t *counts, hsize_t *offsets, hsize_t *dims_out, T *data)
   {
      hid_t dataspace = H5Dget_space(dataset);
      H5Sselect_hyperslab(dataspace, H5S_SELECT_SET, offsets, nullptr, counts, nullptr);
      hid_t memspace = H5Screate_simple(2, dims_out, nullptr);
      H5Dread(dataset, H5T_NATIVE_INT16, memspace, dataspace, H5P_DEFAULT, data);
      H5Sclose(memspace);
      H5Sclose(dataspace);
   }

   hid_t _file{};
   hid_t _group{}; /// The group that contains the events (get in original unpacker)
   hid_t _dataset{};
   std::vector<std::string> _eventsbyname;

   AtPad *createPadAndSetIsAux(const AtPadReference &padRef);
   void setDimensions(AtPad *pad);
   void setAdc(AtPad *pad, const std::vector<int16_t> &data);

   // Following methods satisfy the data_handler interface
   std::vector<uint64_t> get_header(std::string headerName);
   std::size_t datasets();
   static herr_t file_info(hid_t loc_id, const char *name, const H5L_info_t *linfo, void *opdata);
   void close();
   std::string get_event_name(std::size_t idx);
   ClassDefOverride(AtHDFUnpacker, 1);
};

#endif
