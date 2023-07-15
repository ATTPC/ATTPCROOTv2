#include "AtFRIBHDFUnpacker.h"

#include "AtGenericTrace.h" // for AtGenericTrace
#include "AtRawEvent.h"

#include <FairLogger.h>

#include <Rtypes.h>
#include <TString.h>

#include <H5Cpp.h>     // for herr_t, hsize_t
#include <H5Gpublic.h> // for H5Giterate
#include <H5Ipublic.h> // for hid_t

#include <algorithm> // for max_element, min_element
#include <cstdint>   // for int16_t
#include <regex>     // for match_results<>::_Base_type, regex_replace
#include <sstream>   // for basic_stringbuf<>::int_type, basic_strin...
#include <tuple>     // for get
#include <utility>   // for __tuple_element_t

ClassImp(AtFRIBHDFUnpacker);

AtFRIBHDFUnpacker::AtFRIBHDFUnpacker(mapPtr map) : AtHDFUnpacker(map) {}

void AtFRIBHDFUnpacker::Init()
{
   auto numEvents = open(fInputFileName.c_str());
   auto uniqueEvents = GetNumEvents();
   LOG(info) << numEvents << " " << uniqueEvents << "\n";
   if (fEventID > uniqueEvents)
      LOG(fatal) << "Exceded valid range of event numbers. Looking for " << fEventID << " max event number is "
                 << uniqueEvents;
   if (uniqueEvents != numEvents / 3)
      LOG(error) << "Number of events from metaData does not match the number of entries in HDF5 file!";

   // Correct event ID for offset in file
   fDataEventID = fFirstEvent + fEventID;
}

std::size_t AtFRIBHDFUnpacker::open(char const *file)
{
   auto f = open_file(file, AtHDFUnpacker::IO_MODE::READ);
   if (f == 0)
      return 0;
   _file = f;

   auto group_n_entries = open_group(f, "frib/evt");
   if (std::get<0>(group_n_entries) == -1)
      return 0;
   _group = std::get<0>(group_n_entries);
   setFirstAndLastEventNum();
   return std::get<1>(group_n_entries);
}

void AtFRIBHDFUnpacker::processData()
{
   TString event_name = TString::Format("evt%lld_1903", fDataEventID);
   fRawEvent->SetEventName(event_name.Data());
   std::size_t npads = n_pads(event_name.Data());

   for (auto ipad = 0; ipad < npads; ++ipad)
      processPad(ipad);

   end_raw_event(); // Close dataset
}

void AtFRIBHDFUnpacker::processPad(std::size_t ipad)
{
   std::vector<int16_t> rawadc = pad_raw_data(ipad);

   auto trace = fRawEvent->AddGenericTrace(ipad);
   auto baseline = getBaseline(rawadc);
   for (Int_t iTb = 0; iTb < 2048; iTb++) {
      trace->SetRawADC(iTb, rawadc.at(iTb));
      trace->SetADC(iTb, rawadc.at(iTb) - baseline);
   }
}

std::vector<int16_t> AtFRIBHDFUnpacker::pad_raw_data(std::size_t i_pad)
{
   int16_t data[2048];
   hsize_t counts[2] = {2048, 1};
   hsize_t offsets[2] = {0, i_pad};
   hsize_t dims_out[2] = {2048, 1};
   read_slab<int16_t>(_dataset, counts, offsets, dims_out, data);
   std::vector<int16_t> datav(data, data + 2048);
   return datav;
}

std::size_t AtFRIBHDFUnpacker::n_pads(std::string i_raw_event)
{
   std::string dataset_name = i_raw_event;
   auto dataset_dims = open_dataset(_group, dataset_name.c_str());
   if (std::get<0>(dataset_dims) == 0)
      return 0;
   _dataset = std::get<0>(dataset_dims);
   return std::get<1>(dataset_dims)[1];
}

void AtFRIBHDFUnpacker::setFirstAndLastEventNum()
{
   // Look for the meta group and from it pull the minimum and maximum event numbers
   open_group(_file, "meta");

   // N.B. This function is adapted to the format of the FRIB DAQ data stored in the HDF5.
   auto addToVector = [](hid_t group, const char *name, void *op_data) -> herr_t {
      std::string text = std::string(name);
      if (text.find("header") != std::string::npos)
         return 0;
      std::regex regex("evt(\\d+)_\\d+");
      std::string result = std::regex_replace(text, regex, "$1\n");

      int number;
      std::istringstream iss(result);
      while (iss >> number) {
         LOG(debug) << number << std::endl;
      }

      // auto num = std::stoi(std::regex_replace(std::string(name), std::regex("[^0-9]"), ""));
      auto data = (std::vector<long> *)op_data; // NOLINT
      data->push_back(number);
      if (number == 0)
         LOG(info) << name << " to " << number << " " << data->size();
      return 0;
   };
   int idx = 0;
   std::vector<long> eventIDs;
   H5Giterate(_file, "frib/evt", &idx, addToVector, &eventIDs);
   fLastEvent = *std::max_element(eventIDs.begin(), eventIDs.end());
   fFirstEvent = *std::min_element(eventIDs.begin(), eventIDs.end());
   //}
   LOG(info) << "Events: " << fFirstEvent << " to " << fLastEvent;
}
