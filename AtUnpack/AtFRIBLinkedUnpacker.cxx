#include "AtFRIBLinkedUnpacker.h"

#include "AtGenericTrace.h" // for AtGenericTrace
#include "AtMap.h"
#include "AtPad.h"
#include "AtPadReference.h"
#include "AtRawEvent.h"

#include <FairLogger.h>

#include <H5Apublic.h>
#include <H5Gpublic.h>
#include <H5Ppublic.h>

ClassImp(AtFRIBLinkedHDFUnpacker);

std::size_t AtFRIBLinkedHDFUnpacker::open(char const *file)
{
   auto f = open_file(file, AtHDFUnpacker::IO_MODE::READ);
   if (f == 0)
      return 0;
   _file = f;

   auto group_n_entries = open_group(f, "events");
   if (std::get<0>(group_n_entries) == -1)
      return 0;
   _group = std::get<0>(group_n_entries);
   setFirstAndLastEventNum();
   return std::get<1>(group_n_entries);
};

void AtFRIBLinkedHDFUnpacker::setFirstAndLastEventNum()
{
   // Assume that events are 0 indexed and contiguous
   fFirstEvent = 0;

   hsize_t num_objs;
   H5Gget_num_objs(_group, &num_objs);
   fLastEvent = num_objs - 1;
   LOG(debug) << "Events: " << fFirstEvent << " to " << fLastEvent;
};

void AtFRIBLinkedHDFUnpacker::setEventIDAndTimestamps()
{
   fRawEvent->SetEventID(fEventID);

   // Try to pull the timestamp
   try {

      // Open the dataset associated with the internal event id
      std::string dataset_name = TString::Format("event_%lld/get_traces", fDataEventID).Data();
      auto dataset_dims = open_dataset(_group, dataset_name.c_str());
      _dataset = std::get<0>(dataset_dims);

      auto _attr = H5Aopen(_dataset, "timestamp", H5P_DEFAULT);
      if (_attr < 0) {
         LOG(error) << "Could not open timestamp attribute for event " << fDataEventID;
         fRawEvent->SetNumberOfTimestamps(0);
         return;
      } else {
         unsigned long long timestamp;
         H5Aread(_attr, H5T_NATIVE_ULLONG, &timestamp);
         H5Aclose(_attr);
         LOG(debug) << "Setting timestamp " << timestamp << " for event " << fDataEventID;
         fRawEvent->SetNumberOfTimestamps(1);
         fRawEvent->SetTimestamp(timestamp, 0);
      }

      _attr = H5Aopen(_dataset, "timestamp_other", H5P_DEFAULT);
      if (_attr < 0) {
         LOG(error) << "Could not open timestamp_other attribute for event " << fDataEventID;
         return;
      } else {
         unsigned long long timestamp;
         H5Aread(_attr, H5T_NATIVE_ULLONG, &timestamp);
         H5Aclose(_attr);
         LOG(debug) << "Setting timestamp_other " << timestamp << " for event " << fDataEventID;
         fRawEvent->SetNumberOfTimestamps(2);
         fRawEvent->SetTimestamp(timestamp, 1);
      }

   } catch (const std::exception &e) {
      LOG(error) << "Failed to load the header, not setting timestamps.";
      fRawEvent->SetNumberOfTimestamps(0);
   }
}

std::size_t AtFRIBLinkedHDFUnpacker::n_pads(std::string i_raw_event)
{
   std::string dataset_name = i_raw_event + "/get_traces";
   auto dataset_dims = open_dataset(_group, dataset_name.c_str());
   if (std::get<0>(dataset_dims) == 0)
      return 0;
   _dataset = std::get<0>(dataset_dims);
   return std::get<1>(dataset_dims)[0];
};

std::size_t AtFRIBLinkedHDFUnpacker::n_aux(std::string i_raw_event)
{
   std::string dataset_name = i_raw_event + fFribPath;
   auto dataset_dims = open_dataset(_group, dataset_name.c_str());
   if (std::get<0>(dataset_dims) == 0)
      return 0;
   _dataset = std::get<0>(dataset_dims);
   return std::get<1>(dataset_dims)[1];
};

void AtFRIBLinkedHDFUnpacker::processAux(std::size_t padIndex)
{
   int16_t data[2048];
   hsize_t counts[2] = {2048, 1};
   hsize_t offsets[2] = {0, padIndex};
   hsize_t dims_out[2] = {2048, 1};
   read_slab<int16_t>(_dataset, counts, offsets, dims_out, data);
   std::vector<int16_t> rawadc(data, data + 2048);

   auto trace = fRawEvent->AddGenericTrace(padIndex);
   auto baseline = getBaseline(rawadc);
   for (Int_t iTb = 0; iTb < 2048; iTb++) {
      trace->SetRawADC(iTb, rawadc.at(iTb));
      trace->SetADC(iTb, rawadc.at(iTb) - baseline);

      if (padIndex == 0 && iTb > 2000)
         LOG(debug) << "Aux trace " << iTb << " " << rawadc.at(iTb);
   }
};

void AtFRIBLinkedHDFUnpacker::processData()
{
   TString event_name = TString::Format("event_%lld", fDataEventID);
   fRawEvent->SetEventName(event_name.Data());

   // Loop through and grab all of the pads in the event
   std::size_t npads = n_pads(event_name.Data());
   LOG(info) << "Unpacking " << npads << " pads in event " << fDataEventID;
   for (std::size_t i = 0; i < npads; i++) {
      processPad(i);
   }

   // Loop through and grab all of the generic traces in the event
   auto nAux = n_aux(event_name.Data());
   LOG(info) << "Unpacking " << nAux << " generic traces in event " << fDataEventID;
   for (auto i = 0; i < nAux; ++i)
      processAux(i);

   end_raw_event(); // Close dataset
};
