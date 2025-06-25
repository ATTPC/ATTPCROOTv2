#include "AtFRIBLinkedUnpacker.h"

#include "AtGenericTrace.h" // for AtGenericTrace
#include "AtMap.h"
#include "AtPad.h"
#include "AtPadReference.h"
#include "AtRawEvent.h"

#include <FairLogger.h>

#include <H5Apublic.h>
#include <H5Gpublic.h>
#include <H5Opublic.h>
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
      std::string obj_name = TString::Format("event_%lld", fDataEventID).Data() + fGetPath;
      LOG(info) << "Looking for dataset or group " << obj_name << " for event " << fDataEventID;
      hid_t _objID = -1;

      H5O_info_t get_info;
      auto status = H5Oget_info_by_name(_group, obj_name.c_str(), &get_info, H5P_DEFAULT);

      if (status < 0) {
         LOG(warning) << "Could not find object " << obj_name << " in group " << _group;
         fRawEvent->SetNumberOfTimestamps(0);
         return;
      }

      switch (get_info.type) {
      case H5O_TYPE_DATASET: {
         auto dataset_dims = open_dataset(_group, obj_name.c_str());
         _dataset = std::get<0>(dataset_dims);
         _objID = _dataset;
         break;
      }

      case H5O_TYPE_GROUP: {
         auto group_dims = open_group(_group, obj_name.c_str());
         _objID = std::get<0>(group_dims);
         break;
      }

      default:
         LOG(warning) << "Could not find object " << obj_name << " in group " << _group;
         fRawEvent->SetNumberOfTimestamps(0);
         return;
      }

      LOG(info) << "Opened object ID " << _objID << " for event " << fDataEventID;

      auto _attr = H5Aopen(_objID, "timestamp", H5P_DEFAULT);
      if (_attr < 0) {
         LOG(error) << "Could not open timestamp attribute for event " << fDataEventID;
         fRawEvent->SetNumberOfTimestamps(0);
         if (get_info.type == H5O_TYPE_GROUP) {
            H5Gclose(_objID);
         }
         return;
      } else {
         unsigned long long timestamp;
         H5Aread(_attr, H5T_NATIVE_ULLONG, &timestamp);
         H5Aclose(_attr);
         LOG(debug) << "Setting timestamp " << timestamp << " for event " << fDataEventID;
         fRawEvent->SetNumberOfTimestamps(1);
         fRawEvent->SetTimestamp(timestamp, 0);
      }

      _attr = H5Aopen(_objID, "timestamp_other", H5P_DEFAULT);
      if (_attr < 0) {
         LOG(error) << "Could not open timestamp_other attribute for event " << fDataEventID;
         if (get_info.type == H5O_TYPE_GROUP) {
            H5Gclose(_objID);
         }
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
   return n_entries(i_raw_event + "/get_traces")[0];
};

std::size_t AtFRIBLinkedHDFUnpacker::n_aux(std::string i_raw_event)
{
   std::string fFribPath = "/frib_physics/1903";
   std::string dataset_name = i_raw_event + fFribPath;
   return n_entries(dataset_name)[1]; // These are trace x channel so index is 1
};

void AtFRIBLinkedHDFUnpacker::processAux(std::size_t padIndex, std::size_t nTB)
{
   u_int16_t data[nTB];
   hsize_t counts[2] = {nTB, 1};
   hsize_t offsets[2] = {0, padIndex};
   hsize_t dims_out[2] = {nTB, 1};
   read_slab<u_int16_t>(_dataset, counts, offsets, dims_out, data);
   std::vector<u_int16_t> rawadc(data, data + nTB);

   auto trace = fRawEvent->AddGenericTrace(padIndex, nTB);
   for (Int_t iTb = 0; iTb < nTB; iTb++) {
      trace->SetRawADC(iTb, rawadc.at(iTb));
      trace->SetADC(iTb, rawadc.at(iTb));

      if (padIndex == 0 && iTb > nTB - 48)
         LOG(debug) << "Aux trace " << iTb << " " << rawadc.at(iTb);
   }
};

void AtFRIBLinkedHDFUnpacker::processSIS(std::string i_raw_event, std::string name)
{
   // Open the dataset for the SIS digitizer
   std::string dataset_name = i_raw_event + "/frib_physics/" + name;
   auto dims = n_entries(dataset_name);
   std::size_t nTB = 0;
   std::size_t nChannels = 0;
   if (dims.size() == 1) {
      nTB = 1;
      nChannels = 1;
   } else {
      nTB = dims.at(0);
      nChannels = dims.at(1);
   }

   LOG(info) << "Processing SIS digitizer " << name << " with " << nChannels << " channels and " << nTB
             << " time bins.";

   for (auto i = 0; i < nChannels; ++i)
      processAux(i, nTB);
}

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
   for (auto &sis : fFribPaths) {
      processSIS(event_name.Data(), sis);
   }

   end_raw_event(); // Close dataset
};
