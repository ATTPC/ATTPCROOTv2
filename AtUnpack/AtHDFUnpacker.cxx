/*********************************************************************
 *
 *********************************************************************/

#include "AtHDFUnpacker.h"

#include "AtAuxPad.h"
#include "AtMap.h"
#include "AtPad.h"
#include "AtPadReference.h"
#include "AtRawEvent.h"

#include <FairLogger.h>

#include <Rtypes.h>
#include <TString.h>

#include <H5Gpublic.h>
#include <H5Ppublic.h>

#include <algorithm> // for max_element, min_element
#include <cstdint>
#include <exception> // for exception
#include <iostream>
#include <memory>
#include <regex>
#include <sstream>   // for basic_stringbuf<>::int_type, basic_strin...
#include <stdexcept> // for out_of_range
#include <string>
#include <type_traits>
#include <utility>

ClassImp(AtHDFUnpacker);

AtHDFUnpacker::AtHDFUnpacker(mapPtr map) : AtUnpacker(map) {}

void AtHDFUnpacker::Init()
{
   auto numEvents = open(fInputFileName.c_str());
   auto uniqueEvents = GetNumEvents();
   if (fEventID > uniqueEvents)
      LOG(fatal) << "Exceded valid range of event numbers. Looking for " << fEventID << " max event number is "
                 << uniqueEvents;
   if (uniqueEvents != numEvents / 2)
      LOG(error) << "Number of events from metaData does not match the number of entries in HDF5 file!";

   // Correct event ID for offset in file
   fDataEventID = fFirstEvent + fEventID;
}

void AtHDFUnpacker::FillRawEvent(AtRawEvent &event)
{
   LOG(debug) << " Unpacking event ID: " << fEventID << " with internal ID " << fDataEventID;
   fRawEvent = &event;
   setEventIDAndTimestamps();
   processData();

   fRawEvent->SetIsGood(kTRUE);
   LOG(debug) << " Unpacked " << fRawEvent->GetNumPads() << " pads";
   fEventID++;
   fDataEventID++;
}
Long64_t AtHDFUnpacker::GetNumEvents()
{
   return fLastEvent - fFirstEvent + 1;
}
bool AtHDFUnpacker::IsLastEvent()
{
   return fDataEventID >= fLastEvent;
}

void AtHDFUnpacker::setEventIDAndTimestamps()
{
   fRawEvent->SetEventID(fEventID);

   try {
      TString header_name = TString::Format("evt%lld_header", fDataEventID);
      auto header = get_header(header_name.Data());

      fRawEvent->SetNumberOfTimestamps(fNumberTimestamps);
      for (int i = 0; i < fNumberTimestamps; ++i) {
         fRawEvent->SetTimestamp(header.at(i + 1), i);
      }
   } catch (const std::out_of_range &e) {
      LOG(error) << "Expected " << fNumberTimestamps << " but the header is not long enough!";
   } catch (const std::exception &e) {
      LOG(debug) << "Failed to load the header, not setting timestamps.";
      fRawEvent->SetNumberOfTimestamps(0);
   }
}
void AtHDFUnpacker::processData()
{
   TString event_name = TString::Format("evt%lld_data", fDataEventID);
   // fRawEvent->SetEventName(event_name.Data());
   LOG(debug) << fRawEvent->GetEventName() << "\n";
   std::size_t npads = n_pads(event_name.Data());

   for (auto ipad = 0; ipad < npads; ++ipad)
      processPad(ipad);

   end_raw_event(); // Close dataset
}

void AtHDFUnpacker::processPad(std::size_t ipad)
{
   std::vector<int16_t> rawadc = pad_raw_data(ipad);
   AtPadReference PadRef = {rawadc[0], rawadc[1], rawadc[2], rawadc[3]};

   auto pad = createPadAndSetIsAux(PadRef);
   setDimensions(pad);
   setAdc(pad, rawadc);
}
AtPad *AtHDFUnpacker::createPadAndSetIsAux(const AtPadReference &padRef)
{
   if (fSaveFPN && fMap->IsFPNchannel(padRef))
      return fRawEvent->AddFPN(padRef);

   if (fMap->IsAuxPad(padRef)) {
      auto padName = fMap->GetAuxName(padRef);
      return fRawEvent->AddAuxPad(padName).first;
   }

   auto padNumber = fMap->GetPadNum(padRef);
   return fRawEvent->AddPad(padNumber);
}
void AtHDFUnpacker::setAdc(AtPad *pad, const std::vector<int16_t> &data)
{
   auto baseline = getBaseline(data);
   for (Int_t iTb = 0; iTb < 512; iTb++) {
      pad->SetRawADC(iTb, data.at(iTb + 5)); // First 5 words are electronic id
      pad->SetADC(iTb, data.at(iTb + 5) - baseline);
   }
   pad->SetPedestalSubtracted(fIsBaseLineSubtraction);
}

Float_t AtHDFUnpacker::getBaseline(const std::vector<int16_t> &data)
{
   Float_t baseline = 0;

   if (fIsBaseLineSubtraction) {
      for (Int_t iTb = 5; iTb < 25; iTb++) // First 5 words are electronic id
         baseline += data[iTb];
      baseline /= 20.0;
   }
   return baseline;
}
void AtHDFUnpacker::setDimensions(AtPad *pad)
{
   auto PadCenterCoord = fMap->CalcPadCenter(pad->GetPadNum());
   Int_t pSizeID = fMap->GetPadSize(pad->GetPadNum());
   pad->SetPadCoord(PadCenterCoord);
   pad->SetSizeID(pSizeID);
}

hid_t AtHDFUnpacker::open_file(char const *file, IO_MODE mode)
{
   hid_t fileId;
   (mode == IO_MODE::READ) ? fileId = H5Fopen(file, H5F_ACC_RDONLY, H5P_DEFAULT)
                           : fileId = H5Fopen(file, H5F_ACC_RDWR, H5P_DEFAULT);

   if (fileId >= 0) {
      std::cout << "> hdf5_wrapper::open_file:MESSAGE, opening file: " << file << ", ID: " << fileId << '\n';

      return fileId;
   } else {
      std::cerr << "> AtHDFUnpacker::open_file:ERROR, invalid ID for file: " << file << '\n';
      return 0;
   }
}

std::tuple<hid_t, hsize_t> AtHDFUnpacker::open_group(hid_t fileId, char const *group)
{
   hid_t groupId = H5Gopen2(fileId, group, H5P_DEFAULT);
   if (groupId >= 0) {
      // std::cout << "> hdf5_wrapper::open_group:MESSAGE, opening group: " << group << ", ID: " << groupId << '\n';
      hsize_t size;
      H5Gget_num_objs(groupId, &size);
      return std::make_tuple(groupId, size);

   } else {
      std::cerr << "> AtHDFUnpacker::open_group:ERROR, invalid ID for group: " << group << '\n';
      return std::make_tuple(-1, -1);
   }
}

std::tuple<hid_t, std::vector<hsize_t>> AtHDFUnpacker::open_dataset(hid_t locId, char const *dataset)
{

   hid_t datasetId = H5Dopen2(locId, dataset, H5P_DEFAULT);
   if (datasetId >= 0) {
      // std::cout << "> hdf5_wrapper::open_dataset:MESSAGE, opening dataset: " << dataset << ", ID: " << datasetId <<
      // '\n';
      hid_t dspaceId = H5Dget_space(datasetId);
      int n_dims = H5Sget_simple_extent_ndims(dspaceId);
      hsize_t dims[n_dims];
      H5Sget_simple_extent_dims(dspaceId, dims, nullptr);
      std::vector<hsize_t> v_dims(dims, dims + n_dims);
      return std::make_tuple(datasetId, v_dims);
   } else {
      std::cerr << "> AtHDFUnpacker::open_dataset:ERROR, invalid ID for dataset: " << dataset << '\n';
      std::vector<hsize_t> v{0};
      return std::make_tuple(0, v);
   }
}

void AtHDFUnpacker::close_file(hid_t fileId)
{
   herr_t retId = H5Fclose(fileId);
   if (retId < 0)
      std::cerr << "> AtHDFUnpacker::close_file:ERROR, cannot close file with ID: " << fileId << '\n';
}

void AtHDFUnpacker::close_group(hid_t groupId)
{
   herr_t retId = H5Gclose(groupId);
   if (retId < 0)
      std::cerr << "> AtHDFUnpacker::close_group:ERROR, cannot close group with ID: " << groupId << '\n';
}

void AtHDFUnpacker::close_dataset(hid_t datasetId)
{
   herr_t retId = H5Dclose(datasetId);
   if (retId < 0)
      std::cerr << "> AtHDFUnpacker::close_dataset:ERROR, cannot close dataset with ID: " << datasetId << '\n';
}

void AtHDFUnpacker::setFirstAndLastEventNum()
{
   // Look for the meta group and from it pull the minimum and maximum event numbers
   auto meta_size = open_group(_file, "meta");
   auto metaID = std::get<0>(meta_size);
   if (metaID > 0) {
      std::string datasetName = "meta";
      auto dataset_dims = open_dataset(metaID, datasetName.c_str());
      auto datasetId = std::get<0>(dataset_dims);
      auto len = std::get<1>(dataset_dims).at(0);

      auto *data = new int64_t[len]; // NOLINT
      H5Dread(datasetId, H5T_NATIVE_ULONG, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);

      fFirstEvent = data[0];
      fLastEvent = data[2];

      delete[] data; // NOLINT
   } else {

      // If there is not the meta data then we need to look through every event in the
      // file and pull out the first and last event numbers
      auto addToVector = [](hid_t group, const char *name, void *op_data) -> herr_t {
         auto num = std::stoi(std::regex_replace(std::string(name), std::regex("[^0-9]"), ""));
         auto data = (std::vector<long> *)op_data; // NOLINT
         data->push_back(num);
         LOG(debug) << name << " to " << num << " " << data->size();
         return 0;
      };
      int idx = 0;
      std::vector<long> eventIDs;
      H5Giterate(_file, "get", &idx, addToVector, &eventIDs);
      fLastEvent = *std::max_element(eventIDs.begin(), eventIDs.end());
      fFirstEvent = *std::min_element(eventIDs.begin(), eventIDs.end());
   }
   LOG(info) << "Events: " << fFirstEvent << " to " << fLastEvent;
}

/**
 * @brief Open the file and sets the first/last event number in the file
 *
 * @return the number of events in the file
 */
std::size_t AtHDFUnpacker::open(char const *file)
{
   auto f = open_file(file, AtHDFUnpacker::IO_MODE::READ);
   if (f == 0)
      return 0;
   _file = f;

   auto group_n_entries = open_group(f, "get");
   if (std::get<0>(group_n_entries) == -1)
      return 0;
   _group = std::get<0>(group_n_entries);
   setFirstAndLastEventNum();
   return std::get<1>(group_n_entries);
}

std::string AtHDFUnpacker::get_event_name(std::size_t idx)
{
   if (_group >= 0) {
      char name[100];
      H5Lget_name_by_idx(_group, ".", H5_INDEX_NAME, H5_ITER_INC, idx, name, 100, H5P_DEFAULT);
      std::string name_string(name);
      return name_string;
   } else {
      std::cerr << "> AtHDFUnpacker::get_event_name:ERROR, invalid ID  " << idx << '\n';
      return "invalid";
   }
}

std::vector<uint64_t> AtHDFUnpacker::get_header(std::string headerName)
{
   std::vector<uint64_t> retVec;

   auto dataset_dims = open_dataset(_group, headerName.c_str());
   if (std::get<0>(dataset_dims) == 0)
      return retVec;

   _dataset = std::get<0>(dataset_dims);

   // Get the length of the header
   auto len = std::get<1>(dataset_dims).at(0);

   retVec.resize(len);
   // auto *data = new uint64_t[len];
   H5Dread(_dataset, H5T_NATIVE_ULONG, H5S_ALL, H5S_ALL, H5P_DEFAULT, retVec.data());

   close_dataset(_dataset);
   // Add read data to the vector and return it
   // for (int i = 0; i < len; ++i)
   // retVec.push_back(data[i]);

   // delete[] data;

   return retVec;
}

std::size_t AtHDFUnpacker::n_pads(std::string i_raw_event)
{
   std::string dataset_name = i_raw_event;
   auto dataset_dims = open_dataset(_group, dataset_name.c_str());
   if (std::get<0>(dataset_dims) == 0)
      return 0;
   _dataset = std::get<0>(dataset_dims);
   return std::get<1>(dataset_dims)[0];
}

std::vector<int16_t> AtHDFUnpacker::pad_raw_data(std::size_t i_pad)
{
   int16_t data[517];
   hsize_t counts[2] = {1, 517};
   hsize_t offsets[2] = {i_pad, 0};
   hsize_t dims_out[2] = {1, 517};
   read_slab<int16_t>(_dataset, counts, offsets, dims_out, data);
   std::vector<int16_t> datav(data, data + 517);
   return datav;
}

/*std::size_t AtHDFUnpacker::inievent()
{
   return _inievent;
}
*/

std::size_t AtHDFUnpacker::datasets()
{
   H5Literate(_group, H5_INDEX_NAME, H5_ITER_INC, nullptr, file_info, nullptr);
   return 0;
}

herr_t AtHDFUnpacker::file_info(hid_t loc_id, const char *name, const H5L_info_t *linfo, void *opdata)
{
   hid_t group;
   /*
    * Open the group using its name.
    */
   group = H5Gopen2(loc_id, name, H5P_DEFAULT);
   /*
    * Display group name.
    */
   std::cout << "Name : " << name << "\n";
   H5Gclose(group);
   return 0;
}

void AtHDFUnpacker::end_raw_event()
{
   close_dataset(_dataset);
}

void AtHDFUnpacker::close()
{
   close_group(_group);
   close_file(_file);
}
