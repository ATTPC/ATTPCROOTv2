/*********************************************************************
* 
*********************************************************************/

#ifndef _ATHDFPARSER_H_
#define _ATHDFPARSER_H_

#include "TObject.h"
#include "TROOT.h"

#include <hdf5.h>

#include <memory>
#include <tuple>
#include <vector>

#include <iostream>

class ATHDFParser : public TObject {

public:

  ATHDFParser();
  ~ATHDFParser();

  enum class IO_MODE {
    READ,
      WRITE
      };
  hid_t open_file(char const* file, IO_MODE mode);
  std::tuple<hid_t, hsize_t> open_group(hid_t fileId, char const* group);
  std::tuple<hid_t, std::vector<hsize_t> > open_dataset(hid_t locId, char const* dataset);
  void close_file(hid_t file);
  void close_group(hid_t group);
  void close_dataset(hid_t dataset);

  template<typename T>
  void read_slab(hid_t dataset, hsize_t* counts, hsize_t* offsets, hsize_t* dims_out, T* data)
    {
      hid_t dataspace = H5Dget_space(dataset);
      H5Sselect_hyperslab(dataspace, H5S_SELECT_SET, offsets, nullptr, counts, nullptr);
      hid_t memspace = H5Screate_simple(2, dims_out, nullptr);
      H5Dread(dataset, H5T_NATIVE_INT16, memspace, dataspace, H5P_DEFAULT, data);
      H5Sclose(memspace);
      H5Sclose(dataspace);
    }

  // Following methods satisfy the data_handler interface
  std::size_t open(char const* file);
  std::size_t n_pads(std::string i_raw_event);
  std::vector<int16_t> pad_raw_data(std::size_t i_pad);
  std::vector<int64_t> get_header(std::string headerName);
  std::size_t datasets();
  std::size_t inievent();
  static herr_t file_info(hid_t loc_id, const char *name, const H5L_info_t *linfo, void *opdata);
  void end_raw_event();
  void close();

  std::vector<std::string> get_events_by_name() {return _eventsbyname;}
  void set_inievent(std::size_t inievent)       {_inievent = inievent;}
  std::string get_event_name(std::size_t idx);

private:

  hid_t               _file;
  hid_t       	      _group;
  hid_t       	      _dataset;
  std::size_t         _inievent;
  std::vector<std::string>  _eventsbyname;
	 
   

  ClassDef(ATHDFParser, 1);
};

#endif
