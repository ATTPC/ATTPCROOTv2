/*********************************************************************
* 
*********************************************************************/

#include <iostream>
#include "ATHDFParser.hh"

ClassImp(ATHDFParser);

	
ATHDFParser::ATHDFParser(){
 ;	
}	

ATHDFParser::~ATHDFParser(){
 ;	
}

hid_t ATHDFParser::open_file(char const* file, IO_MODE mode)
{
    hid_t fileId;
    (mode == IO_MODE::READ) ?
      fileId = H5Fopen(file, H5F_ACC_RDONLY, H5P_DEFAULT) :
      fileId = H5Fopen(file, H5F_ACC_RDWR, H5P_DEFAULT);

    if (fileId >= 0)
    {
      std::cout << "> hdf5_wrapper::open_file:MESSAGE, opening file: " << file << ", ID: " << fileId << '\n';

      return fileId;
    }
    else
    {
      std::cerr << "> ATHDFParser::open_file:ERROR, invalid ID for file: " << file << '\n';
      return 0;
    }
}

std::tuple<hid_t, hsize_t> ATHDFParser::open_group(hid_t fileId, char const* group)
{
    hid_t groupId = H5Gopen2(fileId, group, H5P_DEFAULT);
    if (groupId >= 0)
    {
      //std::cout << "> hdf5_wrapper::open_group:MESSAGE, opening group: " << group << ", ID: " << groupId << '\n';
      hsize_t size;
      H5Gget_num_objs(groupId, &size);

      //Determining the first event
      /*std::vector<unsigned long int> events;
      int cnt = 0;
      
      for(auto isize=0;isize<size;++isize)
      {
	      char name[100];
	      H5Lget_name_by_idx(groupId,".",H5_INDEX_NAME, H5_ITER_INC,isize,name,100, H5P_DEFAULT);
	      events.push_back(std::atoi(name));
        std::string name_string(name);
        std::cout<<name_string<<" "<<groupId<<"\n";
        _eventsbyname.push_back(name_string);
	      ++cnt;
	  }

	 std::vector<unsigned long int>::iterator result = std::min_element(std::begin(events), std::end(events));
     _inievent = events.at(std::distance(std::begin(events), result));

     //std::cout<<" Minimum "<<_inievent<<" Position "<<std::distance(std::begin(events), result)<<" Size "<<events.size()<<"\n";*/

      return std::make_tuple(groupId, size);
    }
    else
    {
      std::cerr << "> ATHDFParser::open_group:ERROR, invalid ID for group: " << group << '\n';
      return std::make_tuple(-1, -1);
    }
 }

 std::tuple<hid_t, std::vector<hsize_t> > ATHDFParser::open_dataset(hid_t locId, char const* dataset)
 {

    hid_t datasetId = H5Dopen2(locId, dataset, H5P_DEFAULT);
    if (datasetId >= 0)
    {
      //std::cout << "> hdf5_wrapper::open_dataset:MESSAGE, opening dataset: " << dataset << ", ID: " << datasetId << '\n';
      hid_t dspaceId = H5Dget_space(datasetId);
      int n_dims = H5Sget_simple_extent_ndims(dspaceId);
      hsize_t dims[n_dims];
      H5Sget_simple_extent_dims(dspaceId, dims, nullptr);
      std::vector<hsize_t> v_dims(dims,dims + n_dims); 
      return std::make_tuple(datasetId,v_dims);
    }
    else
    {
      std::cerr << "> ATHDFParser::open_dataset:ERROR, invalid ID for dataset: " << dataset << '\n';
      std::vector<hsize_t> v{0};
      return std::make_tuple(0,v);
    }
}

void ATHDFParser::close_file(hid_t fileId)
{
    herr_t retId = H5Fclose(fileId);
    if (retId < 0)
      std::cerr << "> ATHDFParser::close_file:ERROR, cannot close file with ID: " << fileId << '\n';
}

void ATHDFParser::close_group(hid_t groupId)
{
    herr_t retId = H5Gclose(groupId);
    if (retId < 0)
      std::cerr << "> ATHDFParser::close_group:ERROR, cannot close group with ID: " << groupId << '\n';
}

void ATHDFParser::close_dataset(hid_t datasetId)
{
    herr_t retId = H5Dclose(datasetId);
    if (retId < 0)
      std::cerr << "> ATHDFParser::close_dataset:ERROR, cannot close dataset with ID: " << datasetId << '\n';
}

std::size_t ATHDFParser::open(char const* file)
{
    auto f = open_file(file, ATHDFParser::IO_MODE::READ);
    if (f == 0)
      return 0;
    _file = f;
    
    //Look for the meta group and from it pull the minimum and maximum event numbers
    auto meta_size = open_group(_file, "meta");
    auto metaID = std::get<0>(meta_size);
    if(metaID > 0)
    {
      char *datasetName = "meta";
      auto dataset_dims = open_dataset(metaID, datasetName);
      auto datasetId = std::get<0>(dataset_dims);
      auto len = std::get<1>(dataset_dims).at(0);

      int64_t  *data = new int64_t[len];      
      auto status = H5Dread(datasetId, H5T_NATIVE_ULONG, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
      std::cout << "Events: " << data[0] << " to " << data[2] << std::endl;

      _firstEvent = data[0];
      _lastEvent = data[1];
    }

    auto group_n_entries = open_group(f, "get");
    if (std::get<0>(group_n_entries)==-1)
      return 0;
    _group = std::get<0>(group_n_entries);

    return std::get<1>(group_n_entries);
}

 std::string ATHDFParser::get_event_name(std::size_t idx)
 {
    if (_group >= 0)
    {
       char name[100];
       H5Lget_name_by_idx(_group,".",H5_INDEX_NAME, H5_ITER_INC,idx,name,100, H5P_DEFAULT);
       std::string name_string(name);
       return name_string;
    }else{
      std::cerr << "> ATHDFParser::get_event_name:ERROR, invalid ID  " << idx << '\n';
      return "invalid";
    }
 }

std::vector<int64_t> ATHDFParser::get_header(std::string headerName)
{
  std::vector<int64_t> retVec;
  
  auto dataset_dims = open_dataset(_group, headerName.c_str());
  if( std::get<0>(dataset_dims) == 0)
    return retVec;

  _dataset = std::get<0>(dataset_dims);


  //Get the length of the header
  auto len = std::get<1>(dataset_dims).at(0);
    
  int64_t  *data = new int64_t[len];
  auto status = H5Dread(_dataset, H5T_NATIVE_ULONG, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);

  //Add read data to the vector and return it
  for ( int i = 0; i < len; ++i)
    retVec.push_back(data[i]);
  
  return retVec;
  
}

std::size_t ATHDFParser::n_pads(std::string i_raw_event)
{
    std::string dataset_name = i_raw_event;
    auto dataset_dims = open_dataset(_group, dataset_name.c_str());
    if (std::get<0>(dataset_dims)==0) return 0;
    _dataset = std::get<0>(dataset_dims);
    return std::get<1>(dataset_dims)[0];
}

std::vector<int16_t> ATHDFParser::pad_raw_data(std::size_t i_pad)
{
    int16_t data[517];
    hsize_t counts[2] = {1, 517};
    hsize_t offsets[2] = {i_pad, 0};
    hsize_t dims_out[2] = {1, 517};
    read_slab<int16_t>(_dataset, counts, offsets, dims_out, data);
    std::vector<int16_t> datav(data,data+517);
    return datav;
}

std::size_t ATHDFParser::inievent()
{
	return _inievent;
}

std::size_t ATHDFParser::datasets()
{
	herr_t idx = H5Literate(_group, H5_INDEX_NAME, H5_ITER_INC, NULL, file_info, NULL);
}

herr_t ATHDFParser::file_info(hid_t loc_id, const char *name, const H5L_info_t *linfo, void *opdata)
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


void ATHDFParser::end_raw_event()
{
    close_dataset(_dataset);
}

void ATHDFParser::close()
{
    close_group(_group);
    close_file(_file);
}
