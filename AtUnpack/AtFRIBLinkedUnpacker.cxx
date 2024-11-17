#include "AtFRIBLinkedUnpacker.h"

#include "AtGenericTrace.h" // for AtGenericTrace
#include "AtMap.h"
#include "AtPad.h"
#include "AtPadReference.h"
#include "AtRawEvent.h"

#include <FairLogger.h>

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
   LOG(info) << "Events: " << fFirstEvent << " to " << fLastEvent;
};
