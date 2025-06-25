#include "AtFRIBSiUnpacker.h"

#include "AtGenericTrace.h" // for AtGenericTrace
#include "AtMap.h"
#include "AtPad.h"
#include "AtPadReference.h"
#include "AtRawEvent.h"

#include <FairLogger.h>

#include <H5Apublic.h>
#include <H5Gpublic.h>
#include <H5Ppublic.h>

ClassImp(AtFRIBSiUnpacker);

std::size_t AtFRIBSiUnpacker::n_pads(std::string i_raw_event)
{
   return n_entries(i_raw_event + "/get/pads")[0];
};
