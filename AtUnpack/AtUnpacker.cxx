#include "AtUnpacker.h"

#include <Rtypes.h>

#include <utility>

ClassImp(AtUnpacker);

AtUnpacker::AtUnpacker(mapPtr map) : fMap(std::move(map)) {}
