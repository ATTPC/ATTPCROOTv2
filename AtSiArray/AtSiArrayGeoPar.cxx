#include "AtSiArrayGeoPar.h"

#include <FairParGenericSet.h>

#include <FairParamList.h>
#include "TObjArray.h"

ClassImp(AtSiArrayGeoPar)

   AtSiArrayGeoPar ::AtSiArrayGeoPar(const char *name, const char *title, const char *context)
   : FairParGenericSet(name, title, context), fGeoSensNodes(new TObjArray()), fGeoPassNodes(new TObjArray())
{
}

AtSiArrayGeoPar::~AtSiArrayGeoPar(void) {}

void AtSiArrayGeoPar::clear(void)
{
   if (fGeoSensNodes) {
      delete fGeoSensNodes;
   }
   if (fGeoPassNodes) {
      delete fGeoPassNodes;
   }
}

void AtSiArrayGeoPar::putParams(FairParamList *l)
{
   if (!l) {
      return;
   }
   l->addObject("FairGeoNodes Sensitive List", fGeoSensNodes);
   l->addObject("FairGeoNodes Passive List", fGeoPassNodes);
}

Bool_t AtSiArrayGeoPar::getParams(FairParamList *l)
{
   if (!l) {
      return kFALSE;
   }
   if (!l->fillObject("FairGeoNodes Sensitive List", fGeoSensNodes)) {
      return kFALSE;
   }
   if (!l->fillObject("FairGeoNodes Passive List", fGeoPassNodes)) {
      return kFALSE;
   }
   return kTRUE;
}
