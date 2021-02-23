#ifndef ATSIARRAYGEOPAR_H
#define ATSIARRAYGEOPAR_H

#include "FairParGenericSet.h"

class TObjArray;
class FairParamList;

class AtSiArrayGeoPar : public FairParGenericSet {
public:
   /** List of FairGeoNodes for sensitive  volumes */
   TObjArray *fGeoSensNodes;

   /** List of FairGeoNodes for sensitive  volumes */
   TObjArray *fGeoPassNodes;

   AtSiArrayGeoPar(const char *name = "AtSiArrayGeoPar", const char *title = "AtSiArray Geometry Parameters",
                   const char *context = "TestDefaultContext");
   ~AtSiArrayGeoPar(void);
   void clear(void);
   void putParams(FairParamList *);
   Bool_t getParams(FairParamList *);
   TObjArray *GetGeoSensitiveNodes() { return fGeoSensNodes; }
   TObjArray *GetGeoPassiveNodes() { return fGeoPassNodes; }

private:
   AtSiArrayGeoPar(const AtSiArrayGeoPar &);
   AtSiArrayGeoPar &operator=(const AtSiArrayGeoPar &);

   ClassDef(AtSiArrayGeoPar, 1)
};

#endif
