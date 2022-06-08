#ifndef AtGEARRAYGEOPAR_H
#define AtGEARRAYGEOPAR_H

#include <FairParGenericSet.h>

#include <Rtypes.h>

class TObjArray;
class FairParamList;
class TBuffer;
class TClass;
class TMemberInspector;

class AtGeArrayGeoPar : public FairParGenericSet {
public:
   /** List of FairGeoNodes for sensitive  volumes */
   TObjArray *fGeoSensNodes;

   /** List of FairGeoNodes for sensitive  volumes */
   TObjArray *fGeoPassNodes;

   AtGeArrayGeoPar(const char *name = "AtGeArrayGeoPar", const char *title = "AtGeArray Geometry Parameters",
                   const char *context = "TestDefaultContext");
   ~AtGeArrayGeoPar(void);
   void clear(void);
   void putParams(FairParamList *);
   Bool_t getParams(FairParamList *);
   TObjArray *GetGeoSensitiveNodes() { return fGeoSensNodes; }
   TObjArray *GetGeoPassiveNodes() { return fGeoPassNodes; }

private:
   AtGeArrayGeoPar(const AtGeArrayGeoPar &);
   AtGeArrayGeoPar &operator=(const AtGeArrayGeoPar &);

   ClassDef(AtGeArrayGeoPar, 1)
};

#endif
