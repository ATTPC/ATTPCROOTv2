#ifndef SIARRAYGEO_H
#define SIARRAYGEO_H

#include <FairGeoSet.h>

#include <Rtypes.h>
#include <TString.h>

class TBuffer;
class TClass;
class TMemberInspector;

class AtSiArrayGeo : public FairGeoSet {

protected:
   char modName[20]{}; // name of module
   char eleName[20]{}; // substring for elements in module
public:
   AtSiArrayGeo();
   ~AtSiArrayGeo() {}
   const char *getModuleName(Int_t);
   const char *getEleName(Int_t);
   inline Int_t getModNumInMod(const TString &);
   ClassDef(AtSiArrayGeo, 1)
};

inline Int_t AtSiArrayGeo::getModNumInMod(const TString &name)
{
   /** returns the module index from module name
    ?? in name[??] has to be the length of the detector name in the
    .geo file. For example if all nodes in this file starts with
    newdetector ?? has to be 11.
   */
   return (Int_t)(name[11] - '0') - 1; //
}

#endif
