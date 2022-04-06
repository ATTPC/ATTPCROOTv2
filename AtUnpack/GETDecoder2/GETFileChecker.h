// =================================================
//  GETFileChecker Class
//
//  Description:
//    Check if the file exists or not
//
//  Genie Jhang ( geniejhang@majimak.com )
//  2015. 09. 01
// =================================================

#ifndef GETFILECHECKER
#define GETFILECHECKER

#include <Rtypes.h>

#include "TString.h"

class TBuffer;
class TClass;
class TMemberInspector;

class GETFileChecker {
public:
   static TString CheckFile(TString filename);

   ClassDef(GETFileChecker, 1)
};

#endif
