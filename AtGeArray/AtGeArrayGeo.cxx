#include "AtGeArrayGeo.h"

#include <FairGeoSet.h>

#include <cstdio>

ClassImp(AtGeArrayGeo)

   // -----   Default constructor   -------------------------------------------
   AtGeArrayGeo::AtGeArrayGeo()
   : FairGeoSet()
{
   // Constructor
   // fName has to be the name used in the geometry for all volumes.
   // If there is a mismatch the geometry cannot be build.
   fName = "Ge";
   maxSectors = 0;
   maxModules = 10;
}

// -------------------------------------------------------------------------

const char *AtGeArrayGeo::getModuleName(Int_t m)
{
   /** Returns the module name of AtGeArray number m
       Setting AtGeArray here means that all modules names in the
       ASCII file should start with AtGeArray otherwise they will
       not be constructed
   */
   sprintf(modName, "AtGeArray%i", m + 1);
   return modName;
}

const char *AtGeArrayGeo::getEleName(Int_t m)
{
   /** Returns the element name of Det number m */
   sprintf(eleName, "AtGeArray%i", m + 1);
   return eleName;
}
