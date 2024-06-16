#include "AtSiArrayGeo.h"

#include <FairGeoSet.h>

#include <cstdio>

ClassImp(AtSiArrayGeo)

   // -----   Default constructor   -------------------------------------------
   AtSiArrayGeo::AtSiArrayGeo()
   : FairGeoSet()
{
   // Constructor
   // fName has to be the name used in the geometry for all volumes.
   // If there is a mismatch the geometry cannot be build.
   fName = "Si";
   maxSectors = 0;
   maxModules = 10;
}

// -------------------------------------------------------------------------

const char *AtSiArrayGeo::getModuleName(Int_t m)
{
   /** Returns the module name of AtSiArray number m
       Setting AtSiArray here means that all modules names in the
       ASCII file should start with AtSiArray otherwise they will
       not be constructed
   */
   sprintf(modName, "AtSiArray%i", m + 1);
   return modName;
}

const char *AtSiArrayGeo::getEleName(Int_t m)
{
   /** Returns the element name of Det number m */
   sprintf(eleName, "AtSiArray%i", m + 1);
   return eleName;
}
