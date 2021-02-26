/********************************************************************************
 *    Copyright (C) 2014 GSI Helmholtzzentrum fuer Schwerionenforschung GmbH    *
 *                                                                              *
 *              This software is distributed under the terms of the             *
 *         GNU Lesser General Public Licence version 3 (LGPL) version 3,        *
 *                  copied verbatim in the file "LICENSE"                       *
 ********************************************************************************/
#include "AtTpcGeo.h"
#include "FairGeoNode.h"

ClassImp(AtTpcGeo)

   // -----   Default constructor   -------------------------------------------
   AtTpcGeo::AtTpcGeo()
   : FairGeoSet()
{
   // Constructor
   // fName has to be the name used in the geometry for all volumes.
   // If there is a mismatch the geometry cannot be build.
   fName = "newdetector";
   maxSectors = 0;
   maxModules = 10;
}

// -------------------------------------------------------------------------

const char *AtTpcGeo::getModuleName(Int_t m)
{
   /** Returns the module name of AtTpc number m
       Setting AtTpc here means that all modules names in the
       ASCII file should start with AtTpc otherwise they will
       not be constructed
   */
   sprintf(modName, "AtTpc%i", m + 1);
   return modName;
}

const char *AtTpcGeo::getEleName(Int_t m)
{
   /** Returns the element name of Det number m */
   sprintf(eleName, "AtTpc%i", m + 1);
   return eleName;
}
