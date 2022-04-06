/*********************************************************************
 *   AtTPC Pad Class	AtPad                                            *
 *   Author: Y. Ayyad            				                     *
 *   Log: 05-03-2015 19:24 JST					                     *
 *   Adapted from SPiRITROOT STPad by G. Jhang                        *
 *								                                     *
 *********************************************************************/

#ifndef AtAUXPAD_H
#define AtAUXPAD_H

#include <Rtypes.h>
#include <RtypesCore.h>
#include <string>
#include <utility>

#include "AtPad.h"

class TBuffer;
class TClass;
class TMemberInspector;

class AtAuxPad : public AtPad {
protected:
   std::string fAuxName;

public:
   AtAuxPad(std::string fAuxName = "noname");
   AtAuxPad(const AtAuxPad &obj) = default;
   ~AtAuxPad() = default;

   void SetAuxName(std::string val) { fAuxName = std::move(val); }
   std::string GetAuxName() const { return fAuxName; }
   Bool_t IsAux() const { return true; }

   // Delete functions that have no meaning for aux pads
   // Note, the AtPad:: version of these classes will be called if using
   // a pointer or referece to AtPad because they are not virtual.
   Int_t GetPadNum() const = delete;
   Int_t GetSizeID() const = delete;
   XYPoint GetPadCoord() const = delete;

   ClassDef(AtAuxPad, 1);
};

#endif
