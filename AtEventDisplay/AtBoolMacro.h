#ifndef ATBOOLMACRO_H
#define ATBOOLMACRO_H

#include "AtTabInfo.h"

#include <Rtypes.h>

#include <functional> // for function
#include <memory>     // for unique_ptr

/// An object for running bool functions using AtTabInfo
class AtBoolMacro {
protected:
   std::unique_ptr<AtTabInfo> fTabInfo{std::make_unique<AtTabInfo>()};
   std::function<bool(AtTabInfo(*))> fFunction;

public:
   AtBoolMacro(std::function<bool(AtTabInfo(*))> function = nullptr) : fFunction(function) {}
   ~AtBoolMacro() = default;

   bool ExecFunction() { return fFunction(fTabInfo.get()); }

   AtTabInfo *GetTabInfo() { return fTabInfo.get(); }
   void SetFunction(std::function<bool(AtTabInfo(*))> function) { fFunction = function; }

protected:
   ClassDef(AtBoolMacro, 1)
};

#endif
