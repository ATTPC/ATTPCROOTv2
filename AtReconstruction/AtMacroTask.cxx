#include "AtMacroTask.h"

ClassImp(AtMacroTask);

InitStatus AtMacroTask::Init()
{
   for (auto func : fInitFunctions)
      func();

   return kSUCCESS;
}

void AtMacroTask::Exec(Option_t *option)
{
   for (auto func : fFunctions)
      func();
}
