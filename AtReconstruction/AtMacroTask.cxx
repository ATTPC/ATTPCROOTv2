#include "AtMacroTask.h"

ClassImp(AtMacroTask);

void AtMacroTask::Exec(Option_t *option)
{
   for (auto func : fFunctions)
      func();
}
