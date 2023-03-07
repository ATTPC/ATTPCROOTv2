#ifndef ATMACROTASK_H
#define ATMACROTASK_H

#include <FairTask.h>

#include <functional> // for function
#include <vector>

class TClass;

class AtMacroTask : public FairTask {
private:
   std::vector<std::function<void()>> fFunctions;

public:
   AtMacroTask() = default;
   ~AtMacroTask() = default;

   void Exec(Option_t *option);

   void AddFunction(std::function<void()> function) { fFunctions.push_back(function); }


   ClassDef(AtMacroTask, 1);
};

#endif
