#include "AtMCFitter.h"

#include "AtClusterizeLineTask.h"

namespace MCFitter {

AtMCFitter::AtMCFitter()
{
   // Create a basic cluster task
   fClusterize = std::make_shared<AtClusterizeLineTask>();
}

void AtMCFitter::AddParameter(const std::string &name, ParamPtr param)
{
   fParameters[name] = param;
}

AtMCFitter::ParamPtr AtMCFitter::GetParameter(const std::string &name) const
{
   if (fParameters.find(name) != fParameters.end()) {
      return fParameters.at(name);
   }
   return nullptr;
}

} // namespace MCFitter
