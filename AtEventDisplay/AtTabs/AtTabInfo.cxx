#include "AtTabInfo.h"

#include <FairLogger.h> // for Logger, LOG

#include <iostream> // for operator<<, basic_ostream, basic_ostream<>::...
#include <utility>  // for move, pair

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

void AtTabInfo::Init()
{
   for (auto const &x : fInfoAugments) {
      x.second->Init();
   }
}

void AtTabInfo::Update(DataHandling::Subject *changedSubject)
{
   for (auto const &[name, info] : fInfoAugments) {
      LOG(debug2) << "Updating source for" << name;
      info->Update(changedSubject);
   }
}

AtTabInfoBase *AtTabInfo::AddAugment(std::unique_ptr<AtTabInfoBase> augment)
{
   return AddAugment(std::move(augment), augment->GetDefaultName());
}
AtTabInfoBase *AtTabInfo::AddAugment(std::unique_ptr<AtTabInfoBase> augment, std::string name)
{
   if (fInfoAugments.find(name) != fInfoAugments.end())
      LOG(error)
         << "AtTabInfo augment " << name
         << "already exists in this AtTabInfo. If replacement is intentional, use AtTabInfo::ReplaceAugment() instead.";

   return ReplaceAugment(std::move(augment), name);
}

AtTabInfoBase *AtTabInfo::ReplaceAugment(std::unique_ptr<AtTabInfoBase> augment)
{

   return ReplaceAugment(std::move(augment), augment->GetDefaultName());
}
AtTabInfoBase *AtTabInfo::ReplaceAugment(std::unique_ptr<AtTabInfoBase> augment, std::string name)
{
   fInfoAugments[name] = std::move(augment);
   return fInfoAugments[name].get();
}

AtTabInfoBase *AtTabInfo::GetAugment(std::string name)
{
   if (fInfoAugments.find(name) == fInfoAugments.end())
      return nullptr;
   else
      return fInfoAugments.at(name).get();
}
