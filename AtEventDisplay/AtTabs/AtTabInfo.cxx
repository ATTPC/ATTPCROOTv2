#include "AtTabInfo.h"

#include <FairLogger.h> // for Logger, LOG

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

void AtTabInfo::AddAugment(std::shared_ptr<AtTabInfoBase> augment)
{
   if (augment == nullptr)
      return;

   AddAugment(augment, augment->GetDefaultName());
}

void AtTabInfo::AddAugment(std::shared_ptr<AtTabInfoBase> augment, std::string name)
{
   if (augment == nullptr)
      return;

   if (fInfoAugments.find(name) != fInfoAugments.end())
      LOG(error)
         << "AtTabInfo augment " << name
         << "already exists in this AtTabInfo. If replacement is intentional, use AtTabInfo::ReplaceAugment() instead.";

   ReplaceAugment(augment, name);
}

void AtTabInfo::ReplaceAugment(std::shared_ptr<AtTabInfoBase> augment)
{
   if (augment == nullptr)
      return;
   ReplaceAugment(augment, augment->GetDefaultName());
}
void AtTabInfo::ReplaceAugment(std::shared_ptr<AtTabInfoBase> augment, std::string name)
{
   if (augment == nullptr)
      return;
   fInfoAugments[name] = augment;
}

AtTabInfo::BasePtr AtTabInfo::GetAugment(std::string name)
{
   if (fInfoAugments.find(name) == fInfoAugments.end())
      return nullptr;
   else
      return fInfoAugments.at(name);
}
