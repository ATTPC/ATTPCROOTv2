#include "AtTabInfo.h"

#include <FairLogger.h> // for Logger, LOG

#include <iostream> // for operator<<, basic_ostream, basic_ostream<>::...
#include <utility>  // for move, pair

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

ClassImp(AtTabInfo);

void AtTabInfo::Init()
{
   for (auto const &x : fInfoAugments) {
      x.second->Init();
   }
}

void AtTabInfo::Update()
{
   for (auto const &x : fInfoAugments) {
      x.second->Update();
   }
}

AtTabInfoBase *AtTabInfo::AddAugment(std::string name, std::unique_ptr<AtTabInfoBase> augment)
{
   if (fInfoAugments.find(name) != fInfoAugments.end())
      LOG(error)
         << "AtTabInfo augment " << name
         << "already exists in this AtTabInfo. If replacement is intentional, use AtTabInfo::ReplaceAugment() instead.";

   return ReplaceAugment(name, std::move(augment));
}

AtTabInfoBase *AtTabInfo::ReplaceAugment(std::string name, std::unique_ptr<AtTabInfoBase> augment)
{
   fInfoAugments[name] = std::move(augment);
   return fInfoAugments[name].get();
}

AtTabInfoBase *AtTabInfo::GetAugment(std::string name)
{
   if(fInfoAugments.find(name) == fInfoAugments.end())
      return nullptr;
   else
      return fInfoAugments.at(name).get();
}
