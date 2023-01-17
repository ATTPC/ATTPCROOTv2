#ifndef ATTABINFO_H
#define ATTABINFO_H

#include "AtTabInfoBase.h"

#include <map>
#include <memory> // for unique_ptr
#include <string>

class AtTabInfo : public AtTabInfoBase {
protected:
   std::map<std::string, std::unique_ptr<AtTabInfoBase>> fInfoAugments;

public:
   AtTabInfo() = default;
   void Init() override;
   void Update() override;

   AtTabInfoBase *AddAugment(std::string name, std::unique_ptr<AtTabInfoBase> augment);
   AtTabInfoBase *ReplaceAugment(std::string name, std::unique_ptr<AtTabInfoBase> augment);
   AtTabInfoBase *GetAugment(std::string name);
};

#endif
