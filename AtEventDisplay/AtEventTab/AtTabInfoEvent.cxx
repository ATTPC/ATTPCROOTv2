#include "AtTabInfoEvent.h"

#include "AtEvent.h"
#include "AtEventManagerNew.h"

#include <FairRootManager.h>

#include <TClonesArray.h>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

ClassImp(AtTabInfoEvent)

AtTabInfoEvent::AtTabInfoEvent() 
: fBranchName("AtEvent") 
{
}

void AtTabInfoEvent::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   fEventManager = AtEventManagerNew::Instance();

   fEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fBranchName));
   if (fEventArray)
      LOG(INFO) << cGREEN << "Event Array Found in branch " << fBranchName << "." << cNORMAL << std::endl;

}

void AtTabInfoEvent::Update()
{
   fEvent = dynamic_cast<AtEvent *>(fEventArray->At(0));
}

