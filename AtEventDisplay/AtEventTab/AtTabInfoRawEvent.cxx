#include "AtTabInfoRawEvent.h"

#include "AtEventManagerNew.h"
#include "AtRawEvent.h"

#include <FairRootManager.h>

#include <TClonesArray.h>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

ClassImp(AtTabInfoRawEvent)

AtTabInfoRawEvent::AtTabInfoRawEvent() 
: fBranchName("AtRawEvent")
{
}

void AtTabInfoRawEvent::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   fEventManager = AtEventManagerNew::Instance();

   fRawEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fBranchName));
   if (fRawEventArray)
      LOG(INFO) << cGREEN << "Raw Event Array Found in branch " << fBranchName << "." << cNORMAL << std::endl;

}

void AtTabInfoRawEvent::Update()
{
   fRawEvent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));
}

