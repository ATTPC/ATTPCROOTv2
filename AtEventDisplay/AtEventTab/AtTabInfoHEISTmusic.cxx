#include "AtTabInfoHEISTmusic.h"

#include "AtRawEvent.h"

#include <FairRootManager.h>

#include <TTreeReader.h>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

ClassImp(AtTabInfoHEISTmusic);

AtTabInfoHEISTmusic::AtTabInfoHEISTmusic()
{
   fInfoRawEvent = std::make_unique<AtTabInfoFairRoot<AtRawEvent>>();
}

void AtTabInfoHEISTmusic::Init()
{
   fInfoRawEvent->Init();
   fReader = new TTreeReader(fTree);
   fMusicReader = new TTreeReaderValue<HTMusicIC>(*fReader, "MUSIC");
}

void AtTabInfoHEISTmusic::Update()
{
   fInfoRawEvent->Update();
   fReader->SetEntry(fInfoRawEvent->GetInfo()->GetEventID());
   fMusicIC = fMusicReader->Get();
}
