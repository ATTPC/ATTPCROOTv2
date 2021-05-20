#include "AtLinkDAQTask.h"

#include "TClonesArray.h"
#include "TChain.h"
#include "TTreeReader.h"
#include "TTreeReaderValue.h"

AtLinkDAQTask::AtLinkDAQTask()
{
   HTTimestamp *test = new HTTimestamp("tstamp");
   test->SetTimestamp(10);
   std::cout << "Set timestamp to: " << test->GetTimestamp() << std::endl;
}

AtLinkDAQTask::~AtLinkDAQTask() {}
