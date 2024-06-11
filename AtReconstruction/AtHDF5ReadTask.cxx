#include "AtHDF5ReadTask.h"

#include "AtEvent.h"
#include "AtHit.h"

#include <FairLogger.h>      // for LOG
#include <FairRootManager.h> // for FairRootManager

#include <TObject.h> // for TObject

#include <H5Cpp.h>

#include <utility> // for move

AtHDF5ReadTask::AtHDF5ReadTask(TString fileName, TString outputBranchName)
   : fInputFileName(std::move(fileName)), fOutputBranchName(std::move(outputBranchName)), fEventArray("AtEvent", 1)
{
}

InitStatus AtHDF5ReadTask::Init()
{

   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(error) << "Cannot find RootManager!";
      return kERROR;
   }

   ioMan->Register(fOutputBranchName, "AtTPC", &fEventArray, fIsPersistence);

   // TODO: Here is where we will need to open the HDF5 file

   return kSUCCESS;
}

void AtHDF5ReadTask::Exec(Option_t *opt)
{
   // You will want to do something like this I think:
   // http://davis.lbl.gov/Manuals/HDF5-1.6.5/cpplus_RM/h5group_8cpp-example.html starting from the comment "Now reopen
   // the group in the file"

   // Because events are indexted by their event number the opening line will be something like
   auto eventGroup = std::make_unique<H5::Group>(fFile->openGroup(TString::Format("Event_[%d]", fEventNum++)));

   // Then we create the event to fill
   auto *event = dynamic_cast<AtEvent *>(fEventArray.ConstructedAt(0, "C")); // Get and clear old event

   // Then we will look for the hits in the event group and add them to the event. This will involve
   // iterating through the dataset. The code in the loop will look something like.
   AtHit_t hitFromFile{};
   event->AddHit(-1, AtHit::XYZPoint(hitFromFile.x, hitFromFile.y, hitFromFile.z), hitFromFile.A);

   // After this loop filling the event, I don't think there is anything else to do
}

ClassImp(AtHDF5ReadTask);
