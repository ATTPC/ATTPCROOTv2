#include "AtHDF5WriteTask.h"

#include "AtEvent.h"
#include "AtHit.h"

#include <Math/Point3D.h>
#include <TClonesArray.h>

#include <H5Cpp.h>

AtHDF5WriteTask::AtHDF5WriteTask(TString fileName, TString branchName)
   : fOutputFileName(std::move(fileName)), fInputBranchName(std::move(branchName))
{
}

InitStatus AtHDF5WriteTask::Init()
{

   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(ERROR) << "Cannot find RootManager!";
      return kERROR;
   }

   fEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fInputBranchName));

   // TODO: Here is where we will need to open the HDF5 file (fFile)

   return kSUCCESS;
}

void AtHDF5WriteTask::Exec(Option_t *opt)
{
   // You will want to do something like this I think:
   // http://davis.lbl.gov/Manuals/HDF5-1.6.5/cpplus_RM/h5group_8cpp-example.html starting from the
   // comment "Create a group in the file"

   // Because events are indexted by their event number the opening line will be something like
   auto eventGroup = std::make_unique<H5::Group>(fFile->createGroup(TString::Format("Event_[%d]", fEventNum++)));

   // Then we get the event we want to write
   auto *event = dynamic_cast<AtEvent *>(fEventArray->At(0));

   // Then we will look for the hits in the event and add them to the event group. This is basically
   // the code from R2HMain.cc starting at the line:
   // "hsize_t dim[] = {(hsize_t)nHits}; /* Dataspace dimensions */"

   // Then we need to write the dataset of the hits. Rather than creating the type specification here
   // You can call AtHit::GetHDF5Type() and that will return the composite type needed to write the
   // AtHit_t struct

   // Then we need to create the dataset with the trace and write it
}

ClassImp(AtHDF5WriteTask);
