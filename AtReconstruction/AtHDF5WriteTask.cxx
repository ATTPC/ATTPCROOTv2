#include "AtHDF5WriteTask.h"

#include "AtEvent.h"
#include "AtHit.h"

#include <FairLogger.h>      // for LOG
#include <FairRootManager.h> // for FairRootManager

#include <Math/Point3D.h> // for PositionVector3D
#include <TClonesArray.h>
#include <TObject.h> // for TObject

#include <H5Cpp.h>

#include <array>   // for array
#include <utility> // for move

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

   fFile = std::make_unique<H5::H5File>(fOutputFileName, H5F_ACC_TRUNC);

   return kSUCCESS;
}

void AtHDF5WriteTask::Exec(Option_t *opt)
{
   auto eventGroup = std::make_unique<H5::Group>(fFile->createGroup(TString::Format("/Event_[%d]", fEventNum)));

   auto *event = dynamic_cast<AtEvent *>(fEventArray->At(0));
   Int_t nHits = event->GetNumHits();
   const auto &traceEv = event->GetMesh();

   hsize_t hitdim[] = {(hsize_t)nHits};
   hsize_t tracedim[] = {(hsize_t)traceEv.size()};
   H5::DataSpace hitSpace(1, hitdim);
   H5::DataSpace traceSpace(1, tracedim);

   /**
      @TODO At some point we may have to think about how to generalize this so it can also write
      derived types of AtHit.
   **/
   AtHit_t hits[nHits];
   auto hdf5Type = AtHit().GetHDF5Type();

   for (Int_t iHit = 0; iHit < nHits; iHit++) {
      const AtHit &hit = event->GetHit(iHit);

      auto hitPos = hit.GetPosition();
      hits[iHit].x = hitPos.X();
      hits[iHit].y = hitPos.Y();
      hits[iHit].z = hitPos.Z();
      hits[iHit].t = hit.GetTimeStamp();
      hits[iHit].A = hit.GetCharge();
   }

   H5::DataSet hitset = fFile->createDataSet(TString::Format("/Event_[%d]/HitArray", fEventNum), hdf5Type, hitSpace);
   hitset.write(hits, hdf5Type);

   H5::DataSet traceset =
      fFile->createDataSet(TString::Format("/Event_[%d]/Trace", fEventNum), H5::PredType::NATIVE_FLOAT, traceSpace);
   traceset.write(traceEv.data(), H5::PredType::NATIVE_FLOAT);

   ++fEventNum;
}

ClassImp(AtHDF5WriteTask);
