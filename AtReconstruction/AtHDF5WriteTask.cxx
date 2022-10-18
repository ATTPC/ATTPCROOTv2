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
   auto traceEv = event->GetMesh();

   hsize_t hitdim[] = {(hsize_t)nHits};
   hsize_t tracedim[] = {(hsize_t)traceEv.size()};
   H5::DataSpace hitSpace(1, hitdim);
   H5::DataSpace traceSpace(1, tracedim);

   AtHit_t hits[nHits];
   H5::CompType hdf5Type(sizeof(AtHit_t));

   for (Int_t iHit = 0; iHit < nHits; iHit++) {
      AtHit hit = event->GetHit(iHit);
      // hdf5Type = hit.GetHDF5Type();

      auto hitPos = hit.GetPosition();
      hits[iHit].x = hitPos.X();
      hits[iHit].y = hitPos.Y();
      hits[iHit].z = hitPos.Z();
      hits[iHit].t = hit.GetTimeStamp();
      hits[iHit].A = hit.GetCharge();
   }

   hdf5Type.insertMember("x", HOFFSET(AtHit_t, x), H5::PredType::NATIVE_DOUBLE);
   hdf5Type.insertMember("y", HOFFSET(AtHit_t, y), H5::PredType::NATIVE_DOUBLE);
   hdf5Type.insertMember("z", HOFFSET(AtHit_t, z), H5::PredType::NATIVE_DOUBLE);
   hdf5Type.insertMember("t", HOFFSET(AtHit_t, t), H5::PredType::NATIVE_INT);
   hdf5Type.insertMember("A", HOFFSET(AtHit_t, A), H5::PredType::NATIVE_DOUBLE);
   hdf5Type.insertMember("trackID", HOFFSET(AtHit_t, trackID), H5::PredType::NATIVE_INT);
   hdf5Type.insertMember("pointIDMC", HOFFSET(AtHit_t, pointIDMC), H5::PredType::NATIVE_INT);
   hdf5Type.insertMember("energyMC", HOFFSET(AtHit_t, energyMC), H5::PredType::NATIVE_DOUBLE);
   hdf5Type.insertMember("elossMC", HOFFSET(AtHit_t, elossMC), H5::PredType::NATIVE_DOUBLE);
   hdf5Type.insertMember("angleMC", HOFFSET(AtHit_t, angleMC), H5::PredType::NATIVE_DOUBLE);
   hdf5Type.insertMember("AMC", HOFFSET(AtHit_t, AMC), H5::PredType::NATIVE_INT);
   hdf5Type.insertMember("ZMC", HOFFSET(AtHit_t, ZMC), H5::PredType::NATIVE_INT);

   H5::DataSet *hitset;
   hitset = new H5::DataSet(
      fFile->createDataSet(TString::Format("/Event_[%d]/Hit_[%d]", fEventNum, fEventNum), hdf5Type, hitSpace));
   hitset->write(hits, hdf5Type);
   delete hitset;

   Double_t *trace = new Double_t[traceEv.size()];

   for (auto itrace = 0; itrace < traceEv.size(); ++itrace) {
      trace[itrace] = traceEv[itrace];
   }

   H5::DataSet *traceset;
   traceset = new H5::DataSet(fFile->createDataSet(TString::Format("/Event_[%d]/Trace_[%d]", fEventNum, fEventNum),
                                                   H5::PredType::NATIVE_DOUBLE, traceSpace));
   traceset->write(trace, H5::PredType::NATIVE_DOUBLE);
   delete traceset;

   ++fEventNum;
}

ClassImp(AtHDF5WriteTask);
