#include "AtEventProtoDrawTask.h"
#include "AtTpcProtoMap.h"
#include "TPaletteAxis.h"

ClassImp(AtEventProtoDrawTask)

   AtEventProtoDrawTask::AtEventProtoDrawTask()
{
}

AtEventProtoDrawTask::~AtEventProtoDrawTask() {}

void AtEventProtoDrawTask::DrawPadPlane()
{

   fAtMapPtr = new AtTpcProtoMap();
   if (fPadPlane) {
      fPadPlane->Reset(0);
      return;
   }

   dynamic_cast<AtTpcProtoMap *>(fAtMapPtr)->SetGeoFile("proto_geo_hires.root");
   fPadPlane = dynamic_cast<AtTpcProtoMap *>(fAtMapPtr)->GetAtTpcPlane("AtTPC_Proto");
   fCvsPadPlane->cd();
   fPadPlane->Draw("zcol");
}
