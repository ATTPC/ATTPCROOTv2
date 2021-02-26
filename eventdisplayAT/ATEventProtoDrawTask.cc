#include "ATEventProtoDrawTask.hh"
#include "AtTpcProtoMap.h"
#include "TPaletteAxis.h"

ClassImp(ATEventProtoDrawTask)

ATEventProtoDrawTask::ATEventProtoDrawTask()
{

  
}

ATEventProtoDrawTask::~ATEventProtoDrawTask()
{

  
}

void
ATEventProtoDrawTask::DrawPadPlane()
{

  fAtMapPtr = new AtTpcProtoMap();
  if(fPadPlane) 
  {
    fPadPlane->Reset(0);
    return;
  }

    dynamic_cast<AtTpcProtoMap*>(fAtMapPtr)->SetGeoFile("proto_geo_hires.root");
    fPadPlane = dynamic_cast<AtTpcProtoMap*>(fAtMapPtr)->GetATTPCPlane("ATTPC_Proto");
    fCvsPadPlane -> cd();
    fPadPlane -> Draw("zcol");
    
   

  
}
