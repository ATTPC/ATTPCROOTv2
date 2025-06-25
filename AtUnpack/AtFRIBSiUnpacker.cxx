#include "AtFRIBSiUnpacker.h"

#include "AtGenericTrace.h" // for AtGenericTrace
#include "AtMap.h"
#include "AtPad.h"
#include "AtPadReference.h"
#include "AtRawEvent.h"

#include <FairLogger.h>

#include <H5Apublic.h>
#include <H5Gpublic.h>
#include <H5Ppublic.h>

ClassImp(AtFRIBSiUnpacker);

std::size_t AtFRIBSiUnpacker::n_pads(std::string i_raw_event)
{
   return n_entries(i_raw_event + "/get/pads")[0];
}
void AtFRIBSiUnpacker::processSiChannel(std::size_t chIndex)
{
   auto rawadc = pad_raw_data(chIndex);
   AtPadReference PadRef = {rawadc[0], rawadc[1], rawadc[2], rawadc[3]};

   TString pad_name = TString::Format("Si_%d_%d_%d_%d", PadRef.cobo, PadRef.asad, PadRef.aget, PadRef.ch);
   LOG(debug) << "Processing Si channel: " << pad_name;

   auto pad = fRawEvent->AddAuxPad(pad_name.Data()).first;
   setAdc(pad, rawadc);
};

void AtFRIBSiUnpacker::processData()
{
   TString event_name = TString::Format("event_%lld", fDataEventID);
   fRawEvent->SetEventName(event_name.Data());

   // Loop through and grab all of the pads in the event
   std::size_t npads = n_pads(event_name.Data());
   LOG(info) << "Unpacking " << npads << " pads in event " << fDataEventID;
   for (std::size_t i = 0; i < npads; i++) {
      processPad(i);
   }

   // Loop through and grab all of the generic traces in the event
   for (auto &sis : fFribPaths) {
      processSIS(event_name.Data(), sis);
   }

   // Add all the Si channels as aux pads. For now the naming scheme is "CoBo_AsAd_AGet_Ch"
   // Maybe this should change with the mapping?
   for (auto &siPath : fSiPaths) {
      std::string siDataset = "/get/" + siPath;
      auto nSiChannels = n_entries(event_name.Data() + siDataset)[0];
      LOG(info) << "Processing Si channels in dataset " << siPath << " with " << nSiChannels << " channels.";
      for (std::size_t chIndex = 0; chIndex < nSiChannels; ++chIndex) {
         processSiChannel(chIndex);
      }
   }

   end_raw_event(); // Close dataset
};
