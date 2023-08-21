#include "AtTabCanvas.h"

#include <TCanvas.h>
#include <TEveWindow.h>

ClassImp(AtTabCanvas);

void AtTabCanvas::MakeTab(TEveWindowSlot *slot)
{
   auto pack = slot->MakePack();
   pack->SetElementName(fTabName); // Sets name on tab
   pack->SetShowTitleBar(false);

   slot = pack->NewSlot();
   slot->StartEmbedding();

   fCanvas = new TCanvas(TString::Format("AtTabCanvas%d", fTabId));
   fCanvas->Divide(fCols, fRows);

   slot->StopEmbedding();
}

void AtTabCanvas::UpdateCanvas()
{
   fCanvas->Modified();
   fCanvas->Update();
}
