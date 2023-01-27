#ifndef ATTABMACRO_H
#define ATTABMACRO_H

#include "AtDataObserver.h"
#include "AtTabCanvas.h"
#include "AtViewerManagerSubject.h" // for AtPadNum

#include <Rtypes.h>  // for Int_t, Bool_t, THashConsistencyHolder, Color_t
#include <TString.h> // for TString

#include <functional> // for function
#include <unordered_map>

class AtTabInfo;
class TEveWindowSlot;
class TBuffer;
class TClass;
class TMemberInspector;
namespace DataHandling {
class AtSubject;
}

/// Tab for drawing arbitrary functions (probably needs refactor)
class AtTabMacro : public AtTabCanvas, public DataHandling::AtObserver {
protected:
   DataHandling::AtPadNum *fPadNum;

   std::unordered_map<Int_t, std::function<void(AtTabInfo(*))>> fDrawTreeMap;
   std::unordered_map<Int_t, std::function<void(AtTabInfo(*))>> fDrawEventMap;
   std::unordered_map<Int_t, std::function<void(AtTabInfo(*), Int_t)>> fDrawPadMap;

public:
   AtTabMacro(int nRow = 1, int nCol = 1, TString name = "Macro");
   ~AtTabMacro();

   void InitTab() override;
   void Exec() override;
   void Update(DataHandling::AtSubject *sub) override;

   void SetDrawTreeFunction(std::function<void(AtTabInfo(*))> function, int row = 0, int col = 0);
   void SetDrawEventFunction(std::function<void(AtTabInfo(*))> function, int row = 0, int col = 0);
   void SetDrawPadFunction(std::function<void(AtTabInfo(*), Int_t)> function, int row = 0, int col = 0);

protected:
   void MakeTab(TEveWindowSlot *) override;
   void DrawTree();

   ClassDefOverride(AtTabMacro, 1)
};

#endif
