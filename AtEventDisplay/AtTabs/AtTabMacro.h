#ifndef ATTABMACRO_H
#define ATTABMACRO_H

#include "AtDataObserver.h"
#include "AtTabCanvas.h"
#include "AtViewerManagerSubject.h" // for AtPadNum

#include <Rtypes.h>  // for Int_t, Bool_t, THashConsistencyHolder, Color_t
#include <TString.h> // for TString

#include <functional> // for function
#include <memory>     // for shared_ptr, unique_ptr
#include <unordered_map>

class AtTabInfo;
class AtMap;
class TEveWindowSlot;
class TBuffer;
class TClass;
class TMemberInspector;
class TCanvas;
class TChain;
class TTree;
namespace DataHandling {
class AtSubject;
}

/// Tab for drawing arbitrary functions (probably needs refactor)
class AtTabMacro : public AtTabCanvas, public DataHandling::AtObserver {
protected:
   TChain *fTree;

   DataHandling::AtPadNum *fPadNum;

   std::unordered_map<Int_t, std::function<void(TTree(*))>> fDrawTreeMap;
   std::unordered_map<Int_t, std::function<void(AtTabInfo(*))>> fDrawEventMap;
   std::unordered_map<Int_t, std::function<void(AtTabInfo(*), Int_t)>> fDrawPadMap;

public:
   AtTabMacro(int nRow = 1, int nCol = 1, TString name = "Macro");
   ~AtTabMacro();

   void InitTab() override;
   void Exec() override;
   void Update(DataHandling::AtSubject *sub) override;

   void SetInputTree(TString fileName, TString treeName);
   void SetDrawTreeFunction(Int_t pos, std::function<void(TTree(*))> function);
   void SetDrawEventFunction(Int_t pos, std::function<void(AtTabInfo(*))> function);
   void SetDrawPadFunction(Int_t pos, std::function<void(AtTabInfo(*), Int_t)> function);

protected:
   void MakeTab(TEveWindowSlot *) override;
   void DrawTree();

private:
   void UpdateCvsMacro();

   // Functions for drawing hits

   ClassDefOverride(AtTabMacro, 1)
};

#endif
