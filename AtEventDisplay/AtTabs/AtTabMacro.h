#ifndef ATTABMACRO_H
#define ATTABMACRO_H

#include "AtTabBase.h"

#include <Rtypes.h>  // for Int_t, Bool_t, THashConsistencyHolder, Color_t
#include <TString.h> // for TString

#include <functional> // for function
#include <memory>     // for shared_ptr, unique_ptr
#include <string>     // for string
#include <unordered_map>

class TBuffer;
class TClass;
class TMemberInspector;

class AtTabInfo;
class AtTabInfoBase;
class AtEventManagerNew;
class AtMap;

class TCanvas;
class TEveRGBAPalette;
class TChain;
class TTree;

class AtTabMacro : public AtTabBase {
protected:
   std::shared_ptr<AtMap> fDetmap;

   TString fMap;

   TCanvas *fCvsMacro;

   Int_t fRows;
   Int_t fCols;

   TChain *fTree;

   std::unordered_map<Int_t, std::function<void(TTree(*))>> fDrawTreeMap;
   std::unordered_map<Int_t, std::function<void(AtTabInfo(*))>> fDrawEventMap;
   std::unordered_map<Int_t, std::function<void(AtTabInfo(*), Int_t)>> fDrawPadMap;

   TString fTabName;

   TEveRGBAPalette *fRGBAPalette;

public:
   AtTabMacro();
   void InitTab() override;
   void Exec() override;

   void MakeTab() override;

   void SetInputTree(TString fileName, TString treeName);
   void SetDrawTreeFunction(Int_t pos, std::function<void(TTree(*))> function);
   void SetDrawEventFunction(Int_t pos, std::function<void(AtTabInfo(*))> function);
   void SetDrawPadFunction(Int_t pos, std::function<void(AtTabInfo(*), Int_t)> function);

   void SetMap(std::shared_ptr<AtMap> map) { fDetmap = map; }
   void SetColumns(Int_t cols) { fCols = cols; }
   void SetRows(Int_t rows) { fRows = rows; }
   void SetTabName(TString tabName) { fTabName = tabName; }
   void DrawTree();

   void DrawPad(Int_t padNum) override;

private:
   void UpdateCvsMacro();

   // Functions for drawing hits

   ClassDefOverride(AtTabMacro, 1)
};

#endif
