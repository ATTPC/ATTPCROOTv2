#ifndef __S800SETTINGS_HH
#define __S800SETTINGS_HH

#include <Rtypes.h>
#include <TObject.h>
#include <string>

class TBuffer;
class TClass;
class TMemberInspector;

class S800Settings : public TObject {
public:
   S800Settings();
   S800Settings(const char *);
   ~S800Settings();

   void ReadSettings();
   void SetFile(const char *filename) { fInputFile = filename; }

   const std::string InputFile() { return fInputFile; }

   int XFit() { return fXFit; }
   int XFitFunc() { return fXFitFunc; }
   Float_t XOffset(int ch) { return fxOffset[ch]; }
   Float_t XSlope(int ch) { return fxSlope[ch]; }
   Float_t YOffset(int ch) { return fyOffset[ch]; }
   Float_t YSlope(int ch) { return fySlope[ch]; }
   const char *CalFile() { return fCalFile.c_str(); }
   const char *PedestalFile() { return fPedestalFile.c_str(); }
   const char *BadFile() { return fBadFile.c_str(); }
   const char *CalFileIC() { return fCalFileIC.c_str(); }

protected:
   std::string fInputFile;
   std::string fCalFile;
   std::string fPedestalFile;
   std::string fBadFile;
   std::string fCalFileIC;

   Int_t fXFit{};
   Int_t fXFitFunc{};
   Float_t fxOffset[2]{};
   Float_t fxSlope[2]{};
   Float_t fyOffset[2]{};
   Float_t fySlope[2]{};

   ClassDef(S800Settings, 1)
};

#endif
