/*

  Header For ROOT Settings Object.
  Will Store information for various builds.



 */
#ifndef __SETTINGS_HH
#define __SETTINGS_HH

#include <Rtypes.h>
#include <TNamed.h>

#include <iostream>
#include <map>
#include <string>

class TBuffer;
class TClass;
class TMemberInspector;

using namespace std;

class S800TSettings : public TNamed {

public:
   S800TSettings();
   ~S800TSettings();

   void PrintAll();
   void PrintChannelCorrections(string Name);
   void PrintChannelMapInfo(int GlobalID);
   void PrintChannelMapInfo(string Name);
   void PrintFilterInfo(string Name);

   double GetChannelsSlope(string Name) { return TheEnergySlopes[Name]; }
   double GetChannelsIntercept(string Name) { return TheEnergyIntercepts[Name]; }
   double GetChannelsTimeOffset(string Name) { return TheTimingOffsets[Name]; }

   int GetBarId(string Name) { return BarIds[Name]; }
   string GetBarName(int BarId) { return BarId2Name[BarId]; }

   void AddCorrectionSettings(string Name, double slope, double inter, double toff);
   void AddMapSettings(string Name, int GlobalID, string RefName, int refGlobalID);
   void AddFilterSettings(string Name, int FL, int FG, int d, int w, bool flag);

   void SetBarIds(map<string, int> v);

   inline Int_t GetNumBars() { return BarIds.size(); }

private:
   map<string, double> TheTimingOffsets;
   map<string, double> TheEnergySlopes;
   map<string, double> TheEnergyIntercepts;

   map<string, int> TheFLs;
   map<string, int> TheFGs;
   map<string, int> Theds;
   map<string, int> Thews;
   map<string, bool> TheDontTraceAnalyzeFlags;

   map<int, string> GlobalID2FullName;
   map<int, int> GlobalID2RefGlobalID;
   map<int, string> GlobalID2RefName;

   map<string, int> Name2GlobalID;

   map<string, int> BarIds;
   void BuildReverseMap();
   map<int, string> BarId2Name;

public:
   ClassDef(S800TSettings, 2);
};

#endif
