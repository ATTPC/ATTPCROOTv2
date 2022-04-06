#ifndef __S800_HH
#define __S800_HH

#include <Rtypes.h>
#include <RtypesCore.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <algorithm>

#include "TObject.h"

class TBuffer;
class TClass;
class TMemberInspector;

using namespace std;

/**Raw data structure for S800 time of flight information.

 */
class GTimeOfFlight : public TObject {
public:
   GTimeOfFlight()
   {
      frf = 0;
      fobj = 0;
      fxfp = 0;
      fsi = 0;
      ftac_obj = 0;
      ftac_xfp = 0;
   }
   ~GTimeOfFlight() { Clear(); };
   void Clear()
   {
      frf = 0;
      fobj = 0;
      fxfp = 0;
      fsi = 0;
      ftac_obj = 0;
      ftac_xfp = 0;
   }
   void Set(Short_t rf, Short_t obj, Short_t xfp, Short_t si)
   {
      frf = rf;
      fobj = obj;
      fxfp = xfp;
      fsi = si;
   }
   void SetTAC(Short_t tac_obj, Short_t tac_xfp)
   {
      ftac_obj = tac_obj;
      ftac_xfp = tac_xfp;
   }
   Short_t GetRF() { return frf; }
   Short_t GetOBJ() { return fobj; }
   Short_t GetXFP() { return fxfp; }
   Short_t GetSi() { return fsi; }
   Short_t GetTACOBJ() { return ftac_obj; }
   Short_t GetTACXFP() { return ftac_xfp; }

protected:
   Short_t frf;
   Short_t fobj;
   Short_t fxfp;
   Short_t fsi;
   Short_t ftac_obj;
   Short_t ftac_xfp;

   ClassDef(GTimeOfFlight, 1);
};

/**Raw data structure for S800 trigger information

 */
class GTrigger : public TObject {
public:
   GTrigger()
   {
      fregistr = 0;
      fs800 = 0;
      fexternal1 = 0;
      fexternal2 = 0;
      fsecondary = 0;
   }
   ~GTrigger() { Clear(); };
   void Clear()
   {
      fregistr = 0;
      fs800 = 0;
      fexternal1 = 0;
      fexternal2 = 0;
      fsecondary = 0;
   }
   void Set(int registr, int s800, int external1, int external2, int secondary)
   {
      fregistr = registr;
      fs800 = s800;
      fexternal1 = external1;
      fexternal2 = external2;
      fsecondary = secondary;
   }
   Short_t GetRegistr() { return fregistr; }
   Short_t GetS800() { return fs800; }
   Short_t GetExternal1() { return fexternal1; }
   Short_t GetExternal2() { return fexternal2; }
   Short_t GetSecondary() { return fsecondary; }

protected:
   Short_t fregistr;
   Short_t fs800;
   Short_t fexternal1;
   Short_t fexternal2;
   Short_t fsecondary;

   ClassDef(GTrigger, 1);
};

/**Raw data structure for S800 scintillator information.

*/
class GScintillator : public TObject {
public:
   GScintillator()
   {
      fID = -1;
      fde_up = -1.0;
      fde_down = -1.0;
      ftime_up = -1.0;
      ftime_down = -1.0;
   }
   ~GScintillator() { Clear(); };
   void Clear()
   {
      fID = -1;
      fde_up = -1.0;
      fde_down = -1.0;
      ftime_up = -1.0;
      ftime_down = -1.0;
   }
   void SetID(int id) { fID = id; }
   void Set(int de_up, int time_up, int de_down, int time_down)
   {
      if (de_up > -1)
         fde_up = de_up;
      if (time_up > -1)
         ftime_up = time_up;
      if (de_down > -1)
         fde_down = de_down;
      if (time_down > -1)
         ftime_down = time_down;
   }
   Int_t GetID() { return fID; }
   Float_t GetDE_up() { return fde_up; }
   Float_t GetTime_up() { return ftime_up; }
   Float_t GetDE_down() { return fde_down; }
   Float_t GetTime_down() { return ftime_down; }

protected:
   Int_t fID;
   Float_t fde_up;
   Float_t fde_down;
   Float_t ftime_up;
   Float_t ftime_down;

   ClassDef(GScintillator, 1);
};

/**Raw data structure for S800 Hodoscope single crystal CsI(Na).
 */
class GHodoscope : public TObject {
public:
   GHodoscope() { fenergy = -1.0; }
   ~GHodoscope() { Clear(); }
   void Clear() { fenergy = -1.0; }
   void SetEnergy(Int_t energy) { fenergy = (Float_t)energy; }
   Float_t GetEnergy() { return fenergy; }

protected:
   Float_t fenergy;

   ClassDef(GHodoscope, 1);
};

/**Raw data structure for S800 Ion Chamber.

 */
class GIonChamber : public TObject {
public:
   GIonChamber()
   {
      fchannels.clear();
      fdata.clear();
   }
   ~GIonChamber() { Clear(); };
   void Clear()
   {
      fchannels.clear();
      fdata.clear();
   }
   void Set(int ch, int data)
   {
      fchannels.push_back(ch);
      fdata.push_back(data);
   }

   std::vector<int> GetChannels() { return fchannels; }
   std::vector<float> GetData() { return fdata; }

protected:
   std::vector<int> fchannels;
   std::vector<float> fdata;

   ClassDef(GIonChamber, 1);
};

/**Raw data structure for S800 CRDC.

*/
class GCrdc : public TObject {
public:
   GCrdc()
   {
      fID = -1;
      fanode = -1;
      ftac = -1;
      fdata.clear();
      fsample.clear();
      fchannels.clear();
   }
   ~GCrdc() { Clear(); };
   void Clear()
   {
      fID = -1;
      fanode = -1;
      ftac = -1;
      fdata.clear();
      fsample.clear();
      fchannels.clear();
   }
   void SetID(int id) { fID = id; }
   void SetAnodeTAC(int anode, int tac)
   {
      fanode = anode;
      ftac = tac;
   }
   void SetSampleWidth(int width) { fwidth = width; }
   void Set(Short_t data, Short_t sample, Short_t ch)
   {
      fdata.push_back(data);
      fsample.push_back(sample);
      fchannels.push_back(ch);
   }
   Float_t GetAnode() { return fanode; }
   Float_t GetTAC() { return ftac; }
   std::vector<Short_t> GetData() { return fdata; }
   std::vector<Short_t> GetSample() { return fsample; }
   std::vector<Short_t> GetChannels() { return fchannels; }

   Int_t GetID() { return fID; }

protected:
   Int_t fID;
   Float_t fanode;
   Float_t ftac;
   Short_t fwidth;
   std::vector<Short_t> fdata;
   std::vector<Short_t> fsample;
   std::vector<Short_t> fchannels;

   ClassDef(GCrdc, 1);
};

/**Raw Data Strcture for the New Multi hit TDC
 */
class GMultiHitTOF : public TObject {
public:
   GMultiHitTOF() { Clear(); }
   ~GMultiHitTOF() { Clear(); }

   void Clear()
   {

      fE1Up.clear();
      fE1Down.clear();
      fXf.clear();
      fObj.clear();
      fGalotte.clear();
      fRf.clear();
      fHodoscope.clear();
   }

   vector<Float_t> fE1Up;
   vector<Float_t> fE1Down;
   vector<Float_t> fXf;
   vector<Float_t> fObj;
   vector<Float_t> fGalotte;
   vector<Float_t> fRf;
   vector<Float_t> fHodoscope;

   ClassDef(GMultiHitTOF, 1);
};

///////////////////////// Added Simon
class GCrdc_test : public TObject {
public:
   GCrdc_test()
   {
      fdata.clear();
      memset(raw_crdc, 0, sizeof(raw_crdc));
   }
   ~GCrdc_test() { Clear(); };
   void Clear()
   {
      fdata.clear();
      memset(raw_crdc, 0, sizeof(raw_crdc));
   }

   void Set_raw(int n, int q, float val) { raw_crdc[n][q] = val; }
   std::vector<float> GetData() { return fdata; }
   float Get_raw(int n, int q) { return raw_crdc[n][q]; }

protected:
   float raw_crdc[2][224];
   std::vector<float> fdata;

   ClassDef(GCrdc_test, 1);
};
/////////////////////////

/**Raw data structure for S800 as a whole.  Contains instances of
   GTimeOfFlight GTrigger GScintillator GIonChamber  GHodoscope.  S800
   class will be a copy of the raw event information in the data stream.

   Contains methods to unpack the data for each of the S800's dectors
   for NSCL DAQ 10.02 format
*/
class S800 : public TObject {
public:
   S800() { Clear(); }
   void Clear()
   {
      fMultiHitTOF.Clear();
      fTof.Clear();
      fTrigger.Clear();
      for (int i = 0; i < 3; i++)
         fScintillator[i].Clear();
      fIonChamber.Clear();
      for (int i = 0; i < 2; i++)
         fCrdc[i].Clear();
      for (int i = 0; i < 32; i++) {
         fHodoscope[i].Clear();
      }
      fts = -1;
   }
   void SetTS(long long int ts) { fts = ts; }
   void SetInternalTS(long long int ts) { fits = ts; }
   void SetEvtNr(long long int nr) { fevtnr = nr; }
   GTimeOfFlight *GetTimeOfFlight() { return &fTof; }
   GTrigger *GetTrigger() { return &fTrigger; }
   GScintillator *GetScintillator(int id) { return &fScintillator[id]; }
   GIonChamber *GetIonChamber() { return &fIonChamber; }
   GHodoscope *GetHodoscope(int id) { return &fHodoscope[id]; }
   GCrdc *GetCrdc(int id) { return &fCrdc[id]; }
   long long int GetTS() { return fts; }
   long long int GetInternalTS() { return fits; }
   long long int GetEvtNr() { return fevtnr; }

   GCrdc_test *GetCrdc_test() { return &fCrdc_test; } // Added Simon

   GMultiHitTOF *GetMultiHitTOF() { return &fMultiHitTOF; }
   int DecodeS800(unsigned short *pevent, unsigned short twords);

protected:
   GTimeOfFlight fTof;
   GTrigger fTrigger;
   GScintillator fScintillator[3];
   GIonChamber fIonChamber;
   GHodoscope fHodoscope[32];
   GCrdc fCrdc[2];
   GMultiHitTOF fMultiHitTOF;

   GCrdc_test fCrdc_test; // Added Simon

   long long int fts;  // global
   long long int fits; // internal
   long long int fevtnr;

   unsigned short *DecodeS800Crdc(unsigned short *pevent, int id);
   unsigned short *DecodeS800CrdcRaw(unsigned short *pevent, int id);
   unsigned short *DecodeS800Scintillator(unsigned short *pevent, unsigned short updown, int id);
   unsigned short *DecodeS800IonChamber(unsigned short *pevent);
   unsigned short *DecodeS800TimeOfFlight(unsigned short *pevent);
   unsigned short *DecodeS800Trigger(unsigned short *pevent);
   unsigned short *DecodeS800HodoScope(unsigned short *pevent);

   unsigned short *DecodeS800NewMultiHitTDC(unsigned short *pevent);

   ClassDef(S800, 1);
};

#endif
