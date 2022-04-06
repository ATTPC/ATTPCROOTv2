#ifndef __S800CALIBRAtION_HH
#define __S800CALIBRAtION_HH

#include <RtypesCore.h>
#include <ext/alloc_traits.h>
#include <algorithm>
#include <memory>
#include <vector>

#include "S800Calc.h"

class GTimeOfFlight;
class S800;
class S800Settings;

/**Class responsible for calibrating the S800.  Takes a S800 raw data object as input
and creates a S800Calc object.  This requires a variety of input files that are specfied
in a general input file along with other general settings about the calibration process.
That file is specifed in S800Settings.
*/
class S800Calibration {
public:
   S800Calibration();
   S800Calibration(S800Settings *);
   ~S800Calibration();

   void ReadCrdcCalibration(const char *filename, const char *pedestalfile);
   void ReadCrdcBadPads(const char *filename);
   // bool IsCrdcBadPad(int ch);
   bool IsBad(int ch, int CrdcId)
   {
      for (UShort_t b = 0; b < fbad[CrdcId].size(); b++) {
         if (ch == fbad[CrdcId][b])
            return true;
      }
      return false;
   };

   /**Main method for calibrating s800. Calls all other methods needed for
      calibrating each S800 detector element.
    */
   void S800Calculate(S800 *in, S800Calc *out);

   /**This method will be removed.  It is being replaced by MakeCalibratedCRDC and
      GetCalibratedCrdcPads.
   */
   void CrdcCal(std::vector<Short_t> channel, std::vector<Short_t> data, Int_t id);

   /**Method to form and calibrated the CRDC pad distribution.  Channels a vector holding each
      read of each PAD that fired in the event. Data is a vector containning the corresponding read values
      id is 0 or 1 specfying which CRDC is built.
   */
   std::vector<Float_t> GetCalibratedCrdcPads(std::vector<Short_t> channels, std::vector<Short_t> data, Int_t id,
                                              vector<Float_t> &PedSubtractedPads);

   /**Method to build a fully calibrated CRDC object.  The calibrated values will be set in theCRDC.  Inputs are the
      same as for GetCalibratedCrdcPads

    */
   void MakeCalibratedCRDC(CRDC *theCRDC, std::vector<Short_t> channels, std::vector<Short_t> data, Float_t tac,
                           Float_t anode, Int_t id);

   /**Method will be removed.  Will be replaced by CalcX2()
    */
   Float_t CalcX();
   Float_t CalcX2(CRDC *theCRDC);

   /**Method will be removed.
    */
   void SetCrdc(std::vector<Short_t> channel, std::vector<Short_t> data, Float_t tac, Float_t anode, Int_t id);

   /**Method will be removed.
    */
   CRDC GetCrdc() { return fcrdc; }

   Float_t TimeOffset(Float_t time1, Float_t time2);
   Float_t TimeOffset(Float_t time);
   void SetTof(GTimeOfFlight *tof);
   TOF GetTof() { return ftof; }

   void ReadICCalibration(const char *filename);
   std::vector<Float_t> ICCal(std::vector<int> chan, std::vector<float> raw);
   Float_t ICSum(std::vector<Float_t> cal);
   Float_t ICDE(Float_t sum, Float_t x, Float_t y);

   void SetTS800(Short_t ts800) { fts800 = ts800; }

   std::vector<Float_t> GetCRDCCal() { return fcrdccal; }

private:
   S800Settings *fSett;
   std::vector<std::vector<Float_t>> fped;
   std::vector<std::vector<Float_t>> fslope;
   std::vector<std::vector<Float_t>> foffset;
   std::vector<std::vector<Int_t>> fbad;
   std::vector<Float_t> fcrdccal;
   Short_t fts800;

   std::vector<Float_t> fICoffset;
   std::vector<Float_t> fICslope;
   Float_t fde_slope;
   Float_t fde_offset;

   CRDC fcrdc;
   TOF ftof;
};

#endif
