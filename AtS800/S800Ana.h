#ifndef S800Ana_H
#define S800Ana_H

#include <Rtypes.h>
#include <TObject.h>

#include <vector>

// AtTPCROOT classes

class FairLogger;
class S800Calc;
class TBuffer;
class TClass;
class TCutG;
class TMemberInspector;
class TString;

class S800Ana : public TObject {
private:
   FairLogger *fLogger;

   std::vector<Double_t> fParameters;
   std::vector<Double_t> fTofObjCorr;
   std::vector<Double_t> fE1dECorr;
   std::vector<Double_t> fMTDCObjRange;
   std::vector<Double_t> fMTDCXfRange;

   std::vector<TCutG *> fcutPID1;
   std::vector<TCutG *> fcutPID2;
   std::vector<TCutG *> fcutPID3;

   Double_t fXfObj_ToF;
   Double_t fObjCorr_ToF;
   Double_t fICSum_E;
   Double_t fE1_dE;
   Double_t fX0;
   Double_t fX1;
   Double_t fY0{0};
   Double_t fY1{0};
   Double_t fAfp;
   Double_t fBfp{0};

public:
   S800Ana();
   ~S800Ana() = default;

   void SetPID1cut(std::vector<TString> file);
   void SetPID2cut(std::vector<TString> file);
   void SetPID3cut(std::vector<TString> file);
   void SetParameters(std::vector<Double_t> vec);
   void SetTofObjCorr(std::vector<Double_t> vec);
   void SetE1dECorr(std::vector<Double_t> vec);
   void SetMTDCObjRange(std::vector<Double_t> vec);
   void SetMTDCXfRange(std::vector<Double_t> vec);

   std::vector<Double_t> GetParameters();
   std::vector<Double_t> GetTofObjCorr();
   std::vector<Double_t> GetMTDCObjRange();
   std::vector<Double_t> GetMTDCXfRange();
   Double_t GetXfObj_ToF();
   Double_t GetObjCorr_ToF();
   Double_t GetICSum_E();
   Double_t GetE1_dE();
   std::vector<Double_t> GetFpVariables();

   Bool_t isInPID(S800Calc *s800calc);
   void Calc(S800Calc *s800calc);

   // void InitStatus Init();
   // virtual void Exec(Option_t *opt);

private:
   void Reset();

   ClassDef(S800Ana, 1);
};

#endif
