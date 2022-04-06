#ifndef TINVERSEMAP_H_
#define TINVERSEMAP_H_

#include <TNamed.h>
#include <Rtypes.h>
#include <map>
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>

class TBuffer;
class TClass;
class TMemberInspector;
class TSpline3;

class TInverseMap : public TNamed {

public:
   TInverseMap(const char *filename);
   // TInverseMap(std::vector<std::string> &fileList);
   TInverseMap();
   static TInverseMap *Get(const char *filename = "");
   virtual ~TInverseMap();

   virtual void Print(Option_t *opt = "") const; //{ ; }
   virtual void Clear(Option_t *opt = "") { ; }

   float Ata(int degree, double xfp, double afp, double yfp, double bfp) const;
   float Ata(int degree, double xfp, double afp, double yfp, double bfp, double z);
   // float Ata(int,const TS800*);
   float Bta(int degree, double xfp, double afp, double yfp, double bfp) const;
   float Bta(int degree, double xfp, double afp, double yfp, double bfp, double z);
   // float Bta(int,const TS800*);
   float Yta(int degree, double xfp, double afp, double yfp, double bfp) const;
   float Yta(int degree, double xfp, double afp, double yfp, double bfp, double z);
   // float Yta(int,const TS800*);
   float Dta(int degree, double xfp, double afp, double yfp, double bfp) const;
   float Dta(int degree, double xfp, double afp, double yfp, double bfp, double z);
   // float Dta(int,const TS800*);

   float MapCalc(int, int, float *) const;
   float MapCalc_s(int order, int par, float *input, double z);

   void SetDistPivotTarget(std::vector<Double_t> vec)
   {
      std::cout << "check setDistPivotTarget " << vec.size() << " " << vec.at(2) << std::endl;
      fMapDist_v = vec;
   };

   int Size() { return fMap.size(); }

   bool ReadMultiMapFile(std::vector<std::string> &str);

private:
   // TInverseMap(const char* filename);
   static TInverseMap *fInverseMap;

   bool ReadMapFile(const char *filename);
   // bool ReadMultiMapFile(std::vector<std::string> &str);

   struct InvMapRow {
      double coefficient;
      int order;
      int exp[6];
   };

   struct InvMapRowS {
      TSpline3 *coefficient;
      int order;
      int exp[6];
   };

   // data cleared on reset; i.e. Read new inverse map.
   std::map<int, std::vector<InvMapRow>> fMap;
   std::map<int, std::vector<InvMapRowS>> fMap_s;
   std::vector<std::map<int, std::vector<InvMapRow>>> fMap_v;
   std::vector<Double_t> fMapDist_v;
   float fBrho;
   int fMass;
   int fCharge;
   Int_t fsize;
   std::string info;

   ClassDef(TInverseMap, 0)
};
#endif
