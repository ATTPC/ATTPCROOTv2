#include "AtEDistortionModel.h"

#include <FairLogger.h>

#include <Math/Point2D.h>
#include <Math/Point2Dfwd.h> // for XYPoint
#include <Math/Point3D.h>    // for PositionVector3D
#include <Math/Point3Dfwd.h> // for RhoZPhiPoint, XYZPoint
#include <Math/Vector2D.h>
#include <Math/Vector2Dfwd.h> // for XYVector
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h> // for XYZVector

#include "TMath.h"

#include <cassert> // for assert
#include <iostream>
#include <memory> // for allocator

using XYZPoint = ROOT::Math::XYZPoint;
using XYZVector = ROOT::Math::XYZVector;
using XYPoint = ROOT::Math::XYPoint;
using XYVector = ROOT::Math::XYVector;

AtEDistortionModel::AtEDistortionModel() : AtSpaceChargeModel() {}

XYZPoint AtEDistortionModel::CorrectSpaceCharge(const XYZPoint &input)
{

   if (fDistortionMap->size() == 0) {
      LOG(error) << " Warning! Correction map is empty. No correction applied...";
      return input;
   }

   LOG(debug) << " Input point : " << input.X() << " " << input.Y() << " " << input.Z();

   Double_t inputRad = TMath::Sqrt(TMath::Power(input.X(), 2) + TMath::Power(input.Y(), 2));
   Double_t inputAng = TMath::ATan2(input.Y(), input.X());
   Double_t inputZ = input.Z();
   // TODO:
   // inputRad = input.Rho();
   // inputAng = input.Phi();

   inputZ = inputZ < 0 ? 0 : inputZ;
   inputZ = inputZ > 1000 ? 1000 : inputZ;

   auto [zcorr, radcorr, tracorr] = GetCorrectionFactors((Int_t)inputRad, (Int_t)inputZ);

   Double_t outputRad = TMath::Sqrt(TMath::Power(inputRad + radcorr, 2) + TMath::Power(tracorr, 2));
   Double_t outputAng = inputAng + TMath::ATan2(tracorr, inputRad + radcorr);

   XYZPoint output;
   output.SetX(outputRad * TMath::Cos(outputAng));
   output.SetY(outputRad * TMath::Sin(outputAng));
   output.SetZ(input.Z() - zcorr);
   // TODO:
   // output.SetRho(outputRad)
   // output.SetPhi(outputAng)

   LOG(debug) << " Output point : " << output.X() << " " << output.Y() << " " << output.Z();

   return output;
}

XYZPoint AtEDistortionModel::ApplySpaceCharge(const XYZPoint &input)
{
   return {};
}

Bool_t
AtEDistortionModel::SetCorrectionMaps(const std::string &zlut, const std::string &radlut, const std::string &tralut)
{

   fZLUT.open(zlut, std::ifstream::in);
   fRadLUT.open(radlut, std::ifstream::in);
   fTraLUT.open(tralut, std::ifstream::in);

   if (!fZLUT.fail() && !fRadLUT.fail() && !fTraLUT.fail()) {

      SetBufferMap(fZLUT, fZMap.get());
      SetBufferMap(fRadLUT, fRadMap.get());
      SetBufferMap(fTraLUT, fTraMap.get());

      SetGlobalMap(fZMap.get(), fRadMap.get(), fTraMap.get());

      return kTRUE;

   } else {

      std::cerr << " AtEDistortionModel::SetCorrectionMaps - Error! Missing LUT "
                << "\n";
      std::cerr << zlut << "\n";
      std::cerr << radlut << "\n";
      std::cerr << tralut << "\n";
      return kFALSE;
   }
}

std::tuple<Double_t, Double_t, Double_t>
AtEDistortionModel::GetCorrectionFactors(const Int_t &radius, const Int_t &zpos)
{

   if (fDistortionMap->size() == 0) {
      //  LOG(error) << " Warning! Correction map is empty. ";
      return std::make_tuple(0, 0, 0);
   }

   EFieldMapRef ref{radius, zpos};
   if (fDistortionMap->find(ref) == fDistortionMap->end()) {
      // LOG(error) << " Correction factor not found. ";
      return std::make_tuple(0, 0, 0);
   } else {
      auto corr = fDistortionMap->at(ref);
      return std::make_tuple(corr._zcorr, corr._radcorr, corr._tracorr);
   }
}

void AtEDistortionModel::SetBufferMap(std::ifstream &lut, BufferMap *map)
{

   Int_t radiusCnt = 0;

   while (!lut.eof()) {

      std::string line;
      std::getline(lut, line, '\r');
      Int_t radius;
      Double_t value;
      std::istringstream buffer(line);
      Int_t cnt = 0;

      // N.B. e20009 correction files have an extra parameter at the beginning of each line. That parameter is the
      // radius in mm buffer >> radius;
      radius = radiusCnt;

      while (buffer >> value) {

         EFieldMapRef ref{radius, cnt};
         map->emplace(ref, value);
         ++cnt;
      }

      ++radiusCnt;
   }
}

void AtEDistortionModel::SetGlobalMap(BufferMap *zmap, BufferMap *radmap, BufferMap *tramap)
{

   LOG(info) << " Z map size : " << zmap->size();
   LOG(info) << " Rad map size : " << radmap->size();
   LOG(info) << " Tra map size : " << tramap->size();

   // NB: This is not a necessary condition but it works for the present case
   assert((zmap->size() == radmap->size()) && (radmap->size() == tramap->size()));

   for (const auto &[key, zcorr] : *zmap) {
      auto radcorr = radmap->at(key);
      auto tracorr = tramap->at(key);
      EFieldCorr corr{zcorr, radcorr, tracorr};
      fDistortionMap->emplace(key, corr);
      LOG(debug) << key << " " << zcorr << " " << radcorr << " " << tracorr;
   }

   auto hashT = fDistortionMap->hash_function();
   EFieldMapRef hashRef{200, 200};

   std::cout << " Global map size : " << fDistortionMap->size() << "\n";
   std::cout << " Hash value for (200,200)     : " << hashT(hashRef) << "\n";
}

std::ostream &operator<<(std::ostream &os, const EFieldMapRef &t)
{
   os << " " << t._rad << " - " << t._zpos << " ";
   return os;
}

bool operator<(const EFieldMapRef &l, const EFieldMapRef &r)
{
   return std::hash<EFieldMapRef>()(l) < std::hash<EFieldMapRef>()(r);
}

bool operator==(const EFieldMapRef &l, const EFieldMapRef &r)
{
   return l._rad == r._rad && l._zpos == r._zpos;
}
