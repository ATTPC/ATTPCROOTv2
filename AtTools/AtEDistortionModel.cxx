#include "AtEDistortionModel.h"

#include "AtDigiPar.h"

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

#include <cmath>
#include <memory> // for allocator

using XYZPoint = ROOT::Math::XYZPoint;
using XYZVector = ROOT::Math::XYZVector;
using XYPoint = ROOT::Math::XYPoint;
using XYVector = ROOT::Math::XYVector;

EFieldMapRef::EFieldMapRef() {}

EFieldMapRef::EFieldMapRef(Int_t rad, Int_t zpos) : _rad(rad), _zpos(zpos) {}

AtEDistortionModel::AtEDistortionModel() : AtSpaceChargeModel() {}

XYZPoint AtEDistortionModel::CorrectSpaceCharge(const XYZPoint &input)
{

   if (fDistortionMap->size() == 0) {
      LOG(error) << " Warning! Correction map is empty. No correction applied..."
                 << "\n";
      return XYZPoint(input.X(), input.Y(), input.Z());
   }

   LOG(debug) << " Input point : " << input.X() << " " << input.Y() << " " << input.Z() << "\n";

   Double_t inputRad = TMath::Sqrt(TMath::Power(input.X(), 2) + TMath::Power(input.Y(), 2));
   Double_t inputAng = TMath::ATan2(input.Y(), input.X());
   Double_t inputZ = input.Z();

   inputZ = inputZ < 0 ? 0 : inputZ;
   inputZ = inputZ > 1000 ? 1000 : inputZ;

   auto [zcorr, radcorr, tracorr] = GetCorrectionFactors((Int_t)inputRad, (Int_t)inputZ);

   Double_t outputRad = TMath::Sqrt(TMath::Power(inputRad + radcorr, 2) + TMath::Power(tracorr, 2));
   Double_t outputAng = inputAng + TMath::ATan2(tracorr, inputRad + radcorr);

   XYZPoint output;
   output.SetX(outputRad * TMath::Cos(outputAng));
   output.SetY(outputRad * TMath::Sin(outputAng));
   output.SetZ(input.Z() - zcorr);

   LOG(debug) << " Output point : " << output.X() << " " << output.Y() << " " << output.Z() << "\n";

   return output;
}

XYZPoint AtEDistortionModel::ApplySpaceCharge(const XYZPoint &input)
{
   XYZPoint p;
   return p;
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

   return kTRUE;
}

std::tuple<Double_t, Double_t, Double_t>
AtEDistortionModel::GetCorrectionFactors(const Int_t &radius, const Int_t &zpos)
{

   if (fDistortionMap->size() == 0) {
      LOG(error) << " Warning! Correction map is empty. "
                 << "\n";
      return std::make_tuple(0, 0, 0);
   }

   EFieldMapRef ref(radius, zpos);
   if (fDistortionMap->find(ref) == fDistortionMap->end()) {
      LOG(error) << " Correction factor not found. "
                 << "\n";
      return std::make_tuple(0, 0, 0);
   } else {
      auto corr = fDistortionMap->at(ref);
      return std::make_tuple(corr.GetZCorr(), corr.GetRadCorr(), corr.GetTraCorr());
   }
}

void AtEDistortionModel::SetBufferMap(std::ifstream &lut, BufferMap *map)
{

   while (!lut.eof()) {

      std::string line;
      std::getline(lut, line, '\r');
      Int_t radius;
      Double_t value;
      std::istringstream buffer(line);
      Int_t cnt = 0;

      buffer >> radius;

      while (buffer >> value) {

         EFieldMapRef ref(radius, cnt);
         map->emplace(ref, value);
         ++cnt;
      }
   }
}

void AtEDistortionModel::SetGlobalMap(BufferMap *zmap, BufferMap *radmap, BufferMap *tramap)
{

   std::cout << " Z map size : " << zmap->size() << "\n";
   std::cout << " Rad map size : " << radmap->size() << "\n";
   std::cout << " Tra map size : " << tramap->size() << "\n";

   // NB: This is not a necessary condition but it works for the present case
   assert((zmap->size() == radmap->size()) && (radmap->size() == tramap->size()));

   for (const auto &[key, zcorr] : *zmap) {
      auto radcorr = radmap->at(key);
      auto tracorr = tramap->at(key);
      EFieldCorr corr(zcorr, radcorr, tracorr);
      fDistortionMap->emplace(key, corr);
      LOG(debug) << key << " " << zcorr << " " << radcorr << " " << tracorr << '\n';
   }

   auto hashT = fDistortionMap->hash_function();
   EFieldMapRef hashRef(200, 200);

   std::cout << " Global map size : " << fDistortionMap->size() << "\n";
   std::cout << " Hash value for (200,200)     : " << hashT(hashRef) << "\n";
}

void AtEDistortionModel::LoadParameters(AtDigiPar *par)
{
   if (par == nullptr)
      return;
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
