#ifndef ATEDISTORTIONMODEL_H
#define ATEDISTORTIONMODEL_H

#include "AtSpaceChargeModel.h"

#include <Rtypes.h> // for Double_t

#include <cstddef>       // for size_t
#include <fstream>       // for ifstream
#include <functional>    // for hash
#include <memory>        // for make_unique, unique_ptr
#include <string>        // for string
#include <tuple>         // for tuple
#include <unordered_map> // for unordered_map

class AtDigiPar;

struct EFieldCorr {
   Double_t _zcorr;
   Double_t _radcorr;
   Double_t _tracorr;
};

struct EFieldMapRef {
   Int_t _rad{0};  //! radius integer value
   Int_t _zpos{0}; //! z integer value
};

bool operator<(const EFieldMapRef &l, const EFieldMapRef &r);
bool operator==(const EFieldMapRef &l, const EFieldMapRef &r);
std::ostream &operator<<(std::ostream &os, const EFieldMapRef &t);

namespace std {
template <>
struct hash<EFieldMapRef> {
   inline size_t operator()(const EFieldMapRef &x) const { return x._rad + x._zpos * 10000; }
};
} // namespace std

using eDistortionMap = std::unordered_map<EFieldMapRef, EFieldCorr>; // Global LUT
using BufferMap = std::unordered_map<EFieldMapRef, Double_t>;        //! To parse individual LUT

/**
 * @brief Model for E-field distortion correction
 *
 *
 *
 *     The implemented correction is applied to every hit
 *     Radial, transversal and longitudinal (z) corrections as a function of radial distance and z (map keys).
 *     The Class EFieldCorr contains the correction values
 */

class AtEDistortionModel : public AtSpaceChargeModel {

private:
   void SetBufferMap(std::ifstream &lut, BufferMap *map);
   void SetGlobalMap(BufferMap *zmap, BufferMap *radmap, BufferMap *tramap);

   std::ifstream fZLUT{0};
   std::ifstream fRadLUT{0};
   std::ifstream fTraLUT{0};

   std::unique_ptr<eDistortionMap> fDistortionMap{std::make_unique<eDistortionMap>()};
   std::unique_ptr<BufferMap> fZMap{std::make_unique<BufferMap>()};
   std::unique_ptr<BufferMap> fRadMap{std::make_unique<BufferMap>()};
   std::unique_ptr<BufferMap> fTraMap{std::make_unique<BufferMap>()};

public:
   AtEDistortionModel();

   virtual XYZPoint CorrectSpaceCharge(const XYZPoint &directInputPosition) override;
   virtual XYZPoint ApplySpaceCharge(const XYZPoint &reverseInputPosition) override;

   void LoadParameters(AtDigiPar *par) override{};

   Bool_t SetCorrectionMaps(const std::string &zlut, const std::string &radlut,
                            const std::string &tralut); //! Set the mapping files via file path
   std::tuple<Double_t, Double_t, Double_t> GetCorrectionFactors(const Int_t &radius, const Int_t &zpos);
};
#endif /* ATEDISTORTIONMODEL_H */
