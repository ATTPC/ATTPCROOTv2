/*********************************************************************
 *   ATTPC Mapping Class	AtTpcMap.cxx			             *
 *   Author: Y. Ayyad            				     *
 *   Log: 13-02-2015 17:16 JST					     *
 *								     *
 *********************************************************************/

#include "AtTpcMap.h"

#include <FairLogger.h>

#include <Math/Point2D.h>
#include <Rtypes.h>
#include <TH2Poly.h>
#include <TMath.h>
#include <TMathBase.h>

#include <boost/multi_array/base.hpp>
#include <boost/multi_array/extent_gen.hpp>
#include <boost/multi_array/multi_array_ref.hpp>
#include <boost/multi_array/subarray.hpp>

#include <algorithm>
#include <cmath>
#include <fstream> // IWYU pragma: keep
#include <iostream>
#include <vector>

#undef BOOST_MULTI_ARRAY_NO_GENERATORS
#define BOOST_MULTI_ARRAY_NO_GENERATORS

using std::cout;
using std::endl;
using XYPoint = ROOT::Math::XYPoint;

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";

AtTpcMap::AtTpcMap() : AtMap()
{
   AtPadCoord.resize(boost::extents[10240][3][2]);
   std::fill(AtPadCoord.data(), AtPadCoord.data() + AtPadCoord.num_elements(), 0);
   std::cout << " ATTPC Map initialized " << std::endl;
   std::cout << " ATTPC Pad Coordinates container initialized " << std::endl;
   fNumberPads = 10240;
}

AtTpcMap::~AtTpcMap() = default;

void AtTpcMap::Dump()
{

   std::ofstream coordmap;
   coordmap.open("coordmap.txt");

   int values = 0;
   for (index i = 0; i != 10240; ++i) {
      coordmap << i << "  ";
      for (index j = 0; j != 3; ++j) {
         for (index k = 0; k != 2; ++k) {
            std::cout << " ATTPC Triangular pad coordinates - Pad Index : " << i << "   X(" << j << ")  -  Y(" << k
                      << ") :" << AtPadCoord[i][j][k] << std::endl;
            coordmap << AtPadCoord[i][j][k] << "  ";
         } // k
      }    // j

      coordmap << std::endl;

   } // i

   coordmap.close();
}

void AtTpcMap::GeneratePadPlane()
{
   if (fPadPlane) {
      LOG(error) << "Skipping generation of pad plane because it was already parsed!";
      return;
   }

   std::cout << " ATTPC Map : Generating the map geometry of the ATTPC " << std::endl;
   // Local variables
   Float_t pads_in_half_hex = 0;
   Float_t pads_in_hex = 0;
   Float_t row_length = 0;
   Float_t pads_in_half_row = 0;
   Int_t pads_out_half_hex = 0;
   Int_t pads_in_row = 0;
   Int_t ort = 0;
   Float_t pad_x_off = 0;
   Float_t pad_y_off = 0;
   Float_t tmp_pad_x_off = 0;
   Float_t tmp_pad_y_off = 0;

   Float_t small_z_spacing = 2 * 25.4 / 1000.;
   Float_t small_tri_side = 184. * 25.4 / 1000.;
   Double_t umega_radius = 10826.772 * 25.4 / 1000.;
   Float_t beam_image_radius = 4842.52 * 25.4 / 1000.;
   Int_t pad_index = 0;
   Int_t pad_index_aux = 0;

   Float_t small_x_spacing = 2. * small_z_spacing / TMath::Sqrt(3.);
   Float_t small_y_spacing = small_x_spacing * TMath::Sqrt(3.);
   Float_t dotted_s_tri_side = 4. * small_x_spacing + small_tri_side;
   Float_t dotted_s_tri_hi = dotted_s_tri_side * TMath::Sqrt(3.) / 2.;
   Float_t dotted_l_tri_side = 2. * dotted_s_tri_side;
   Float_t dotted_l_tri_hi = dotted_l_tri_side * TMath::Sqrt(3.) / 2.;
   Float_t large_x_spacing = small_x_spacing;
   Float_t large_y_spacing = small_y_spacing;
   Float_t large_tri_side = dotted_l_tri_side - 4. * large_x_spacing;
   Float_t large_tri_hi = dotted_l_tri_side * TMath::Sqrt(3.) / 2.;
   // Float_t row_len_s = 2**TMath::Ceil(TMath::Log(beam_image_radius/dotted_s_tri_side)/TMath::Log(2.0));
   Float_t row_len_s = pow(2, TMath::Ceil(TMath::Log(beam_image_radius / dotted_s_tri_side) / TMath::Log(2.0)));
   Float_t row_len_l = TMath::Floor(umega_radius / dotted_l_tri_hi);

   Float_t xoff = 0.;
   Float_t yoff = 0.;

   for (Int_t j = 0; j < row_len_l; j++) {
      pads_in_half_hex = 0;
      pads_in_hex = 0;
      // row_length = TMath::Abs(sqrt(umega_radius**2 - (j*dotted_l_tri_hi + dotted_l_tri_hi/2.)**2));
      row_length = TMath::Abs(sqrt(pow(umega_radius, 2) - pow((j * dotted_l_tri_hi + dotted_l_tri_hi / 2.), 2)));

      if (j < row_len_s / 2.) {

         pads_in_half_hex = (2 * row_len_s - 2 * j) / 4.;
         pads_in_hex = 2 * row_len_s - 1. - 2. * j;

      } // if row_len_s

      pads_in_half_row = row_length / dotted_l_tri_side;
      pads_out_half_hex = static_cast<Int_t>(TMath::Nint(2 * (pads_in_half_row - pads_in_half_hex)));
      pads_in_row = 2 * pads_out_half_hex + 4 * pads_in_half_hex - 1;

      ort = 1;

      for (Int_t i = 0; i < pads_in_row; i++) {

         // set initial ort
         if (i == 0) {
            if (j % 2 == 0)
               ort = -1;
            if (((pads_in_row - 1) / 2) % 2 == 1)
               ort = -ort;

         } // i==0
         else
            ort = -ort;

         pad_x_off = -(pads_in_half_hex + pads_out_half_hex / 2.) * dotted_l_tri_side + i * dotted_l_tri_side / 2. +
                     2. * large_x_spacing + xoff;

         if (i < pads_out_half_hex || i > (pads_in_hex + pads_out_half_hex - 1) || j > (row_len_s / 2. - 1)) {

            pad_y_off = j * dotted_l_tri_hi + large_y_spacing + yoff;
            if (ort == -1)
               pad_y_off += large_tri_hi;

            fill_coord(pad_index, pad_x_off, pad_y_off, large_tri_side, ort);
            pad_index += 1;

         } // if
         else {

            pad_y_off = j * dotted_l_tri_hi + large_y_spacing + yoff;
            if (ort == -1)
               pad_y_off = j * dotted_l_tri_hi + 2 * dotted_s_tri_hi - small_y_spacing + yoff;
            fill_coord(pad_index, pad_x_off, pad_y_off, small_tri_side, ort);

            pad_index += 1;
            tmp_pad_x_off = pad_x_off + dotted_s_tri_side / 2.;
            tmp_pad_y_off = pad_y_off + ort * dotted_s_tri_hi - 2 * ort * small_y_spacing;
            fill_coord(pad_index, tmp_pad_x_off, tmp_pad_y_off, small_tri_side, -ort);

            pad_index += 1;
            tmp_pad_y_off = pad_y_off + ort * dotted_s_tri_hi;
            fill_coord(pad_index, tmp_pad_x_off, tmp_pad_y_off, small_tri_side, ort);

            pad_index += 1;
            tmp_pad_x_off = pad_x_off + dotted_s_tri_side;
            fill_coord(pad_index, tmp_pad_x_off, pad_y_off, small_tri_side, ort);
            pad_index += 1;
         }

      } // pads_in_row loop

   } // row_len_l

   for (Int_t i = 0; i < pad_index; i++) {
      for (Int_t j = 0; j < 3; j++) {
         AtPadCoord[i + pad_index][j][0] = AtPadCoord[i][j][0];
         AtPadCoord[i + pad_index][j][1] = -AtPadCoord[i][j][1];
      }
      pad_index_aux++;
   }

   // fPadInd = pad_index + pad_index_aux;
   std::cout << "created pads: " << pad_index + pad_index_aux << std::endl;
   kIsParsed = true;

   if (fPadPlane != nullptr)
      delete fPadPlane;
   fPadPlane = new TH2Poly(); // NOLINT

   fPadPlane->SetName("ATTPC_Plane");
   fPadPlane->SetTitle("ATTPC_Plane");

   // for (Int_t i = 0; i < fPadInd; i++) {
   for (Int_t i = 0; i < fNumberPads; i++) {

      Double_t px[] = {AtPadCoord[i][0][0], AtPadCoord[i][1][0], AtPadCoord[i][2][0], AtPadCoord[i][0][0]};
      Double_t py[] = {AtPadCoord[i][0][1], AtPadCoord[i][1][1], AtPadCoord[i][2][1], AtPadCoord[i][0][1]};
      fPadPlane->AddBin(4, px, py);
   }

   fPadPlane->ChangePartition(500, 500);
}

Int_t AtTpcMap::fill_coord(int pindex, float padxoff, float padyoff, float triside, float fort)
{
   AtPadCoord[pindex][0][0] = padxoff;
   AtPadCoord[pindex][0][1] = padyoff;
   AtPadCoord[pindex][1][0] = padxoff + triside / 2.;
   AtPadCoord[pindex][1][1] = padyoff + fort * triside * TMath::Sqrt(3.) / 2.;
   AtPadCoord[pindex][2][0] = padxoff + triside;
   AtPadCoord[pindex][2][1] = padyoff;
   return 0;
}

XYPoint AtTpcMap::CalcPadCenter(Int_t PadRef)
{
   if (!kIsParsed) {
      LOG(error) << " AtTpcMap::CalcPadCenter Error : Pad plane has not been generated or parsed ";
      return {-9999, -9999};
   }
   if (PadRef < 0) { // Boost multi_array crashes with a negative index
      LOG(debug) << " AtTpcMap::CalcPadCenter Error : Pad not found";
      return {-9999, -9999};
   }

   Float_t x = (AtPadCoord[PadRef][0][0] + AtPadCoord[PadRef][1][0] + AtPadCoord[PadRef][2][0]) / 3.;
   Float_t y = (AtPadCoord[PadRef][0][1] + AtPadCoord[PadRef][1][1] + AtPadCoord[PadRef][2][1]) / 3.;
   return {x, y};
}
ClassImp(AtTpcMap)
