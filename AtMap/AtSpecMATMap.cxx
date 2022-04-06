#include "AtSpecMATMap.h"

#include <TAxis.h>
#include <math.h>
#include <boost/multi_array/base.hpp>
#include <boost/multi_array/extent_gen.hpp>
#include <boost/multi_array/multi_array_ref.hpp>
#include <boost/multi_array/subarray.hpp>
#include <Math/Point2D.h>
#include <fstream> // IWYU pragma: keep
#include <iostream>
#include <algorithm>
#include <vector>

#include "TH2Poly.h"
#include "Rtypes.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"

using XYPoint = ROOT::Math::XYPoint;

AtSpecMATMap::AtSpecMATMap(Int_t numPads) : AtMap()
{
   AtPadCoord.resize(boost::extents[numPads][4][2]);
   std::fill(AtPadCoord.data(), AtPadCoord.data() + AtPadCoord.num_elements(), 0);
   std::cout << " SpecMAT Map initialized " << std::endl;
   std::cout << " SpecMAT Pad Coordinates container initialized " << std::endl;
   fNumberPads = numPads;
}

AtSpecMATMap::~AtSpecMATMap() {}

void AtSpecMATMap::Dump()
{

   std::ofstream coordmap;
   coordmap.open("coordmap_specmat.txt");
   coordmap << " SpecMAT Triangular pad coordinates (centers) - Pad Index - (x,y)" << std::endl;

   for (index i = 0; i < 3174; ++i) {
      coordmap << i << "  ";
      coordmap << AtPadCoord[i][3][0] << "  " << AtPadCoord[i][3][1];
      coordmap << std::endl;
   }

   coordmap.close();
}

std::vector<double> TrianglesGenerator()
{
   std::vector<double> allCoordinates;
   int nRaws = 23;
   int firstElement = 1;
   int elementStep = 2;
   int nElementsInSegment = nRaws * ((2 * firstElement + (nRaws - 1) * elementStep) / 2);
   int nSegments = 6;
   int step = 8;
   int counter = 0;
   int a = 0;
   const int arrayLength = nElementsInSegment * nSegments;
   double x0[arrayLength], x1[arrayLength], x2[arrayLength], xCentre[arrayLength], x0next[arrayLength],
      x1next[arrayLength], x2next[arrayLength];
   double y0[arrayLength], y1[arrayLength], y2[arrayLength], yCentre[arrayLength], y0next[arrayLength],
      y1next[arrayLength], y2next[arrayLength];
   double h = 4.725;

   for (int i = 1; i < nRaws + 1; i++) {
      a = 1 + (i - 1) * 2;
      for (int j = 1; j < a + 1; j++) {
         if (i == 1) {
            x0[counter] = 0.;
            y0[counter] = 0.;

            x1[counter] = -h / sqrt(3);
            y1[counter] = h;

            x2[counter] = h / sqrt(3);
            y2[counter] = h;

            xCentre[counter] = 0.;
            yCentre[counter] = h / sqrt(3);
         } else {
            if (j % 2 == 0) {
               x0[counter] = x2[counter - 1];
               y0[counter] = y2[counter - 1];

               x1[counter] = x2[counter - a + 1];
               y1[counter] = y2[counter - a + 1];

               x2[counter] = x1[counter - a + 1];
               y2[counter] = y1[counter - a + 1];

               xCentre[counter] = x0[counter];
               yCentre[counter] = y0[counter] - h / sqrt(3);
            } else {
               x0[counter] = x0next[i - 2] + (j - 1) * (h / sqrt(3));
               y0[counter] = y0next[i - 2];

               x1[counter] = x1next[i - 2] + (j - 1) * (h / sqrt(3));
               y1[counter] = y1next[i - 2];

               x2[counter] = x2next[i - 2] + (j - 1) * (h / sqrt(3));
               y2[counter] = y2next[i - 2];

               xCentre[counter] = x0[counter];
               yCentre[counter] = y0[counter] + h / sqrt(3);
            }
         }
         counter += 1;
      }

      x0next[i - 1] = x0[counter - a] - h / sqrt(3);
      y0next[i - 1] = y0[counter - a] + h;

      x1next[i - 1] = x1[counter - a] - h / sqrt(3);
      y1next[i - 1] = y1[counter - a] + h;

      x2next[i - 1] = x2[counter - a] - h / sqrt(3);
      y2next[i - 1] = y2[counter - a] + h;
   }

   for (int i = 1; i < nSegments; i++) {
      for (int j = 0; j < nElementsInSegment; j++) {
         x0[i * nElementsInSegment + j] =
            0.5 * (x0[(i - 1) * nElementsInSegment + j] - 0) - sqrt(3) * 0.5 * (y0[(i - 1) * nElementsInSegment + j]);
         y0[i * nElementsInSegment + j] =
            sqrt(3) * 0.5 * (x0[(i - 1) * nElementsInSegment + j] - 0) + 0.5 * (y0[(i - 1) * nElementsInSegment + j]);

         x1[i * nElementsInSegment + j] =
            0.5 * (x1[(i - 1) * nElementsInSegment + j] - 0) - sqrt(3) * 0.5 * (y1[(i - 1) * nElementsInSegment + j]);
         y1[i * nElementsInSegment + j] =
            sqrt(3) * 0.5 * (x1[(i - 1) * nElementsInSegment + j] - 0) + 0.5 * (y1[(i - 1) * nElementsInSegment + j]);

         x2[i * nElementsInSegment + j] =
            0.5 * (x2[(i - 1) * nElementsInSegment + j] - 0) - sqrt(3) * 0.5 * (y2[(i - 1) * nElementsInSegment + j]);
         y2[i * nElementsInSegment + j] =
            sqrt(3) * 0.5 * (x2[(i - 1) * nElementsInSegment + j] - 0) + 0.5 * (y2[(i - 1) * nElementsInSegment + j]);

         xCentre[i * nElementsInSegment + j] = 0.5 * (xCentre[(i - 1) * nElementsInSegment + j] - 0) -
                                               sqrt(3) * 0.5 * (yCentre[(i - 1) * nElementsInSegment + j]);
         yCentre[i * nElementsInSegment + j] = sqrt(3) * 0.5 * (xCentre[(i - 1) * nElementsInSegment + j] - 0) +
                                               0.5 * (yCentre[(i - 1) * nElementsInSegment + j]);
      }
   }

   for (int i = 0; i < arrayLength; i++) {
      allCoordinates.push_back(x0[i]);
      allCoordinates.push_back(y0[i]);
      allCoordinates.push_back(x1[i]);
      allCoordinates.push_back(y1[i]);
      allCoordinates.push_back(x2[i]);
      allCoordinates.push_back(y2[i]);
      allCoordinates.push_back(xCentre[i]);
      allCoordinates.push_back(yCentre[i]);
   }
   return allCoordinates;
} // End TrianglesGenerator()

// TH2Poly *SpecMATHistoGenerator()
TH2Poly *AtSpecMATMap::GetPadPlane()
{
   // TH2Poly *SpecMAThisto = new TH2Poly();
   std::vector<double> arrayAllCoordinates = TrianglesGenerator();
   double x[3];
   double y[3];
   int arrayAllCoordinatesSize = arrayAllCoordinates.size();
   for (int i2 = 0; i2 < arrayAllCoordinatesSize / 8; i2++) {
      x[0] = arrayAllCoordinates[8 * i2];
      y[0] = arrayAllCoordinates[8 * i2 + 1];
      x[1] = arrayAllCoordinates[8 * i2 + 2];
      y[1] = arrayAllCoordinates[8 * i2 + 3];
      x[2] = arrayAllCoordinates[8 * i2 + 4];
      y[2] = arrayAllCoordinates[8 * i2 + 5];
      fPadPlane->AddBin(3, x, y);
   }

   if (kGUIMode)
      drawPadPlane();

   return fPadPlane;
} // SpecMATHistoGenerator()

void AtSpecMATMap::SpecMATPadPlane()
{

   std::vector<double> arrayAllCoordinates2;
   arrayAllCoordinates2 = TrianglesGenerator();

   int nElementsInSegment3;
   nElementsInSegment3 = arrayAllCoordinates2.size() / (6 * 8);

   TH2Poly *XY;
   XY = GetPadPlane();
   // XY = SpecMATHistoGenerator();
   XY->SetTitle("PadPlane X vs Y");
   XY->SetName("XY");
   XY->GetXaxis()->SetTitle("X (mm)");
   XY->GetYaxis()->SetTitle("Y (mm)");

   int currentBin;
   double InitialPoInt_X = 50;
   double InitialPoInt_Y = 50;
   double particleEnergy = 50;

   double binXcentroid;
   double binYcentroid;

   // Histogram filling example
   XY->Fill(InitialPoInt_X, InitialPoInt_Y, particleEnergy);
   XY->Draw("COLZ");

   // Obtain coordinates of the filled pad
   currentBin = XY->FindBin(InitialPoInt_X, InitialPoInt_Y, 0);

   if (currentBin < nElementsInSegment3) {
      binXcentroid = arrayAllCoordinates2[8 * (currentBin) + 6];
      binYcentroid = arrayAllCoordinates2[8 * (currentBin) + 7];
   } else if (currentBin >= nElementsInSegment3 && currentBin < nElementsInSegment3 * 2 - 1) {
      binXcentroid = arrayAllCoordinates2[8 * (currentBin + 1) + 6];
      binYcentroid = arrayAllCoordinates2[8 * (currentBin + 1) + 7];
   } else if (currentBin >= nElementsInSegment3 * 2 - 1 && currentBin < nElementsInSegment3 * 3 - 2) {
      binXcentroid = arrayAllCoordinates2[8 * (currentBin + 2) + 6];
      binYcentroid = arrayAllCoordinates2[8 * (currentBin + 2) + 7];
   } else if (currentBin >= nElementsInSegment3 * 3 - 2 && currentBin < nElementsInSegment3 * 4 - 3) {
      binXcentroid = arrayAllCoordinates2[8 * (currentBin + 3) + 6];
      binYcentroid = arrayAllCoordinates2[8 * (currentBin + 3) + 7];
   } else if (currentBin >= nElementsInSegment3 * 4 - 3 && currentBin < nElementsInSegment3 * 5 - 4) {
      binXcentroid = arrayAllCoordinates2[8 * (currentBin + 4) + 6];
      binYcentroid = arrayAllCoordinates2[8 * (currentBin + 4) + 7];
   } else if (currentBin >= nElementsInSegment3 * 5 - 4 && currentBin < nElementsInSegment3 * 6 - 5) {
      binXcentroid = arrayAllCoordinates2[8 * (currentBin + 5) + 6];
      binYcentroid = arrayAllCoordinates2[8 * (currentBin + 5) + 7];
   }

   std::cout << "Input" << std::endl;
   std::cout << "X: " << InitialPoInt_X << " Y: " << InitialPoInt_Y << std::endl;
   std::cout << "\nOutput" << std::endl;
   std::cout << "Bin: " << currentBin << " binCentroidX: " << binXcentroid << " binCentroidY: " << binYcentroid
             << std::endl;
} // SpecMATPadPlane

void AtSpecMATMap::GeneratePadPlane()
{

   std::cout << " SpecMAT Map : Generating the map geometry of SpecMAT " << std::endl;

   std::vector<double> arrayAllCoord;
   arrayAllCoord = TrianglesGenerator();

   for (int i = 0; i < fNumberPads; i++) {
      AtPadCoord[i][0][0] = arrayAllCoord[8 * i];     // x coordinate of first vertex pad
      AtPadCoord[i][0][1] = arrayAllCoord[8 * i + 1]; // y coordinate of first vertex pad
      AtPadCoord[i][1][0] = arrayAllCoord[8 * i + 2]; // x coordinate of second vertex pad
      AtPadCoord[i][1][1] = arrayAllCoord[8 * i + 3]; // y coordinate of second vertex pad
      AtPadCoord[i][2][0] = arrayAllCoord[8 * i + 4]; // x coordinate of third vertex pad
      AtPadCoord[i][2][1] = arrayAllCoord[8 * i + 5]; // y coordinate of third vertex pad
      AtPadCoord[i][3][0] = arrayAllCoord[8 * i + 6]; // x coordinate of pad center
      AtPadCoord[i][3][1] = arrayAllCoord[8 * i + 7]; // y coordinate of pad center
   }

   std::cout << " A total of  " << fNumberPads << " pads were generated  " << std::endl;
   kIsParsed = true;
}

XYPoint AtSpecMATMap::CalcPadCenter(Int_t PadRef)
{
   if (!kIsParsed) {

      std::cout << " AtSpecMATMap::CalcPadCenter Error : Pad plane has not been generated or parsed " << std::endl;
      return XYPoint(-9999, -9999);
   }

   if (PadRef != -1) { // Boost multi_array crashes with a negative index

      auto x = AtPadCoord[PadRef][3][0];
      auto y = AtPadCoord[PadRef][3][1];
      return XYPoint(x, y);

   } else {

      if (kDebug)
         std::cout << " AtSpecMATMap::CalcPadCenter Error : Pad not found" << std::endl;
      return XYPoint(-9999, -9999);
   }
}

ClassImp(AtSpecMATMap);
