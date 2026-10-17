#include "AtPadLookup.h"

#include "AtGadgetIIMap.h"
#include "AtMap.h"
#include "AtTpcMap.h"

#include <Math/Point2D.h>
#include <Math/Point2Dfwd.h>
#include <TAxis.h>
#include <TGraph.h>
#include <TH2Poly.h>
#include <TList.h>
#include <TRandom3.h>

#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

using XYPoint = ROOT::Math::XYPoint;

namespace {
/**
 * A map with a few pads of different shapes (squares, triangles, one concave pad) and gaps between them, so there are
 * pad edges everywhere and points that are in no pad.
 */
class TestMap : public AtMap {
public:
   void Dump() override {}
   XYPoint CalcPadCenter(Int_t) override { return {0, 0}; }
   Int_t BinToPad(Int_t binval) override { return binval - 1; }
   void GeneratePadPlane() override
   {
      fPadPlane = new TH2Poly("testPlane", "testPlane", 0, 13, 0, 13); // NOLINT: owned by gDirectory
      const double pitch = 1.1, size = 1.0;
      for (int i = 0; i < 10; ++i) {
         for (int j = 0; j < 10; ++j) {
            double x0 = 1 + i * pitch, y0 = 1 + j * pitch;
            if ((i + j) % 3 == 0) {
               double x[3] = {x0, x0 + size, x0};
               double y[3] = {y0, y0, y0 + size};
               fPadPlane->AddBin(3, x, y);
            } else if ((i + j) % 7 == 0) { // An L shape
               double x[6] = {x0, x0 + size, x0 + size, x0 + 0.4, x0 + 0.4, x0};
               double y[6] = {y0, y0, y0 + 0.4, y0 + 0.4, y0 + size, y0 + size};
               fPadPlane->AddBin(6, x, y);
            } else {
               double x[4] = {x0, x0 + size, x0 + size, x0};
               double y[4] = {y0, y0, y0 + size, y0 + size};
               fPadPlane->AddBin(4, x, y);
            }
         }
      }
      fNumberPads = 100;
   }
};

/// The answer FindBin gives, from a copy of the plane
int Expected(TH2Poly &plane, AtMap &map, const XYPoint &p)
{
   auto bin = plane.FindBin(p.X(), p.Y());
   return bin < 0 ? -1 : map.BinToPad(bin);
}

/// The lookup must give the same pad as FindBin at every one of the points
void CheckPoints(AtMap &map, TH2Poly &plane, const std::vector<XYPoint> &points)
{
   int mismatches = 0;
   for (auto &p : points)
      if (map.GetPadNum(p) != Expected(plane, map, p))
         ++mismatches;
   EXPECT_EQ(mismatches, 0);
}
} // namespace

TEST(AtPadLookupTest, SyntheticPlaneMatchesFindBin)
{
   TestMap map;
   map.GeneratePadPlane();
   std::unique_ptr<TH2Poly> plane(map.GetPadPlane());

   TRandom3 rand(1);
   std::vector<XYPoint> points;
   for (int i = 0; i < 200000; ++i) // Inside, around and outside of the pads and the histogram range
      points.emplace_back(rand.Uniform(-2, 15), rand.Uniform(-2, 15));
   for (int i = 0; i < 100000; ++i) // Close together, so most cells are used many times
      points.emplace_back(5.5 + rand.Gaus(0, 0.5), 6.5 + rand.Gaus(0, 0.5));

   // Points on and very close to the edges and corners of every pad
   for (auto *obj : *plane->GetBins()) {
      auto *graph = dynamic_cast<TGraph *>(static_cast<TH2PolyBin *>(obj)->GetPolygon());
      ASSERT_NE(graph, nullptr);
      int n = graph->GetN();
      for (int i = 0; i < n; ++i) {
         double x0 = graph->GetX()[i], y0 = graph->GetY()[i];
         double x1 = graph->GetX()[(i + 1) % n], y1 = graph->GetY()[(i + 1) % n];
         for (double t : {0., 0.25, 0.5, 1.})
            for (double d : {0., 1e-12, 1e-9, 1e-7, 1e-5, 1e-3, -1e-9, -1e-5}) {
               double x = x0 + t * (x1 - x0), y = y0 + t * (y1 - y0);
               points.emplace_back(x + d, y);
               points.emplace_back(x, y + d);
               points.emplace_back(x + d, y - d);
            }
      }
   }
   CheckPoints(map, *plane, points);
}

TEST(AtPadLookupTest, OutsideTheHistogramRangeIsNoPad)
{
   TestMap map;
   map.GeneratePadPlane();
   for (int pass = 0; pass < 3; ++pass) {
      EXPECT_EQ(map.GetPadNum(XYPoint(-100, 5)), -1);
      EXPECT_EQ(map.GetPadNum(XYPoint(5, 1e9)), -1);
      EXPECT_EQ(map.GetPadNum(XYPoint(1e300, -1e300)), -1);
      EXPECT_EQ(map.GetPadNum(XYPoint(std::nan(""), 3)), -1);
   }
}

/**
 * The AT-TPC plane, in one test because ctest runs every test in its own process and the table takes several seconds
 * to build. First points over the whole plane, then the bottom 5 mm densely: there FindBin changes between a pad and
 * "no pad" across lines inside the TH2Poly that are not pad edges, and the table must not hide that. The bottom points
 * are ones the table build did not use (it checks each cell's center and corners pulled 1e-5 mm inside): uniform
 * points, and points 1e-7 mm inside every side of every cell.
 */
TEST(AtPadLookupTest, AtTpcMapMatchesFindBin)
{
   AtTpcMap map;
   map.GeneratePadPlane();
   std::unique_ptr<TH2Poly> plane(map.GetPadPlane());

   TRandom3 rand(2);
   std::vector<XYPoint> points;
   for (int i = 0; i < 300000; ++i) // Over the whole plane and beyond
      points.emplace_back(rand.Uniform(-300, 300), rand.Uniform(-300, 300));
   for (int c = 0; c < 200; ++c) { // Clouds like the diffusion of one line of charge
      double cx = rand.Uniform(-250, 250), cy = rand.Uniform(-250, 250), sigma = rand.Uniform(0.2, 3);
      for (int i = 0; i < 1000; ++i)
         points.emplace_back(cx + rand.Gaus(0, sigma), cy + rand.Gaus(0, sigma));
   }
   // Points on the pad corners, where the table must defer to FindBin
   for (auto *obj : *plane->GetBins()) {
      auto *graph = dynamic_cast<TGraph *>(static_cast<TH2PolyBin *>(obj)->GetPolygon());
      ASSERT_NE(graph, nullptr);
      if (rand.Uniform() > 0.05)
         continue;
      for (int i = 0; i < graph->GetN(); ++i)
         for (double d : {0., 1e-9, -1e-9, 1e-4})
            points.emplace_back(graph->GetX()[i] + d, graph->GetY()[i]);
   }
   {
      SCOPED_TRACE("whole plane");
      CheckPoints(map, *plane, points);
   }

   const double cell = AtPadLookup::kCell, inside = 1e-7, height = 5;
   double x0 = plane->GetXaxis()->GetXmin(), x1 = plane->GetXaxis()->GetXmax();
   double y0 = plane->GetYaxis()->GetXmin();
   int nx = std::ceil((x1 - x0) / cell);
   int ny = std::ceil(height / cell);
   points.clear();
   for (int i = 0; i < 500000; ++i)
      points.emplace_back(rand.Uniform(x0, x1), rand.Uniform(y0, y0 + height));
   for (int iy = 0; iy < ny; ++iy)
      for (int ix = 0; ix < nx; ++ix) {
         double xl = x0 + ix * cell, yl = y0 + iy * cell;
         points.emplace_back(xl + inside, yl + rand.Uniform() * cell);
         points.emplace_back(xl + cell - inside, yl + rand.Uniform() * cell);
         points.emplace_back(xl + rand.Uniform() * cell, yl + inside);
         points.emplace_back(xl + rand.Uniform() * cell, yl + cell - inside);
      }
   SCOPED_TRACE("bottom 5 mm");
   CheckPoints(map, *plane, points);
}

TEST(AtPadLookupTest, SwitchedOffStillMatchesFindBin)
{
   AtTpcMap map;
   map.SetUsePadLookup(false);
   map.GeneratePadPlane();
   std::unique_ptr<TH2Poly> plane(map.GetPadPlane());

   TRandom3 rand(3);
   std::vector<XYPoint> points;
   for (int i = 0; i < 20000; ++i)
      points.emplace_back(rand.Uniform(-300, 300), rand.Uniform(-300, 300));
   CheckPoints(map, *plane, points);
}

TEST(AtPadLookupTest, AtGadgetIIMapMatchesFindBin)
{
   AtGadgetIIMap map;
   map.GeneratePadPlane();
   std::unique_ptr<TH2Poly> plane(map.GetPadPlane());

   // The pads are 2.2 mm squares (and 2 mm veto pads) in four quadrants, spanning about +-36 mm in x and y
   TRandom3 rand(4);
   std::vector<XYPoint> points;
   for (int i = 0; i < 300000; ++i) // Over the whole plane and beyond
      points.emplace_back(rand.Uniform(-42, 42), rand.Uniform(-42, 42));

   // Clouds centered on pads, with widths from well inside one pad to a few pads
   std::vector<XYPoint> centers;
   for (auto *obj : *plane->GetBins()) {
      auto *bin = static_cast<TH2PolyBin *>(obj);
      centers.emplace_back((bin->GetXMin() + bin->GetXMax()) / 2, (bin->GetYMin() + bin->GetYMax()) / 2);
   }
   ASSERT_FALSE(centers.empty());
   for (int c = 0; c < 200; ++c) {
      auto &center = centers[rand.Integer(centers.size())];
      double sigma = rand.Uniform(0.05, 2);
      for (int i = 0; i < 1000; ++i)
         points.emplace_back(center.X() + rand.Gaus(0, sigma), center.Y() + rand.Gaus(0, sigma));
   }
   // Points on the pad corners, where the table must defer to FindBin
   for (auto *obj : *plane->GetBins()) {
      auto *graph = dynamic_cast<TGraph *>(static_cast<TH2PolyBin *>(obj)->GetPolygon());
      ASSERT_NE(graph, nullptr);
      if (rand.Uniform() > 0.25)
         continue;
      for (int i = 0; i < graph->GetN(); ++i)
         for (double d : {0., 1e-9, -1e-9, 1e-4})
            points.emplace_back(graph->GetX()[i] + d, graph->GetY()[i]);
   }
   CheckPoints(map, *plane, points);
}

TEST(AtPadLookupTest, ThreadsSeeTheSameAnswers)
{
   TestMap map;
   map.GeneratePadPlane();
   std::unique_ptr<TH2Poly> plane(map.GetPadPlane());

   TRandom3 rand(3);
   std::vector<XYPoint> points;
   for (int i = 0; i < 100000; ++i)
      points.emplace_back(rand.Uniform(-1, 14), rand.Uniform(-1, 14));
   std::vector<int> expected;
   for (auto &p : points)
      expected.push_back(Expected(*plane, map, p));

   // The threads race to build the table with their first lookups
   std::vector<int> mismatches(4, 0);
   std::vector<std::thread> threads;
   for (int t = 0; t < 4; ++t)
      threads.emplace_back([&, t] {
         for (int pass = 0; pass < 4; ++pass)
            for (size_t i = 0; i < points.size(); ++i)
               if (map.GetPadNum(points[i]) != expected[i])
                  ++mismatches[t];
      });
   for (auto &th : threads)
      th.join();
   for (int m : mismatches)
      EXPECT_EQ(m, 0);
}
