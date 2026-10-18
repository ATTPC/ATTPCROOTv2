#include "AtPadLookup.h"

#include <TAxis.h>
#include <TGraph.h>
#include <TH2Poly.h>
#include <TList.h>

#include <algorithm>
#include <cmath>

// The table is built on first use, not at the end of GeneratePadPlane, because AtMap does not know when a subclass
// has finished its plane (AtTpcProtoMap replaces it later). Building in the first lookup also keeps maps that never
// look up a point free of the 60 MB and the ~5 s.
void AtPadLookup::Build(TH2Poly *plane, const FindBinFunc &findBin)
{
   fPlane = plane;
   if (!CopyPolygons(plane))
      return;

   // The grid covers the range FindBin accepts
   fX0 = plane->GetXaxis()->GetXmin();
   fXMax = plane->GetXaxis()->GetXmax();
   fY0 = plane->GetYaxis()->GetXmin();
   fYMax = plane->GetYaxis()->GetXmax();
   double nx = std::ceil((fXMax - fX0) / kCell), ny = std::ceil((fYMax - fY0) / kCell);
   if (!(nx > 0 && ny > 0 && nx * ny <= kMaxCells)) // Also catches NaN and infinity
      return;
   fNx = static_cast<int>(nx);
   fNy = static_cast<int>(ny);

   IndexPolygons();
   fCells.resize(static_cast<size_t>(fNx) * fNy);
   for (int iy = 0; iy < fNy; ++iy)
      for (int ix = 0; ix < fNx; ++ix)
         fCells[static_cast<size_t>(iy) * fNx + ix] = Classify(ix, iy, findBin);
}

/// Copy the polygons. Their vertices are what TH2PolyBin::IsInside (and so FindBin) uses. False if they can't be used.
bool AtPadLookup::CopyPolygons(TH2Poly *plane)
{
   if (plane == nullptr || plane->GetBins() == nullptr)
      return false;
   for (auto *obj : *plane->GetBins()) {
      auto *bin = dynamic_cast<TH2PolyBin *>(obj);
      auto *graph = bin ? dynamic_cast<TGraph *>(bin->GetPolygon()) : nullptr;
      if (graph == nullptr || graph->GetN() < 1)
         return false;
      Polygon poly;
      poly.x.assign(graph->GetX(), graph->GetX() + graph->GetN());
      poly.y.assign(graph->GetY(), graph->GetY() + graph->GetN());
      poly.xMin = *std::min_element(poly.x.begin(), poly.x.end());
      poly.xMax = *std::max_element(poly.x.begin(), poly.x.end());
      poly.yMin = *std::min_element(poly.y.begin(), poly.y.end());
      poly.yMax = *std::max_element(poly.y.begin(), poly.y.end());
      fPolys.push_back(std::move(poly));
   }
   return true;
}

/// Buckets: every polygon is listed in each bucket its bounding box, grown by one cell, overlaps. Then a cell inside a
/// bucket only needs the polygons of that bucket.
void AtPadLookup::IndexPolygons()
{
   fBucket = 20 * kCell;
   fInvBucket = 1. / fBucket;
   fBucketsX = static_cast<int>((fXMax - fX0) * fInvBucket) + 1;
   fBucketsY = static_cast<int>((fYMax - fY0) * fInvBucket) + 1;
   size_t numBuckets = static_cast<size_t>(fBucketsX) * fBucketsY;
   double grow = kCell + 4 * kMargin;

   auto bucketRange = [&](const Polygon &poly, int &bx0, int &bx1, int &by0, int &by1) {
      bx0 = std::clamp(static_cast<int>(std::floor((poly.xMin - grow - fX0) * fInvBucket)), 0, fBucketsX - 1);
      bx1 = std::clamp(static_cast<int>(std::floor((poly.xMax + grow - fX0) * fInvBucket)), 0, fBucketsX - 1);
      by0 = std::clamp(static_cast<int>(std::floor((poly.yMin - grow - fY0) * fInvBucket)), 0, fBucketsY - 1);
      by1 = std::clamp(static_cast<int>(std::floor((poly.yMax + grow - fY0) * fInvBucket)), 0, fBucketsY - 1);
   };

   std::vector<int> counts(numBuckets + 1, 0);
   for (auto &poly : fPolys) {
      int bx0, bx1, by0, by1;
      bucketRange(poly, bx0, bx1, by0, by1);
      for (int by = by0; by <= by1; ++by)
         for (int bx = bx0; bx <= bx1; ++bx)
            ++counts[static_cast<size_t>(by) * fBucketsX + bx + 1];
   }
   fBucketStart.assign(numBuckets + 1, 0);
   for (size_t i = 0; i < numBuckets; ++i)
      fBucketStart[i + 1] = fBucketStart[i] + counts[i + 1];
   fBucketPolys.assign(fBucketStart.back(), 0);
   std::vector<int> fill(fBucketStart.begin(), fBucketStart.end() - 1);
   for (size_t p = 0; p < fPolys.size(); ++p) {
      int bx0, bx1, by0, by1;
      bucketRange(fPolys[p], bx0, bx1, by0, by1);
      for (int by = by0; by <= by1; ++by)
         for (int bx = bx0; bx <= bx1; ++bx)
            fBucketPolys[fill[static_cast<size_t>(by) * fBucketsX + bx]++] = static_cast<int>(p);
   }
}

namespace {
// True if the segment p-q touches the rectangle
bool SegmentTouchesRect(double px, double py, double qx, double qy, double x0, double y0, double x1, double y1)
{
   if (std::max(px, qx) < x0 || std::min(px, qx) > x1 || std::max(py, qy) < y0 || std::min(py, qy) > y1)
      return false;
   // The rectangle and the segment's bounding box overlap. They only miss if the rectangle is entirely on one side of
   // the line through the segment.
   double dx = qx - px, dy = qy - py;
   double c0 = (x0 - px) * dy - (y0 - py) * dx;
   double c1 = (x1 - px) * dy - (y0 - py) * dx;
   double c2 = (x0 - px) * dy - (y1 - py) * dx;
   double c3 = (x1 - px) * dy - (y1 - py) * dx;
   if (c0 > 0 && c1 > 0 && c2 > 0 && c3 > 0)
      return false;
   if (c0 < 0 && c1 < 0 && c2 < 0 && c3 < 0)
      return false;
   return true;
}
} // namespace

bool AtPadLookup::CellTouchesEdge(int ix, int iy) const
{
   double x0 = fX0 + ix * kCell - kMargin, x1 = fX0 + (ix + 1) * kCell + kMargin;
   double y0 = fY0 + iy * kCell - kMargin, y1 = fY0 + (iy + 1) * kCell + kMargin;

   // FindBin answers differently outside of its range, so the cell must be completely inside
   if (x0 < fX0 || x1 > fXMax || y0 < fY0 || y1 > fYMax)
      return true;

   int bx = std::clamp(static_cast<int>((x0 + 0.5 * kCell - fX0) * fInvBucket), 0, fBucketsX - 1);
   int by = std::clamp(static_cast<int>((y0 + 0.5 * kCell - fY0) * fInvBucket), 0, fBucketsY - 1);
   size_t bucket = static_cast<size_t>(by) * fBucketsX + bx;
   for (int i = fBucketStart[bucket]; i < fBucketStart[bucket + 1]; ++i) {
      const Polygon &poly = fPolys[fBucketPolys[i]];
      if (poly.xMax < x0 || poly.xMin > x1 || poly.yMax < y0 || poly.yMin > y1)
         continue;
      size_t n = poly.x.size();
      for (size_t j = 0; j < n; ++j) {
         size_t k = (j + 1 == n) ? 0 : j + 1; // The polygon is closed
         if (SegmentTouchesRect(poly.x[j], poly.y[j], poly.x[k], poly.y[k], x0, y0, x1, y1))
            return true;
      }
   }
   return false;
}

int16_t AtPadLookup::Classify(int ix, int iy, const FindBinFunc &findBin) const
{
   if (CellTouchesEdge(ix, iy))
      return kMixed;

   // No pad edge crosses the cell, so the center is representative of the whole cell. The corners are checked too
   // because FindBin can also change across the internal partition lines of the TH2Poly, which are not pad edges.
   double xLow = fX0 + ix * kCell, yLow = fY0 + iy * kCell;
   int pad = findBin({xLow + 0.5 * kCell, yLow + 0.5 * kCell});
   for (double x : {xLow + kInset, xLow + kCell - kInset})
      for (double y : {yLow + kInset, yLow + kCell - kInset})
         if (findBin({x, y}) != pad)
            return kMixed;

   if (pad < kNoPad || pad > INT16_MAX)
      return kMixed;
   return static_cast<int16_t>(pad);
}
