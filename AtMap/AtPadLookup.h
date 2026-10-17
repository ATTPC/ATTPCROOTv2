#ifndef ATPADLOOKUP_H
#define ATPADLOOKUP_H

#include <Math/Point2D.h>

#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

class TH2Poly;

/**
 * Grid of pre-computed pad numbers that stands in for TH2Poly::FindBin (used by AtMap::GetPadNum).
 *
 * The bounding box of the pad plane is cut into square cells of kCell (0.1 mm). Each cell stores the pad number at
 * its center if that number is the same at the center and at the four corners (slightly pulled inside the cell) of
 * the cell and no pad edge crosses the cell. A cell where anything disagrees is "mixed" and stores a marker that
 * means "ask FindBin". So every lookup is either one table read or the same FindBin call as before, and for the
 * points tested (see AtPadLookupTest) the answer is the same as FindBin's.
 *
 * The table is built once, on the first lookup (about 5 s, single threaded, for the AT-TPC plane), and is never
 * modified afterwards. All threads share it read-only, so no locking is needed after the build. It takes 2 bytes per
 * cell, about 60 MB for the AT-TPC plane.
 *
 * The table is not built (every lookup uses FindBin) if a bin is not a plain polygon (TGraph) or the grid would be too
 * large. Cells whose pad number does not fit in an int16_t are stored as mixed, so those pads go through FindBin. The
 * table can also be turned off with AtMap::SetUsePadLookup(false).
 */
class AtPadLookup {
public:
   // 0.1 mm: 60 MB for the AT-TPC plane, 10% of the points in its disk ask FindBin. 0.2 mm would be 15 MB with 17%
   // asking FindBin, 0.05 mm 239 MB with 5%.
   static constexpr double kCell = 0.1; // mm

   /// Function that returns the pad number at a point like FindBin does: the pad number, or -1 if there is no pad.
   using FindBinFunc = std::function<int(const ROOT::Math::XYPoint &)>;

   /**
    * True if there is a table for this pad plane. The first call builds the table from the plane, which must be
    * complete by then (std::call_once, so it also works when several threads make their first lookup together).
    * `findBin` gives the pad number from FindBin. The plane is recognized by its address only, so a map that replaces
    * its plane with a different geometry after the first lookup needs a new AtPadLookup.
    */
   template <typename FindBin>
   bool HasTable(TH2Poly *plane, FindBin &&findBin)
   {
      std::call_once(fBuilt, [&] { Build(plane, findBin); });
      return fPlane == plane && !fCells.empty();
   }

   /// Pad number at a point: the table's value, or findBin(point) where the cell is mixed or outside the grid. Only
   /// call if HasTable() returned true.
   template <typename FindBin>
   int Find(const ROOT::Math::XYPoint &point, FindBin &&findBin) const
   {
      double fx = (point.X() - fX0) * kInvCell;
      double fy = (point.Y() - fY0) * kInvCell;
      if (!(fx >= 0 && fx < fNx && fy >= 0 && fy < fNy)) // Also catches NaN
         return findBin(point);
      int16_t value = fCells[static_cast<size_t>(static_cast<int>(fy)) * fNx + static_cast<int>(fx)];
      return value == kMixed ? findBin(point) : value;
   }

private:
   // A cell holds the pad number (>= 0), kNoPad, or kMixed
   static constexpr int16_t kNoPad = -1;
   static constexpr int16_t kMixed = -2;

   static constexpr double kInvCell = 1. / kCell;
   static constexpr double kMargin = 1e-6; // Distance an edge must be from a cell to not count as touching it
   static constexpr double kInset = 1e-5;  // How far inside the cell its corners are tested
   static constexpr size_t kMaxCells = 100'000'000;

   struct Polygon {
      std::vector<double> x, y;
      double xMin, xMax, yMin, yMax;
   };

   std::once_flag fBuilt;
   TH2Poly *fPlane{nullptr};

   // The grid, row by row from (fX0, fY0)
   double fX0{}, fY0{}, fXMax{}, fYMax{};
   int fNx{}, fNy{};
   std::vector<int16_t> fCells;

   // Polygons of the pads, and for each bucket of a coarser grid the polygons that can touch a cell in it
   std::vector<Polygon> fPolys;
   double fBucket{}, fInvBucket{};
   int fBucketsX{}, fBucketsY{};
   std::vector<int> fBucketStart; // Index into fBucketPolys, size buckets + 1
   std::vector<int> fBucketPolys;

   void Build(TH2Poly *plane, const FindBinFunc &findBin);
   bool CopyPolygons(TH2Poly *plane);
   void IndexPolygons();
   bool CellTouchesEdge(int ix, int iy) const;
   int16_t Classify(int ix, int iy, const FindBinFunc &findBin) const;
};

#endif
