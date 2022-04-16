/*******************************************************************
 * Base class for RANSAC                                            *
 * Log: Class started 02-10-2016                                    *
 * Author: Y. Ayyad (NSCL ayyadlim@nscl.msu.edu)                    *
 ********************************************************************/

#ifndef AtRANSAC_H
#define AtRANSAC_H

/*
#ifdef _OPENMP
#include <omp.h>
#endif
*/

#include "AtTrack.h" // for AtTrack

#include <Math/Vector3Dfwd.h> // for XYZVector
#include <Rtypes.h>           // for Double_t, Int_t, Float_t, THashConsist...
#include <TGraph2D.h>         // for TGraph2D
#include <TObject.h>          // for TObject
#include <TVector3.h>         // for TVector3

#include <assert.h> // for assert

#include <fstream> // for operator<<, endl, basic_ostream, basic...
#include <utility> // for pair
#include <vector>  // for vector
class AtEvent;
class AtHit;
class TBuffer;
class TClass;
class TMemberInspector;

namespace AtRANSACN {

class AtRansac : public TObject {
protected:
   TVector3 fVertex_1{-10000, -10000, -10000};
   TVector3 fVertex_2{-10000, -10000, -10000};
   TVector3 fVertex_mean;
   Double_t fMinimum{-1};
   Int_t fLineDistThreshold{3};
   int fRANSACModel{0};
   Float_t fRANSACThreshold{5};
   Double_t fXCenter{0};
   Double_t fYCenter{0};
   Float_t fRANSACPointThreshold{0.01};    // Number of points in percentage
   std::pair<Int_t, Int_t> fVertex_tracks; // ID of the tracks that form the best vertex
   Double_t fVertexTime{-1000};
   Double_t fTiltAng{0}; // From parameter file
   Int_t fMinHitsLine{}; // Minimum number of hits per line

   std::vector<AtTrack> fRansacTracks;
   std::vector<AtTrack> fTrackCand; // Candidate tracks

public:
   AtRansac();
   ~AtRansac();

   void CalcRANSAC(AtEvent *event);
   void CalcRANSACFull(AtEvent *event);
   std::vector<AtTrack> *RansacPCL(const std::vector<AtHit> &hits);
   TVector3 GetVertex1();
   TVector3 GetVertex2();
   Double_t GetVertexTime();
   Double_t GetAngleTracks(const ROOT::Math::XYZVector &vec1, const ROOT::Math::XYZVector &vec2);
   std::pair<Int_t, Int_t> GetPairTracksIndex();
   Int_t FindIndexTrack(Int_t index);
   Double_t GetMinimum(); // Distance of minumum approach between candidate lines (for the moment only 2)
   TVector3 GetVertexMean();
   Int_t MinimizeTrack(AtTrack *track);
   std::vector<AtTrack> &GetTrackCand();
   void SetModelType(int model);                 // RANSAC Model: Line, plane...
   void SetDistanceThreshold(Float_t threshold); // Distance to RANSAC line
   void SetMinHitsLine(Int_t nhits);
   void SetTiltAngle(Double_t val);
   void SetXYCenter(Double_t xc, Double_t yc);
   void SetRANSACPointThreshold(Float_t val);
   void SetVertexTime(Double_t val);
   Double_t Fit3D(AtTrack *track);

   struct PairedLines {
      std::pair<Int_t, Int_t> LinesID;
      std::pair<Double_t, Double_t> AngleZAxis;
      std::pair<Double_t, Double_t> AngleZDet;
      std::pair<Double_t, Double_t> AngleYDet;
      Double_t minDist;
      TVector3 meanVertex;
      Double_t angle;
   };

   std::vector<PairedLines> PLines;
   std::vector<PairedLines> GetPairedLinesArray();

   // AtDigiPar *fPar;
   // FairLogger *fLogger;

protected:
   static double distance2(double x, double y, double z, const double *p);
   void SetLine(double t, const double *p, double &x, double &y, double &z);
   void FindVertex(std::vector<AtTrack *> tracks);
   Bool_t CheckTrackID(Int_t trackID, std::vector<AtTrack> *trackArray); // Check if Track ID is in the list

   struct SumDistance2 {
      TGraph2D *fGraph;

      SumDistance2(TGraph2D *g) : fGraph(g) {}
      double operator()(const double *par)
      {
         assert(fGraph != 0);
         double *x = fGraph->GetX();
         double *y = fGraph->GetY();
         double *z = fGraph->GetZ();
         int npoints = fGraph->GetN();
         double sum = 0;
         for (int i = 0; i < npoints; ++i) {
            double d = distance2(x[i], y[i], z[i], par);
            sum += d;
         }
#ifdef DEBUG
         if (first)
            std::cout << "point " << i << "\t" << x[i] << "\t" << y[i] << "\t" << z[i] << "\t" << std::sqrt(d)
                      << std::endl;
#endif

         // if (first)
         // std::cout << "Total Initial distance square = " << sum << std::endl;
         // first = false;
         return sum;
      }
   };

   // template<typename T>
   friend inline std::ostream &operator<<(std::ostream &o, const AtRANSACN::AtRansac::PairedLines &pl)
   {

      o << " =============================================== " << std::endl;
      o << " Lines ID : " << pl.LinesID.first << " - " << pl.LinesID.second << std::endl;
      o << " Minimum distance between line : " << pl.minDist << std::endl;
      o << " Mean vertex between line - X : " << pl.meanVertex.X() << "  - Y : " << pl.meanVertex.Y()
        << "  - Z : " << pl.meanVertex.Z() << std::endl;
      o << " Angle with Z solenoid axis - Line 1 : " << pl.AngleZAxis.first << "  - Line 2 : " << pl.AngleZAxis.second
        << std::endl;
      o << " Angle with Z detector axis - Line 1 : " << pl.AngleZDet.first << "  - Line 2 : " << pl.AngleZDet.second
        << std::endl;
      o << " Angle with Y detector axis - Line 1 : " << pl.AngleYDet.first << "  - Line 2 : " << pl.AngleYDet.second
        << std::endl;
      o << " Angle between lines : " << pl.angle << std::endl;
      return o;
   }

   ClassDef(AtRansac, 1);
};

} // namespace AtRANSACN

#endif
