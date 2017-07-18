#ifndef  ATD2HEANALYSIS_H
#define ATD2HEANALYSIS_H


#include "TF1.h"
#include "TGraph.h"
#include "TGraph2D.h"
#include <Math/Vector3D.h>
#include <Math/Functor.h>
#include <Math/Minimizer.h>
#include "Fit/Fitter.h"
#include "TFitResult.h"
#include "TRandom3.h"

using namespace ROOT::Math;


class ATd2HeAnalysis {

      public:
	       ATd2HeAnalysis();
        ~ATd2HeAnalysis();

        

        Double_t omega(Double_t x, Double_t  y, Double_t z); 
        void kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t  thetalab, Double_t  K_eject);

        
        Double_t EnergyFluctuation(Double_t energypoint);

        void SetThetaCM(Double_t  value);
        void SetMissingMass(Double_t  value);
        double signo(double value ){
	        if(value<0) return -1.0;
                else return 1.0;	
                }

        Double_t GetThetaCM();
        Double_t GetMissingMass();
        Double_t GetThetaFit();
        Double_t GetPhiFit();
        bool GetFitStatus();
        Double_t GetVertexFit();

        void line(double t, const double *p, double &x, double &y, double &z) {
   // a parametric line is define from 6 parameters but 4 are independent
   // x0,y0,z0,z1,y1,z1 which are the coordinates of two points on the line
   // can choose z0 = 0 if line not parallel to x-y plane and z1 = 1;
                x = p[0] + p[1]*t;
                y = p[2] + p[3]*t;
                z = t;
        }

        

        struct SumDistance2 {
           // the TGraph is a data member of the object
                 TGraph2D *fGraph;
                SumDistance2(TGraph2D *g) : fGraph(g) {}
                // calculate distance line-point
                double distance2(double x,double y,double z, const double *p) {
                // distance line point is D= | (xp-x0) cross  ux |
                // where ux is direction of line and x0 is a point in the line (like t = 0)
                        XYZVector xp(x,y,z);
                        XYZVector x0(p[0], p[2], 0. );
                        XYZVector x1(p[0] + p[1], p[2] + p[3], 1. );
                        XYZVector u = (x1-x0).Unit();
                        double d2 = ((xp-x0).Cross(u)).Mag2();
                        return d2;
                }
                // implementation of the function to be minimized
                double operator() (const double *par) {
                assert(fGraph != 0);
                double * x = fGraph->GetX();
                double * y = fGraph->GetY();
                double * z = fGraph->GetZ();
                int npoints = fGraph->GetN();
                double sum = 0;
                for (int i  = 0; i < npoints; ++i) {
                               double d = distance2(x[i],y[i],z[i],par);
                               sum += d;
                }
                return sum;
                }
        };


        void FitTrack(TGraph2D *f2dtrack, TGraph *fpadtrack, std::vector<Double_t> *iniguess );

        
        Double_t THcm;
        Double_t MissingMass;
        Double_t theta_fit;
        Double_t phi_fit;
        bool ok;
        Double_t vertex;
        

      private:
    

          ClassDef(ATd2HeAnalysis, 1);


};

#endif

