#ifndef ATTRACKINGANALYSIS_H
#define ATTRACKINGANALYSIS_H

#include "ATAnalysis.hh"
#include "ATRansac.hh"
#include "ATPatternEvent.hh"
#include "ATPatternEvent.hh"
#include "ATTrackingEventAna.hh"
#include "ATDigiPar.hh"
#include "ATMCQMinimization.hh"

#include "TF1.h"

#include <cstddef>
#include <iostream>


class ATTrackingAnalysis : public ATAnalysis{

      public:
	       ATTrackingAnalysis();
        ~ATTrackingAnalysis();

        void Analyze(ATRANSACN::ATRansac *Ransac,ATTrackingEventAna *trackingEventAna,TH2Poly* hPadPlane,const multiarray& PadCoord); // Analysis after RANSAC considering a reaction vertex
        void AnalyzeSimple(ATRANSACN::ATRansac *Ransac,ATTrackingEventAna *trackingEventAna,TH2Poly* hPadPlane,const multiarray& PadCoord); // Analysis
        void SetElossParameters(std::vector<Double_t> (&parE)[10]);
        void SetEtoRParameters(std::vector<Double_t> (&parRtoE)[10]);
        void AddParticle(std::vector<std::pair<Int_t,Int_t>>& ptcl);

        void Analyze(ATPatternEvent *patternEvent,ATTrackingEventAna *trackingEventAna,TH2Poly* hPadPlane,const multiarray& PadCoord); // Analysis after RANSAC considering a reaction vertex


      private:

          static Double_t GetEloss(Double_t c0,std::vector<Double_t>& par);
          static Double_t GetEnergyFromRange(Double_t range,std::vector<Double_t>& par);
          Double_t GetVertexTime(ATRANSACN::ATRansac *Ransac);
          Double_t GetTime(Double_t Z);
          std::pair<Double_t,Double_t> GetAnglesSolenoid(ATTrack* track);
          std::vector<ATHit> RotateEvent(ATTrack* track);
          std::vector<Double_t> fElossPar[10];
          std::vector<Double_t> fEtoRPar[10];
          std::vector<std::pair<Int_t,Int_t>> fParticleAZ;

          Double_t fVertex; // Physical vertex in mm;
          Double_t fEntTB_calc;//Calculated entrance Time Bucket
          Double_t fVertexEnergy; //Energy of beam from track length

          TGraph *fTrackFit;//!
          TF1 *fFitResult;//!
          TGraph *fTrackFitXY;//!
          TF1 *fFitResultXY;//!

          Int_t fMultiplicity;//Number of tracks to analyze




          // NB: This is just an example
          friend inline std::ostream& operator<<(std::ostream& os,const ATTrackingAnalysis& trackAna)
          {

              if(trackAna.fParticleAZ.size()>0){

                    for(Int_t i=0;i<trackAna.fParticleAZ.size();i++)
                        os<<" Particle "<<i<<" A : "<<trackAna.fParticleAZ.at(i).first<<"  Z : "<<trackAna.fParticleAZ.at(i).first<<std::endl;

                    return os;


              }else return os<<" No particles found !"<<std::endl;

              return os;

          }

          ClassDef(ATTrackingAnalysis, 1);


};

#endif
