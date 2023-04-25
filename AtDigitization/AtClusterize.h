#ifndef ATCLUSTERIZE_H
#define ATCLUSTERIZE_H

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h>

#include <cmath>   // for double_t
#include <cstdint> // for uint64_t
#include <memory>  // for unique_ptr
#include <string>
#include <vector>
class AtDigiPar;
class AtMCPoint;
class AtSimulatedPoint;
class TClonesArray;

/**
 * Class to hold the clusterizing logic
 *
 * Input is an array of AtMCPoints, output is an array of AtSimulatedPoints.
 */
class AtClusterize {
protected:
   using XYZVector = ROOT::Math::XYZVector;
   using XYZPoint = ROOT::Math::XYZPoint;
   using SimPointPtr = std::unique_ptr<AtSimulatedPoint>;

   double fEIonize{};     //!< Effective ionization energy of gas. [eV]
   double fFano{};        //!< Fano factor of the gas
   double fVelDrift{};    //!< Drift velocity of electron in gas. [cm/us]
   double fCoefT{};       //!< Transversal diffusion coefficient. [cm^2/us]
   double fCoefL{};       //!< Longitudinal diffusion coefficient. [cm^2/us]
   double fDetPadPlane{}; //!< Position of the pad plane with respect to the entrance [mm]

   static thread_local XYZPoint fPrevPoint; //!< The previous point we recorded charge.
   static thread_local int fTrackID;        //!< The current track ID

public:
   std::vector<SimPointPtr> ProcessEvent(const TClonesArray &fMCPointArray);
   virtual void GetParameters(const AtDigiPar *fPar);
   virtual std::string GetSavedClassName() const { return "AtSimulatedPoint"; }
   virtual void FillTClonesArray(TClonesArray &array, std::vector<SimPointPtr> &vec);
   virtual std::shared_ptr<AtClusterize> Clone() const { return std::make_shared<AtClusterize>(*this); }

private:
   XYZPoint applyDiffusion(const XYZPoint &loc, double_t sigTrans, double sigLong);

protected:
   virtual std::vector<SimPointPtr> processPoint(AtMCPoint &mcPoint, int pointID = -1);

   void setNewTrack();
   double getTransverseDiffusion(double driftTime);   // in mm
   double getLongitudinalDiffusion(double driftTime); // in us
   uint64_t getNumberOfElectronsGenerated(const AtMCPoint &mcPoint);
   XYZPoint getCurrentPointLocation(const AtMCPoint &mcPoint);
};

#endif //#ifndef ATCLUSTERIZE_H
