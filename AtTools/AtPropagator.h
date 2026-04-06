#ifndef ATPROPAGATOR_H
#define ATPROPAGATOR_H

#include "AtELossModel.h"

#include <functional>

#include "Math/Plane3D.h"
#include "Math/Point3D.h"
#include "Math/Vector3D.h"

namespace AtTools {

class AtMeasurementSurface;
class AtStepper;

/**
 * @brief Class for propagating particles through a medium.
 *
 * This class is responsible for simulating the propagation of particles
 * through a medium, taking into account energy loss and other effects.
 * Uses an AtELossModel to calculate the energy loss and propagates particles
 * in the presence of electric and magnetic fields.
 *
 * Class is designed to be used with a single particle type. Create a new instance of the
 * class if the material or particle type changes.
 */
class AtPropagator {
public:
   enum class StepStateStatus {
      kSuccess,         /// Step was successful
      kInvalidStepSize, /// Step failed
      kStopped          /// Particle is stopped in material
   };
   struct StepState {
      ROOT::Math::XYZPoint fPos;                          /// Position of the particle in mm
      ROOT::Math::XYZVector fMom;                         /// Momentum of the particle in MeV/c
      ROOT::Math::XYZPoint fLastPos;                      /// Last position of the particle in mm
      ROOT::Math::XYZVector fLastMom;                     /// Last momentum of the particle in MeV/c
      double fMass = 0;                                   /// Mass of the particle in MeV/c^2
      double fQ = 0;                                      /// Charge of the particle in Coulombs
      double h = 0;                                       /// Step size to use in m
      double hUsed = 0;                                   /// Step size used in this step in m
      StepStateStatus status = StepStateStatus::kSuccess; /// Whether the step was successful

      operator bool() const { return status == StepStateStatus::kSuccess; }
   };

protected:
   using XYZVector = ROOT::Math::XYZVector;
   using XYZPoint = ROOT::Math::XYZPoint;
   using Plane3D = ROOT::Math::Plane3D;

   // Variables used for the force
   XYZVector fEField{0, 0, 0};                // Electric field vector
   XYZVector fBField{0, 0, 0};                // Magnetic field vector
   std::unique_ptr<AtELossModel> fELossModel; // Energy loss model

   // Internal state variables for the propagator
   StepState fState; /// Current state of the particle

   // Tolerances and limits
   double fETol = 1e-4;    /// Energy tolerance for convergence when fixing energy loss
   double fStopTol = 0.01; /// Maximum kinetic energy to consider the particle stopped (MeV)
   double fDistTol = 1e-2; /// Distance tolerance when considering positions equal. (mm)

   static constexpr double fReltoSImom = 1.60218e-13 / 299792458; // Conversion factor from MeV/c to kg m/s (SI units)

public:
   double fScalingFactor = 1.0; /// Scaling factor for energy loss

   /**
    * @brief Constructor for AtPropagator.
    * @param charge Charge of the particle in Coulombs.
    * @param mass Mass of the particle in MeV/c^2.
    * @param elossModel Energy loss model to use for the particle.
    */
   AtPropagator(double charge, double mass, std::unique_ptr<AtELossModel> elossModel)
      : fELossModel(std::move(elossModel))
   {
      fState.fMass = mass;
      fState.fQ = charge;
   }
   AtPropagator(AtPropagator &&) = default;
   /**
    * @brief Set the electric field (V/m)
    */
   void SetEField(const XYZVector &eField) { fEField = eField; }
   void SetEField(double ex, double ey, double ez) { fEField.SetXYZ(ex, ey, ez); }

   /**
    * @brief Set the magnetic field (T)
    */
   void SetBField(const XYZVector &bField) { fBField = bField; }
   void SetBField(double bx, double by, double bz) { fBField.SetXYZ(bx, by, bz); }

   /**
    * @brief Set the state of the particle.
    *
    * @param pos Position of the particle in mm.
    * @param mom Momentum of the particle in MeV/c.
    */
   void SetState(const XYZPoint &pos, const XYZVector &mom)
   {
      fState.fPos = pos;
      fState.fMom = mom;
   }
   const StepState &GetState() const { return fState; }
   const AtELossModel *GetELossModel() const { return fELossModel.get(); }

   XYZPoint GetPosition() const { return fState.fPos; }
   XYZVector GetMomentum() const { return fState.fMom; }

   /**
    * @brief Propagate the particle to the point of closest approach to the given point.
    *
    * Propagate to a given point in space, adjusting the magnitude of the stopping power
    * to ensure that a specific about of energy is lost during the propagation.
    *
    * @param point The point to approach.
    * @param eLoss If not 0, constrain the energy loss to this value by adjusting fScalingFactor.
    */
   void PropagateToMeasurementSurface(const AtMeasurementSurface &point, double eLoss, AtStepper &stepper);

   void PropagateToMeasurementSurface(const AtMeasurementSurface &surface, AtStepper &stepper);

   /**
    * @brief Propagate the particle using the given stepper.
    * Propagates one step using the provided stepper.
    *
    * @param stepper The stepper to use for propagation.
    */
   void PropagateOneStep(AtStepper &stepper);

   /**
    * @brief Calculate the force acting on the particle.
    *
    * @param pos Position of the particle in mm.
    * @param mom Momentum of the particle in MeV/c.
    * @return The force acting on the particle in N.
    */
   XYZVector Force(XYZPoint pos, XYZVector mom) const;

   /**
    * @brief Calculate the derivate of the momentum w.r.t. arc length.
    *
    * @param pos Position of the particle in mm.
    * @param mom Momentum of the particle in MeV/c.
    * @return The derivative of the momentum w.r.t. arc length in N/m.
    */
   XYZVector dpds(const XYZPoint &pos, const XYZVector &mom) const;

   XYZVector dxds(const XYZPoint &pos, const XYZVector &mom) const
   {
      return mom.Unit(); // The derivative of the position is just the unit vector of the momentum.
   }

   std::pair<ROOT::Math::XYZVector, ROOT::Math::XYZVector>
   Derivatives(const ROOT::Math::XYZPoint &pos, const ROOT::Math::XYZVector &mom) const
   {
      return {dxds(pos, mom), dpds(pos, mom)};
   }

protected:
   /**
    * @brief Calculate the second derivative of the position w.r.t. arc length.
    *
    * \frac{d^2\vec{x}}{ds^2} = \frac{1}{p} \left( \frac{d\vec{p}}{ds} - \hat{p} (\hat{p} \cdot \frac{d\vec{p}}{ds})
    * \right)
    *
    * @param pos Position of the particle in mm.
    * @param mom Momentum of the particle in MeV/c.
    * @return The second derivative of the position w.r.t. arc length in m/m^2.
    */
   XYZVector d2xds2(const XYZPoint &pos, const XYZVector &mom) const;
};
class AtStepper {
public:
   /**
    * @brief Function type defining the derivative of the position and momentum w.r.t. distance.
    *
    * This function takes the current position and momentum and returns the derivate of the position and momentum.
    *
    * @param pos Current position of the particle in mm.
    * @param mom Current momentum of the particle in MeV/c.
    * @return A pair containing the derivatives of the position and momentum in SI units (m and kg m/s).
    * The first element is the derivative of the position, and the second element is the derivative
    * of the momentum.
    */
   using DerivFunc = std::function<std::pair<ROOT::Math::XYZVector, ROOT::Math::XYZVector>(
      const ROOT::Math::XYZPoint &, const ROOT::Math::XYZVector &)>;

   DerivFunc fDeriv;

   virtual AtPropagator::StepState Step(const AtPropagator::StepState &state) const = 0;
   virtual double GetInitialStep() const { return 1e-4; } /// Default initial step size in m

   virtual ~AtStepper() = default;

protected:
   static constexpr double fReltoSImom = 1.60218e-13 / 299792458; // Conversion factor from MeV/c to kg m/s (SI units)
};

class AtRK4Stepper : public AtStepper {
   double fStepSize = 1e-4;

public:
   AtPropagator::StepState Step(const AtPropagator::StepState &state) const override;
   double GetInitialStep() const override { return fStepSize; } /// Default initial step size in m
};
class AtRK4AdaptiveStepper : public AtStepper {
public:
   double fAtolPos = 1e-2;     /// Absolute tolerance for position in mm
   double fAtolMom = 1e-2;     /// Absolute tolerance for momentum in MeV/c
   double fRtol = 1e-6;        /// Relative tolerance for position and momentum
   double fMinStep = 1e-6;     /// Minimum step size in m
   double fMaxStep = 10.0;     /// Maximum step size in m
   double fInitialStep = 1e-4; /// Initial step size in m

   AtPropagator::StepState Step(const AtPropagator::StepState &state) const override;
   double GetInitialStep() const override { return fInitialStep; } /// Default initial step size in m
};

/**
 * @brief Class for measurement surface in the AT-TPC.
 *
 * This class represents a measurement surface or point in the AT-TPC. It's used to define the stopping
 * point and behavior of the propagator.
 */
class AtMeasurementSurface {
public:
   bool fClipToSurface = false; // Whether to clip to the surface

   /**
    * @brief Calculate the distance from the position to the surface.
    */
   virtual double Distance(const ROOT::Math::XYZPoint &pos) const = 0;

   /**
    * @brief Check if we have passed the surface between the last position and the current position.
    */
   virtual bool PassedSurface(AtPropagator::StepState &result) const = 0;

   virtual ROOT::Math::XYZPoint ProjectToSurface(const ROOT::Math::XYZPoint &pos) const = 0;
};
class AtMeasurementPoint : public AtMeasurementSurface {
protected:
   ROOT::Math::XYZPoint fPoint; // The measurement point in mm

public:
   AtMeasurementPoint(const ROOT::Math::XYZPoint &point) : fPoint(point) {}

   template <typename T>
   AtMeasurementPoint(const T &point)
   {
      fPoint = ROOT::Math::XYZPoint(point[0], point[1], point[2]);
   }

   double Distance(const ROOT::Math::XYZPoint &pos) const override { return (fPoint - pos).R(); }
   bool PassedSurface(AtPropagator::StepState &result) const override;
   ROOT::Math::XYZPoint ProjectToSurface(const ROOT::Math::XYZPoint &pos) const override { return pos; }
};

class AtMeasurementPlane : public AtMeasurementSurface {
protected:
   ROOT::Math::Plane3D fPlane; // The measurement plane
public:
   AtMeasurementPlane(const ROOT::Math::Plane3D &plane) : fPlane(plane) { fClipToSurface = true; }

   double Distance(const ROOT::Math::XYZPoint &pos) const override { return std::abs(fPlane.Distance(pos)); }
   bool PassedSurface(AtPropagator::StepState &result) const override;
   ROOT::Math::XYZPoint ProjectToSurface(const ROOT::Math::XYZPoint &pos) const override;
};
} // namespace AtTools
#endif // #ifndef ATPROPAGATOR_H
