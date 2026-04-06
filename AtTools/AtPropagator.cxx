#include "AtPropagator.h"

#include "AtKinematics.h"

#include <FairLogger.h>

// Butcher tableau coefficients for Dormand–Prince 5(4) method
// https://en.wikipedia.org/wiki/Dormand%E2%80%93Prince_method

static constexpr double c[7] = {0.0, 1.0 / 5.0, 3.0 / 10.0, 4.0 / 5.0, 8.0 / 9.0, 1.0, 1.0};
static constexpr double a[7][6] = {
   {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
   {1.0 / 5.0, 0.0, 0.0, 0.0, 0.0, 0.0},
   {3.0 / 40.0, 9.0 / 40.0, 0.0, 0.0, 0.0, 0.0},
   {44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0, 0.0, 0.0, 0.0},
   {19372.0 / 6561.0, -25360.0 / 2187.0, 64448.0 / 6561.0, -212.0 / 729.0, 0.0, 0.0},
   {9017.0 / 3168.0, -355.0 / 33.0, 46732.0 / 5247.0, 49.0 / 176.0, -5103.0 / 18656.0, 0.0},
   {35.0 / 384.0, 0.0, 500.0 / 1113.0, 125.0 / 192.0, -2187.0 / 6784.0, 11.0 / 84.0}};
// b (5th-order)
static constexpr double b[7] = {35.0 / 384.0, 0.0, 500.0 / 1113.0, 125.0 / 192.0, -2187.0 / 6784.0, 11.0 / 84.0, 0.0};
// b* (4th-order)
static constexpr double bs[7] = {5179.0 / 57600.0, 0.0,       7571.0 / 16695.0, 393.0 / 640.0, -92097.0 / 339200.0,
                                 187.0 / 2100.0,   1.0 / 40.0};

using ROOT::Math::Plane3D;
using ROOT::Math::XYZPoint;
using ROOT::Math::XYZVector;
namespace AtTools {

AtPropagator::XYZVector AtPropagator::Force(XYZPoint pos, XYZVector mom) const
{
   auto v = Kinematics::GetVel(mom, fState.fMass);

   auto F_lorentz = fState.fQ * (fEField + v.Cross(fBField));
   LOG(debug) << "F_lorentz: " << F_lorentz;

   auto dedx = fScalingFactor * fELossModel->GetdEdx(Kinematics::KE(mom, fState.fMass)); // Stopping power in MeV/mm
   auto dedx_si = dedx * 1.60218e-10;                                                    // de_dx in SI units (J/m)

   auto drag = -dedx_si * mom.Unit();
   LOG(debug) << "drag: " << drag << " mom " << mom << " dedx " << dedx_si;

   return F_lorentz + drag; // Force in N
}

AtPropagator::XYZVector AtPropagator::dpds(const XYZPoint &pos, const XYZVector &mom) const
{
   // Calculate the force acting on the particle at the given position and momentum
   auto speed = Kinematics::GetSpeed(mom.R(), fState.fMass); // Speed in m/s
   return Force(pos, mom) / speed;
}
AtPropagator::XYZVector AtPropagator::d2xds2(const XYZPoint &pos, const XYZVector &mom) const
{
   auto phat = mom.Unit();         // Unit vector in the direction of momentum
   auto p = mom.R();               // Magnitude of the momentum
   auto dpds_vec = dpds(pos, mom); // Derivative of momentum w.r.t. arc length

   return 1 / p * (dpds_vec - phat * (phat.Dot(dpds_vec))); // Second derivative of position w.r.t. arc length
}

void AtPropagator::PropagateOneStep(AtStepper &stepper)
{
   if (fState.h == 0)
      fState.h = stepper.GetInitialStep(); // Set the initial step size

   stepper.fDeriv = [this](const XYZPoint &pos, const XYZVector &mom) { return this->Derivatives(pos, mom); };

   auto result = stepper.Step(fState);
   if (!result) {
      LOG(error) << "Integration step failed, aborting propagation.";
      return; // Abort propagation if step failed
   }
   fState = result; // Update the internal state
}

void AtPropagator::PropagateToMeasurementSurface(const AtMeasurementSurface &surface, AtStepper &stepper)
{
   LOG(debug) << "Propagating to measurement surface from: " << fState.fPos;
   fState.h = stepper.GetInitialStep(); // Set the initial step size

   auto KE_initial = Kinematics::KE(fState.fMom, fState.fMass);
   if (KE_initial < fStopTol) {
      LOG(warning) << "Initial kinetic energy is below stopping threshold, cannot propagate.";
      return; // Cannot propagate if the initial kinetic energy is below the stopping threshold
   }
   stepper.fDeriv = [this](const XYZPoint &pos, const XYZVector &mom) { return this->Derivatives(pos, mom); };

   while (true) {
      LOG(debug) << "Position: " << GetPosition().X() / 10 << ", " << GetPosition().Y() / 10 << ", "
                 << GetPosition().Z() / 10;
      LOG(debug) << "Momentum: " << GetMomentum().X() << ", " << GetMomentum().Y() << ", " << GetMomentum().Z();

      auto result = stepper.Step(fState);
      if (!result) {
         LOG(error) << "Integration step failed, aborting propagation.";
         return; // Abort propagation if step failed
      }
      fState = result; // Update the internal state

      bool reachedMeasurementPoint = surface.PassedSurface(fState);
      bool particleStopped = Kinematics::KE(fState.fMom, fState.fMass) < fStopTol;
      bool momentumReversed = (fState.fLastMom.Dot(fState.fMom) < 0);

      if (reachedMeasurementPoint && !particleStopped && !momentumReversed) {
         // We reached the measurement surface, so we should figure out how far we are from the measurement point
         LOG(debug) << "------ Reached measurement surface ------";
         double finalH = (fState.fLastPos - fState.fPos).R(); // Distance traveled in the last step
         double approach = surface.Distance(fState.fLastPos);

         LOG(debug) << "Distance to plane: " << approach << " mm";
         LOG(debug) << "Final step size: " << finalH << " mm";
         LOG(debug) << "Position before surface: " << fState.fLastPos.X() << ", " << fState.fLastPos.Y() << ", "
                    << fState.fLastPos.Z();
         LOG(debug) << "Momentum before surface: " << fState.fLastMom.X() << ", " << fState.fLastMom.Y() << ", "
                    << fState.fLastMom.Z();

         finalH = approach * 1e-3;      // Convert to meters for the RK4 step
         fState.h = finalH;             // Set the step size to the distance to the surface
         fState.fPos = fState.fLastPos; // Set position to last position
         fState.fMom = fState.fLastMom; // Set momentum to last momentum
         result = stepper.Step(fState);
         if (!result) {
            LOG(error) << "Failed to propagate to measurement point, aborting.";
            return; // Abort propagation if step failed
         }
         auto origH = fState.h; // Save original step size
         fState = result;       // Update the internal state
         fState.h = origH;      // Restore original step size
      }

      if (particleStopped || momentumReversed) {
         // In this case the particle stopped before hitting the plane
         // we should throw a warning to let the user know that there wasn't
         // enough energy to reach the surface.
         LOG(warning) << "------ Particle stopped before reaching measurement surface ------";

         // Calculate how far to travel before stopping
         double KE_last = Kinematics::KE(fState.fLastMom, fState.fMass);
         double deltaE = KE_last - fStopTol;
         deltaE = std::max(deltaE, 0.0); // Ensure we don't have negative energy loss

         LOG(debug) << "Last KE: " << KE_last << " MeV";
         LOG(debug) << "Energy to loose to stop: " << deltaE << " MeV";
         double h_Stop = deltaE / fELossModel->GetdEdx(KE_last); // Distance to stop in mm
         LOG(debug) << "Estimated distance to stop: " << h_Stop << " mm";

         fState.h = h_Stop * 1e-3;      // Convert to meters for the RK4 step
         fState.fPos = fState.fLastPos; // Set position to last position
         fState.fMom = fState.fLastMom; // Set momentum to last momentum
         result = stepper.Step(fState);
         if (!result) {
            LOG(error) << "Failed to propagate to stopping point, aborting.";
            fState.status = AtPropagator::StepStateStatus::kStopped;
            return; // Abort propagation if step failed
         }
         auto origH = fState.h; // Save original step size
         fState = result;       // Update the internal state
         fState.h = origH;      // Restore original step size
         LOG(debug) << "Propagated to stopping point: " << fState.fPos.X() << ", " << fState.fPos.Y() << ", "
                    << fState.fPos.Z();
         LOG(debug) << "Energy after stopping: " << Kinematics::KE(fState.fMom, fState.fMass) << " MeV";

         while (surface.fClipToSurface) {
            fScalingFactor = 0; // Turn off energy loss.

            // If we still haven't intersected the surface, we need to adjust the step size
            double h = surface.Distance(fState.fPos); // Reduce step size so we hit the surface
            if (h <= fDistTol || surface.PassedSurface(result)) {
               reachedMeasurementPoint = true;
               break;
            }
            LOG(debug) << "Propagating to surface after stopping with step size: " << h << " mm";
            fState.h = h * 1e-3; // Convert to meters for the RK4 step
            result = stepper.Step(fState);
            if (!result) {
               LOG(error) << "Failed to propagate to surface after stopping, aborting.";
               return; // Abort propagation if step failed
            }
            fState = result; // Update the internal state
            LOG(debug) << "New position after adjusting step size: " << fState.fPos.X() << ", " << fState.fPos.Y()
                       << ", " << fState.fPos.Z();
         }
         fState.fLastMom = fState.fMom;
         fState.fMom = XYZVector(0, 0, 0); // Set momentum to zero since we stopped
      }

      if (reachedMeasurementPoint || particleStopped || momentumReversed) {
         double distanceToSurface = surface.Distance(fState.fPos);

         double KE_final = Kinematics::KE(fState.fMom, fState.fMass);
         LOG(debug) << "Initial KE: " << KE_initial << " MeV";
         LOG(debug) << "Final KE: " << KE_final << " MeV";
         auto calc_eLoss = KE_initial - KE_final; // Energy loss in MeV
         LOG(debug) << "Particle stopped: " << particleStopped;
         LOG(debug) << "Reached measurement point: " << reachedMeasurementPoint;
         LOG(debug) << "Distance to surface: " << distanceToSurface << " mm";
         LOG(debug) << "Calculated energy loss: " << calc_eLoss << " MeV";
         LOG(debug) << "Scaling factor: " << fScalingFactor;
         LOG(debug) << "Final Position: " << fState.fPos.X() << ", " << fState.fPos.Y() << ", " << fState.fPos.Z();
         LOG(debug) << "------- End of RK4 interation  ---------" << std::endl;

         // If we reached the measurement surface, we should project the position onto the surface
         if (reachedMeasurementPoint) {
            fState.fPos = surface.ProjectToSurface(fState.fPos);
         }
         if (particleStopped || momentumReversed)
            fState.status = AtPropagator::StepStateStatus::kStopped;
         LOG(debug) << "Projected on surface: " << fState.fPos.X() << ", " << fState.fPos.Y() << ", "
                    << fState.fPos.Z();
         LOG(debug) << "Final Momentum: " << fState.fMom.X() << ", " << fState.fMom.Y() << ", " << fState.fMom.Z();
         return;
      }
   } // End of loop over RK4 integration
}

void AtPropagator::PropagateToMeasurementSurface(const AtMeasurementSurface &surface, double eLoss, AtStepper &stepper)
{
   LOG(debug) << "Propagating to surface with eLoss: " << eLoss;

   if (eLoss == 0) {
      LOG(warn) << "No energy loss specified, propagating without energy loss adjustment.";
      PropagateToMeasurementSurface(surface, stepper);
      return;
   }

   int iterations = 0;
   double calc_eLoss = 0;
   double KE_initial = Kinematics::KE(fState.fMom, fState.fMass);
   auto initialMom = fState.fMom; // Save initial momentum for energy loss calculation
   auto initialPos = fState.fPos; // Save initial position for energy loss calculation

   while (std::abs(calc_eLoss - eLoss) > 1e-4) {
      fState.fMom = initialMom; // Reset position and momentum to initial values for the next iteration
      fState.fPos = initialPos;

      LOG(debug) << "Running iteration " << iterations << " with scaling factor: " << fScalingFactor
                 << " and energy loss: " << calc_eLoss;

      if (iterations > 100) {
         // If we are not converging, we should probably throw an error.
         throw std::runtime_error("Energy loss did not converge after 100 iterations.");
      }

      iterations++;
      PropagateToMeasurementSurface(surface, stepper); // Propagate without energy loss adjustment

      double KE_final = Kinematics::KE(fState.fMom, fState.fMass);
      calc_eLoss = KE_initial - KE_final; // Energy loss in MeV
      fScalingFactor *= eLoss / calc_eLoss;
      LOG(debug) << "Desired energy loss: " << eLoss << " MeV";
      LOG(debug) << "Calculated energy loss: " << calc_eLoss << " MeV";
      LOG(debug) << "Difference: " << calc_eLoss - eLoss << " MeV";
      LOG(debug) << "New scaling factor: " << fScalingFactor;
      LOG(debug) << "Condition: " << (std::abs(calc_eLoss - eLoss) > 1e-4);

   } // End loop over energy loss convergence

   LOG(debug) << "Energy loss converged after " << iterations << " iterations.";

   fScalingFactor = 1; // Reset scaling factor after convergence
}

AtPropagator::StepState AtRK4Stepper::Step(const AtPropagator::StepState &state) const
{

   auto result = state;
   result.fLastPos = state.fPos;
   result.fLastMom = state.fMom;
   result.hUsed = state.h;
   result.status = AtPropagator::StepStateStatus::kSuccess;

   auto h = state.h; // Step size in m
   auto fPos = state.fPos;
   auto fMom = state.fMom;

   LOG(debug) << "Starting RK4 step with initial position: " << fPos.X() << ", " << fPos.Y() << ", " << fPos.Z();
   LOG(debug) << "Initial momentum: " << fMom.X() << ", " << fMom.Y() << ", " << fMom.Z();
   LOG(debug) << "Step size (h): " << h << " m";

   auto [x_k1, p_k1] =
      fDeriv(fPos, fMom); // The derivative of the position is then just the unit vector of the momentum.

   auto x_2 = fPos + x_k1 * h / 2;               // Position at the midpoint
   auto p_2 = fMom + p_k1 * h / 2 / fReltoSImom; // Momentum at the midpoint

   auto [x_k2, p_k2] = fDeriv(x_2, p_2); // Derivative at the midpoint

   auto x_3 = fPos + x_k2 * h / 2;               // Position at the second midpoint
   auto p_3 = fMom + p_k2 * h / 2 / fReltoSImom; // Momentum at the second midpoint
   auto [x_k3, p_k3] = fDeriv(x_3, p_3);

   auto x_4 = fPos + x_k3 * h;               // Position at the end of the step
   auto p_4 = fMom + p_k3 * h / fReltoSImom; // Momentum at the end of the step
   auto [x_k4, p_k4] = fDeriv(x_4, p_4);

   auto dpds_SI = (p_k1 + 2 * p_k2 + 2 * p_k3 + p_k4) / 6; // "Force" in SI units (N)
   auto dxds_SI = (x_k1 + 2 * x_k2 + 2 * x_k3 + x_k4) / 6; // Position derivative in SI units (m)

   LOG(debug) << "dp/ds (SI units): " << dpds_SI.X() << ", " << dpds_SI.Y() << ", " << dpds_SI.Z();
   LOG(debug) << "dx/ds (SI units): " << dxds_SI.X() << ", " << dxds_SI.Y() << ", " << dxds_SI.Z();

   auto mom_SI = fReltoSImom * fMom;
   mom_SI += dpds_SI * h;              // Update momentum in SI units (kg m/s)
   result.fMom = mom_SI / fReltoSImom; // Convert back to

   auto pos_SI = fPos * 1e-3;  // Convert position to SI units (m)
   pos_SI += dxds_SI * h;      // Update position in SI units (m
   result.fPos = pos_SI * 1e3; // Convert back to mm

   return result;
}

AtPropagator::StepState AtRK4AdaptiveStepper::Step(const AtPropagator::StepState &state) const
{

   // Take h to be the step size in m.
   auto result = state;
   result.fLastPos = state.fPos;
   result.fLastMom = state.fMom;
   result.status = AtPropagator::StepStateStatus::kSuccess;

   auto h = state.h; // Step size in m
   auto fPos = state.fPos;
   auto fMom = state.fMom;

   // Take h to be the step size in m.
   // Use DP5(4) method for adaptive step size control.

   auto x0_mm = fPos;
   auto p0 = fMom;
   LOG(debug) << "Starting RK4 step with initial position: " << x0_mm.X() << ", " << x0_mm.Y() << ", " << x0_mm.Z();
   LOG(debug) << "Initial momentum: " << p0.X() << ", " << p0.Y() << ", " << p0.Z();

   while (true) {
      auto x_SI = fPos * 1e-3;        // Convert position to SI units (m)
      auto p_SI = fReltoSImom * fMom; // Convert momentum to SI units (kg m/s)
      XYZVector kx[7];                // kx[i] will hold the position derivatives (unitless)
      XYZVector kp[7];                // kp[i] will hold the momentum derivatives (SI units)

      // anonymous lambda to calculate and store the kx and kp values. Input is SI units.
      auto calc_k = [&](const XYZPoint &x, const XYZVector &p, int i) {
         auto [k_x, k_p] = fDeriv(x * 1e-3, p / fReltoSImom);
         kx[i] = k_x; // Store the position derivative (unitless)
         kp[i] = k_p; // Store the momentum derivative (SI units)
      };

      // anonymous lambda to calculate the position and momentum at the i-th stage
      auto calc_xp = [&](int i) {
         XYZVector dx(0, 0, 0);
         XYZVector dp(0, 0, 0);
         for (int j = 0; j < i; ++j) {
            dx = dx + kx[j] * a[i][j];
            dp = dp + kp[j] * a[i][j];
         }
         XYZPoint x = x_SI + dx * h;
         XYZVector p = p_SI + dp * h;
         return std::make_pair(x, p);
      };

      // Calculate kx and kp for each stage
      // build stage 0
      calc_k(x_SI, p0, 0);

      // build stage 1
      auto [x1, p1] = calc_xp(1);
      calc_k(x1, p1, 1); // k1

      // build stage 2
      auto [x2, p2] = calc_xp(2);
      calc_k(x2, p2, 2); // k2

      // build stage 3
      auto [x3, p3] = calc_xp(3);
      calc_k(x3, p3, 3); // k3

      // build stage 4
      auto [x4, p4] = calc_xp(4);
      calc_k(x4, p4, 4); // k4

      // build stage 5
      auto [x5, p5] = calc_xp(5);
      calc_k(x5, p5, 5); // k5

      // build stage 6
      auto [x6, p6] = calc_xp(6);
      calc_k(x6, p6, 6); // k6

      // Calculate the new position and momentum using the 5th-order method
      XYZVector dx(0, 0, 0);
      XYZVector dp(0, 0, 0);
      for (int i = 0; i < 7; ++i) {
         dx = dx + kx[i] * b[i];
         dp = dp + kp[i] * b[i];
      }
      XYZPoint x_new_5 = x_SI + dx * h;  // New position in SI units (m)
      XYZVector p_new_5 = p_SI + dp * h; // New momentum in SI units (kg m/s)

      // Calculate the new position and momentum using the 4th-order method
      dx = XYZVector(0, 0, 0);
      dp = XYZVector(0, 0, 0);
      for (int i = 0; i < 7; ++i) {
         dx = dx + kx[i] * bs[i];
         dp = dp + kp[i] * bs[i];
      }
      XYZPoint x_new_4 = x_SI + dx * h;  // New position in SI units (m)
      XYZVector p_new_4 = p_SI + dp * h; // New momentum in SI units (kg m/s)

      auto x_4_mm = x_new_4 * 1e3;          // Convert back to mm
      auto p_4_MeV = p_new_4 / fReltoSImom; // Convert back to MeV/c
      auto x_5_mm = x_new_5 * 1e3;          // Convert back to mm
      auto p_5_MeV = p_new_5 / fReltoSImom; // Convert back to MeV/c
      LOG(debug) << "New position (5th order): " << x_5_mm.X() << ", " << x_5_mm.Y() << ", " << x_5_mm.Z();
      LOG(debug) << "New momentum (5th order): " << p_5_MeV.X() << ", " << p_5_MeV.Y() << ", " << p_5_MeV.Z();
      LOG(debug) << "New position (4th order): " << x_4_mm.X() << ", " << x_4_mm.Y() << ", " << x_4_mm.Z();
      LOG(debug) << "New momentum (4th order): " << p_4_MeV.X() << ", " << p_4_MeV.Y() << ", " << p_4_MeV.Z();

      // Convert back to mm and MeV/c
      XYZVector x_err = (x_5_mm - x_4_mm);   // Error in position (mm)
      XYZVector p_err = (p_5_MeV - p_4_MeV); // Error in momentum (MeV/c)

      // Calculate the overall error
      double ex = x_err.X() / (fAtolPos + fRtol * std::abs(x_5_mm.X()));
      double ey = x_err.Y() / (fAtolPos + fRtol * std::abs(x_5_mm.Y()));
      double ez = x_err.Z() / (fAtolPos + fRtol * std::abs(x_5_mm.Z()));

      double ep_x = p_err.X() / (fAtolMom + fRtol * std::abs(p_5_MeV.X()));
      double ep_y = p_err.Y() / (fAtolMom + fRtol * std::abs(p_5_MeV.Y()));
      double ep_z = p_err.Z() / (fAtolMom + fRtol * std::abs(p_5_MeV.Z()));

      // Combine errors (norm)
      double err = std::sqrt(ex * ex + ey * ey + ez * ez + ep_x * ep_x + ep_y * ep_y + ep_z * ep_z);

      double factor = std::pow(err, -1.0 / 5.0); // Adjust step size based on error
      factor = std::clamp(factor, 0.25, 4.0);    // Clamp factor to reasonable limits
      double hNew = h * factor;
      // We now know the local error at this point. Now we need to decide to accept the point or not.
      if (err <= 1.0) {
         // Accept the step
         result.fPos = x_5_mm;  // Update position in mm
         result.fMom = p_5_MeV; // Update momentum in MeV/c
         LOG(debug) << "Accepted step with error: " << err;
         LOG(debug) << "Step size: " << h << " m";
         LOG(debug) << "New step size: " << hNew << " m";
         LOG(debug) << "New Position: " << result.fPos.X() << ", " << result.fPos.Y() << ", " << result.fPos.Z();
         LOG(debug) << "New Momentum: " << result.fMom.X() << ", " << result.fMom.Y() << ", " << result.fMom.Z();

         result.h = hNew;                                         // Adjust the step size for the next iteration
         result.hUsed = h;                                        // Store the step size used
         result.status = AtPropagator::StepStateStatus::kSuccess; // Step accepted
         return result;
      } else {
         // Reject the step and reduce the step size
         LOG(debug) << "Rejected step with error: " << err;
         LOG(debug) << "Step size: " << h << " m";
         LOG(debug) << "Reducing step size to: " << hNew << " m";

         result.h = hNew; // Reduce step size for next iteration
         h = hNew;        // Update h for the next iteration
         if (result.h < fMinStep || result.h > fMaxStep) {
            LOG(error) << "Step size out of bounds, aborting propagation.";
            result.status = AtPropagator::StepStateStatus::kInvalidStepSize;
            result.hUsed = h;
            return result; // Abort propagation if step size is out of bounds
         }
      }
   }
}

bool AtMeasurementPoint::PassedSurface(AtPropagator::StepState &result) const
{
   // Check if the particle has passed the measurement point
   auto lastDeriv = (fPoint - result.fLastPos).Dot(result.fLastMom.Unit());
   auto currDeriv = (fPoint - result.fPos).Dot(result.fMom.Unit());
   LOG(debug) << "Last Derivative: " << lastDeriv << ", Current Derivative: " << currDeriv;
   return lastDeriv * currDeriv <= 0;
}

bool AtMeasurementPlane::PassedSurface(AtPropagator::StepState &result) const
{
   // Check if the particle has crossed the plane this step.
   auto prevSign = fPlane.Distance(result.fLastPos) > 0 ? 1 : -1;
   auto currSign = fPlane.Distance(result.fPos) > 0 ? 1 : -1;
   return (prevSign != currSign);
}

ROOT::Math::XYZPoint AtMeasurementPlane::ProjectToSurface(const ROOT::Math::XYZPoint &pos) const
{
   // Project the position onto the measurement plane
   auto dist = fPlane.Distance(pos);
   return pos - dist * fPlane.Normal();
}
} // namespace AtTools
