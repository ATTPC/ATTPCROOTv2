///
/// Copyright 2022 Mohanad Youssef (Al-khwarizmi)
///
/// Use of this source code is governed by an GPL-3.0 - style
/// license that can be found in the LICENSE file or at
/// https://opensource.org/licenses/GPL-3.0
///
/// @author Mohanad Youssef <mohanad.magdy.hammad@gmail.com>
/// @file unscented_kalman_filter.h
///

#ifndef TRACKFITTERUKF_H
#define TRACKFITTERUKF_H

#include "AtKinematics.h"
#include "AtPropagator.h"

#include <FairLogger.h>

#include <Math/Plane3D.h>
#include <Math/Vector3D.h>
#include <TMatrixDfwd.h>

#include "kalman_filter.h"
#include "kf_util.h"
#include "unscented_kalman_filter.h"
namespace kf {

/**  @brief Class for fitting tracks using the Unscented Kalman Filter (UKF) algorithm.
 *
 * Serves as a templated base class for UKF calculations. It hold no physics information, just
 * the machinery that underlies the UKF formalism. It is a modified version of the UKF provided
 * by OpenKF, that has been expanded to allow for more hooks into the method.
 *
 * Introduces two different types of process noise. We have augmented process noise and
 * model process noise. Augmented process noise acts during the propagator, model process
 * noise can be used to describe noise in the propagation itself and allows for accounting
 * for uncertainty that may arise from unmodeled dynamics.
 *
 * Templated because I believe Eigen can do quite a bit of operation for small matrices like we have here if
 * the size is known at compile time. Worth checking that though.
 *
 * @tparam DIM_X Dimension of the state vector.
 * @tparam DIM_Z Dimension of the measurement vector.
 * @tparam DIM_V Dimension of the process noise vector.
 * @tparam DIM_N Dimension of the measurement noise vector.
 */
template <int32_t DIM_X = 6, int32_t DIM_Z = 3, int32_t DIM_V = 2, int32_t DIM_N = 3>
class TrackFitterUKFBase {
public:
   // Augmented state vector is just the process noise and state vector. The measurement noise is not included as that
   // is independent of the propagation and measurement model and just adds linearly.
   static constexpr int32_t DIM_A{DIM_X + DIM_V};       ///< @brief Augmented state dimension
   static constexpr int32_t SIGMA_DIM_A{2 * DIM_A + 1}; ///< @brief Sigma points dimension for augmented state
   Matrix<DIM_A, SIGMA_DIM_A> m_matSigmaXa{Matrix<DIM_A, SIGMA_DIM_A>::Zero()}; ///< @brief Sigma points matrix

   // Controls and variables for running numerical diagnostics.
   int nTouch{0}; // Variable to track the number of times a matrix has a floor added.
   bool kLogEigen{false};
   bool kReplaceFirstCov{true}; /// If true, replace the seed COV with the predicted cov of the second point

protected:
   using VectorEigenVecDimX = std::vector<Vector<DIM_X>, Eigen::aligned_allocator<Vector<DIM_X>>>;
   using VectorEigenVecDimZ = std::vector<Vector<DIM_Z>, Eigen::aligned_allocator<Vector<DIM_Z>>>;
   using VectorEigenMatDimX = std::vector<Matrix<DIM_X, DIM_X>, Eigen::aligned_allocator<Matrix<DIM_X, DIM_X>>>;

   Vector<DIM_X> m_vecX{Vector<DIM_X>::Zero()};               /// @brief estimated state vector
   Matrix<DIM_X, DIM_X> m_matP{Matrix<DIM_X, DIM_X>::Zero()}; /// @brief state covariance matrix

   /// @brief Augmented state vector (incl. process and measurement noise means)
   Vector<DIM_A> m_vecXa{Vector<DIM_A>::Zero()};
   /// @brief augmented state covariance (incl. process and measurement noise covariances)
   Matrix<DIM_A, DIM_A> m_matPa{Matrix<DIM_A, DIM_A>::Zero()};

   /// Process noise covariance matrix incorporated in propagator (Q_aug)
   Matrix<DIM_V, DIM_V> m_matQaug{Matrix<DIM_V, DIM_V>::Zero()};
   /// Process noise covariance matrix for model noise (Q_mod)
   Matrix<DIM_X, DIM_X> m_matQmod{Matrix<DIM_X, DIM_X>::Zero()};
   /// Measurement noise covariance matrix (R)
   Matrix<DIM_N, DIM_N> m_matR{Matrix<DIM_N, DIM_N>::Zero()};

   /// Unscented transform weight for the mean sigma point in mean
   float32_t m_weightM0;
   /// Unscented transform weight for the mean sigma point in covariance
   float32_t m_weightC0;
   /// Unscented transform weight for the other sigma points
   float32_t m_weighti;
   /// Lambda parameter for sigma point calculation
   float32_t m_lambda{0};

   // Track state over time (can be used for smoothing or diagnostics)
   VectorEigenVecDimX m_vecXPredHist; /// @brief History of predicted state vectors at k+1
   VectorEigenMatDimX m_matPPredHist; /// @brief History of predicted state covariances at k+1
   /// History of filtered (after correction) state vectors at k
   VectorEigenVecDimX m_vecXFiltHist;
   /// History of filtered (after correction) state covariances at k
   VectorEigenMatDimX m_matPFiltHist;
   /// History of measurement points
   VectorEigenVecDimZ m_vecZHist;
   /// The sigma points after propagation for the last prediction step.
   Matrix<DIM_X, SIGMA_DIM_A> m_matSigmaXPred{Matrix<DIM_X, SIGMA_DIM_A>::Zero()};

public:
   TrackFitterUKFBase() { setParameters(1, 2, 0); }
   ~TrackFitterUKFBase() = default;

   virtual void Reset()
   {
      m_vecX.setZero();
      m_matP.setZero();
      m_vecXa.setZero();
      m_matPa.setZero();
      m_matQaug.setZero();
      m_matQmod.setZero();
      m_matR.setZero();
      m_matSigmaXa.setZero();
      m_matSigmaXPred.setZero();

      // Clear the history vectors
      m_vecXPredHist.clear();
      m_matPPredHist.clear();
      m_vecXFiltHist.clear();
      m_matPFiltHist.clear();
   }

   /**
    * @brief Set the weights used to calculate sigma points.
    */
   void setParameters(float alpha, float beta, float kappa)
   {
      static_assert(DIM_A > 0, "DIM_A is Zero which leads to numerical issue.");

      m_lambda = alpha * alpha * (DIM_A + kappa) - DIM_A;
      float32_t denoTerm = m_lambda + static_cast<float32_t>(DIM_A);

      m_weightM0 = m_lambda / denoTerm;
      m_weightC0 = m_weightM0 + (1.0F - alpha * alpha + beta); // Weight for the mean sigma point in covariance
      m_weighti = 0.5F / denoTerm;
      LOG(info) << "Mean weight " << m_weightM0;
      LOG(info) << "Cov weight: " << m_weightC0;
      LOG(info) << "Shared weight: " << m_weighti;
   }

   /**
    * @brief Set process noise covariance Q to be used in the prediction step.
    */
   void setAugmentNoise(const Matrix<DIM_V, DIM_V> &matQ) { m_matQaug = matQ; }
   /**
    * @brief Set the process noise covariance Q_mod to be used in the prediction step.
    */
   void setModelNoise(const Matrix<DIM_X, DIM_X> &matQmod) { m_matQmod = matQmod; }

   /**
    * @brief Set the measurement noise covariance R to be used in the update step.
    */
   void SetMeasurementNoise(const Matrix<DIM_N, DIM_N> &matR) { m_matR = matR; }
   virtual Vector<DIM_X> &vecX() { return m_vecX; }
   virtual const Vector<DIM_X> &vecX() const { return m_vecX; }

   virtual Matrix<DIM_X, DIM_X> &matP() { return m_matP; }
   virtual const Matrix<DIM_X, DIM_X> &matP() const { return m_matP; }
   const VectorEigenVecDimX &GetFilteredStates() const { return m_vecXFiltHist; }
   const VectorEigenMatDimX &GetFilteredCovariances() const { return m_matPFiltHist; }
   const VectorEigenVecDimX &GetPredictedStates() const { return m_vecXPredHist; }
   const VectorEigenMatDimX &GetPredictedCovariances() const { return m_matPPredHist; }

   /**
    * @brief update the augmented state vector and covariance matrix
    *
    * This function fully updates the augmented state vector and covariance matrix using
    * the state and process noise.
    */
   void updateAugmentedStateAndCovariance()
   {
      updateAugWithState();
      updateAugWithProcessNoise();
      // ensurePD(m_matPa); // Ensure the augmented covariance matrix is positive definite
   }

   /**
    * @brief state prediction step of the unscented Kalman filter (UKF).
    * @param predictionModelFunc callback to the prediction/process model function.
    * @param vecZ actual measurement vector.`
    *
    * A template is used here for performace reasons since there is only really a single prediction
    * model used. Honestly it probably does not make a difference compared to an std::function, but
    * this was not changed from OpenKF.
    *
    * This modifies the state vector and state covariance.
    */
   template <typename PredictionModelCallback>
   void predictUKF(PredictionModelCallback predictionModelFunc, const Vector<DIM_Z> &vecZ)
   {
      updateAugmentedStateAndCovariance();

      // Calculate the sigma points for the augmented state vector and save in a matrix where each column is a sigma
      // point.
      m_matSigmaXa = calculateSigmaPoints(m_vecXa, m_matPa);

      // Pull out the sigma points for the state vector
      m_matSigmaXPred = m_matSigmaXa.block(0, 0, DIM_X, SIGMA_DIM_A);

      // Pull out the sigma points for the state vector and process noise in two different matrices.
      Matrix<DIM_X, SIGMA_DIM_A> sigmaXx{m_matSigmaXa.block(0, 0, DIM_X, SIGMA_DIM_A)}; // Sigma points for state vector
      Matrix<DIM_V, SIGMA_DIM_A> sigmaXv{
         m_matSigmaXa.block(DIM_X, 0, DIM_V, SIGMA_DIM_A)}; // Sigma points for process noise

      // Get each sigma point, apply the prediction model function, and store the results
      // back into the sigmaXx matrix (safe since each sigma point is independent).
      for (int32_t i{0}; i < SIGMA_DIM_A; ++i) {
         const Vector<DIM_X> sigmaXxi{util::getColumnAt<DIM_X, SIGMA_DIM_A>(i, sigmaXx)};
         const Vector<DIM_V> sigmaXvi{util::getColumnAt<DIM_V, SIGMA_DIM_A>(i, sigmaXv)};

         const Vector<DIM_X> Yi{predictionModelFunc(sigmaXxi, sigmaXvi, vecZ)}; // y = f(x)

         // Copy the predicted state vector back into the sigmaXx matrix.
         util::copyToColumn<DIM_X, SIGMA_DIM_A>(i, sigmaXx, Yi);
         // Copy the predicted state vector back into the augmented state vector (for use in future functions).
         util::copyToColumn<DIM_A, SIGMA_DIM_A, DIM_X>(i, m_matSigmaXa, Yi);
      }

      // Calculate the weighted mean and covariance of the sigma points for the state vector.
      // This will be the new state vector and covariance matrix.
      calculateWeightedMeanAndCovariance<DIM_X>(sigmaXx, m_vecX, m_matP);
      logEigen("P-", m_matP, 0); // Log the eigenvalues of the covariance matrix
      m_matP += m_matQmod;       // Add the model process noise covariance to the state covariance matrix.
      ensurePD(m_matP);          // Ensure the covariance matrix is positive definite

      if (m_vecXPredHist.size() == 1 && kReplaceFirstCov) {
         // If this is the first prediction step, we replace the initial covariance with the predicted covariance
         // of the second point. This is to avoid numerical issues with the first point on smoothing.
         LOG(info) << "Replacing initial covariance with predicted covariance of second point.";
         LOG(info) << "Initial COV: " << m_matPPredHist[0];
         m_matPPredHist[0] = m_matP;
         LOG(info) << "Replaced COV: " << m_matPPredHist[0];
      }
      // Now we need to store the predicted state and covariance for smoothing later.
      m_vecXPredHist.push_back(m_vecX); // Store the predicted state vector
      m_matPPredHist.push_back(m_matP); // Store the predicted covariance matrix

      logEigen("P-Post", m_matP, 0);
   }

   /**
    * @brief measurement correction step of the unscented Kalman filter (UKF).
    * @param measurementModelFunc callback to the measurement model function
    * @param vecZ actual measurement vector.
    */
   template <typename MeasurementModelCallback>
   void correctUKF(MeasurementModelCallback measurementModelFunc, const Vector<DIM_Z> &vecZ)
   {
      // The state vector used here is an unaugmented state vector (m_vecX) and the covariance matrix is
      // an unaugmented covariance matrix (m_matP). This is because we are assuming the measurement noise
      // is independent of the state vector and process noise, so we can just use the unaugmented state vector
      // and covariance matrix for the measurement correction step, then add the measurement noise covariance.

      // Pull out the sigma points for the state vector after prediction.
      Matrix<DIM_X, SIGMA_DIM_A> sigmaXx{m_matSigmaXa.block(0, 0, DIM_X, SIGMA_DIM_A)}; // Sigma points for state vector

      // Get each sigma point, apply the prediction model function, and store the results
      // in the sigmaZ matrix.
      Matrix<DIM_Z, SIGMA_DIM_A> sigmaZ;
      for (int32_t i{0}; i < SIGMA_DIM_A; ++i) {
         const Vector<DIM_X> sigmaXxi{util::getColumnAt<DIM_X, SIGMA_DIM_A>(i, sigmaXx)};

         const Vector<DIM_Z> Zi{measurementModelFunc(sigmaXxi)}; // z = h(x)

         util::copyToColumn<DIM_Z, SIGMA_DIM_A>(i, sigmaZ, Zi);
      }

      // calculate the mean measurement vector and covariance matrix
      // from the sigma points.
      Vector<DIM_Z> vecZhat;       // Predicted measurement vector
      Matrix<DIM_Z, DIM_Z> matPzz; // Measurement covariance matrix
      calculateWeightedMeanAndCovariance<DIM_Z>(sigmaZ, vecZhat, matPzz);

      // Add in the measurement noise covariance matrix to the measurement covariance matrix.
      matPzz += m_matR; // Add measurement noise covariance so we gen the innovation covariance matrix.
      logEigen("S", matPzz, 0);
      ensurePD(matPzz); // Ensure the covariance matrix is positive definite

      const Matrix<DIM_X, DIM_Z> matPxz{calculateCrossCorrelation(sigmaXx, m_vecX, sigmaZ, vecZhat)};

      // kalman gain
      auto llt = calculateCholesky(matPzz);
      const Matrix<DIM_X, DIM_Z> matK = llt.solve(matPxz.transpose()).transpose();
      // Matrix<DIM_X, DIM_Z> matK = {matPxz * llt.solve(Matrix<DIM_Z, DIM_Z>::Identity())};
      //  Matrix<DIM_X, DIM_Z> matK = matPxz * matPzz.inverse(); By far the worst method for filter stability

      m_vecX += matK * (vecZ - vecZhat);
      m_matP -= matK * matPzz * matK.transpose();
      // m_matP -= matPxz * matK.transpose();
      ensurePD(m_matP); // Ensure the covariance matrix is positive definite

      // After correction we need to save the filtered state
      m_vecXFiltHist.push_back(m_vecX); // Store the filtered state vector
      m_matPFiltHist.push_back(m_matP); // Store the filtered covariance matrix
   }

   template <typename T>
   void logEigen(std::string tag, const T &P, int k)
   {
      if (!kLogEigen) {
         return; // If logging is disabled, do not log the eigenvalues.
      }
      Eigen::SelfAdjointEigenSolver<T> es(P);
      double lmin = es.eigenvalues().minCoeff();
      double lmax = es.eigenvalues().maxCoeff();
      double cond = lmax / lmin;

      LOG(info) << "k: " << k << " " << tag << " Eval: min = " << lmin << ", max = " << lmax
                << ", condition number = " << cond;
   }

protected:
   /**
    * @brief Add state vector and state covariance matrix to the augmented state vector covariance  matrix.
    */
   void updateAugWithState()
   {
      // Copy state vector to augmented state vector
      for (int32_t i{0}; i < DIM_X; ++i) {
         m_vecXa[i] = m_vecX[i];
      }

      // Copy state covariance matrix to augmented covariance matrix
      for (int32_t i{0}; i < DIM_X; ++i) {
         for (int32_t j{0}; j < DIM_X; ++j) {
            m_matPa(i, j) = m_matP(i, j);
         }
      }
   }

   virtual std::array<float32_t, DIM_V> calculateProcessNoiseMean() { return std::array<float32_t, DIM_V>{0}; }

   virtual Matrix<DIM_V, DIM_V> calculateProcessNoiseCovariance() { return m_matQaug; }

   void updateAugWithProcessNoise()
   {
      auto processNoiseMean = calculateProcessNoiseMean();
      m_matQaug = calculateProcessNoiseCovariance();

      // Add the mean process noise to the augmented state vector
      for (int32_t i{0}; i < DIM_V; ++i) {
         m_vecXa[DIM_X + i] = processNoiseMean[i];
      }

      // Add process noise covariance to the augmented covariance matrix
      const int32_t S_IDX{DIM_X};
      const int32_t L_IDX{S_IDX + DIM_V};

      for (int32_t i{S_IDX}; i < L_IDX; ++i) {
         for (int32_t j{S_IDX}; j < L_IDX; ++j) {
            m_matPa(i, j) = m_matQaug(i - S_IDX, j - S_IDX);
         }
      }
   }

   /**
    * @brief Calculate Cholesky decomposition of a covariance matrix and update the matrix so it is PD.
    *
    * Modifies the input matrix to ensure it is symmetric and positive definite.
    * If the decomposition fails, it attempts to regularize the matrix by adding a small value
    * to the diagonal and retrying the decomposition.
    */
   template <int32_t STATE_DIM>
   Eigen::LLT<Matrix<STATE_DIM, STATE_DIM>> calculateCholesky(const Matrix<STATE_DIM, STATE_DIM> &matP)
   {
      Eigen::LLT<Matrix<STATE_DIM, STATE_DIM>> lltOfP(matP);
      if (lltOfP.info() != Eigen::Success) {
         throw std::runtime_error("Cholesky decomposition failed, matrix is not positive definite.");
      }

      return lltOfP; // Return the Cholesky decomposition of the covariance matrix
   }

   template <int32_t STATE_DIM>
   Eigen::LLT<Matrix<STATE_DIM, STATE_DIM>> ensurePD(Matrix<STATE_DIM, STATE_DIM> &matP)
   {
      symmetrize(matP);
      Eigen::LLT<Matrix<STATE_DIM, STATE_DIM>> lltOfP(matP);
      if (lltOfP.info() != Eigen::Success) {
         LOG(warn) << "Cholesky decomposition failed while ensuring PD. Attempting recovery...";
         // Add a small value to the diagonal to regularize the matrix
         int i = 0;
         while (lltOfP.info() != Eigen::Success && i < 3) {
            LOG(debug) << "Attempting to regularize covariance matrix, iteration: " << i;

            symmetrize(matP); // Ensure symmetry before regularization
            matP += Matrix<STATE_DIM, STATE_DIM>::Identity() * std::pow(10, -6 + i); // Regularization value
            lltOfP.compute(matP);
            i = i + 1;
            nTouch++;
            // Check if the regularized matrix is now positive definite
         }
         if (lltOfP.info() != Eigen::Success) {
            LOG(error) << "\n" << matP;
            throw std::runtime_error(
               "Cholesky decomposition failed, matrix is not positive definite even after regularization.");
         } else
            LOG(warn) << "Cholesky decomposition succeeded after regularization of order " << i - 1;
      }
      return lltOfP; // Return the Cholesky decomposition of the covariance matrix
   }

   template <int STATE_DIM>
   void symmetrize(Matrix<STATE_DIM, STATE_DIM> &matP)
   {
      // Ensure the matrix is symmetric
      matP = (matP + matP.transpose()) * 0.5;
   }

   /**
    * @brief Algorithm to calculate the deterministic sigma points for
    * the unscented transformation.
    *
    * @param vecX Mean of the normally distributed state.
    * @param matPxx Covariance of the normally distributed state.
    * @param STATE_DIM Dimension of the vector used to calculate the sigma points.
    * @param SIGMA_DIM Number of sigma points required (default is 2 * STATE_DIM + 1).
    * @return Matrix of sigma points where each column is a sigma point.
    */
   template <int32_t STATE_DIM, int32_t SIGMA_DIM = 2 * STATE_DIM + 1>
   Matrix<STATE_DIM, SIGMA_DIM>
   calculateSigmaPoints(const Vector<STATE_DIM> &vecXa, const Matrix<STATE_DIM, STATE_DIM> &matPa)
   {
      const float32_t scalarMultiplier{std::sqrt(STATE_DIM + m_lambda)}; // sqrt(n + \kappa)

      Eigen::LLT<Matrix<STATE_DIM, STATE_DIM>> lltOfPa = calculateCholesky<STATE_DIM>(matPa);

      Matrix<STATE_DIM, STATE_DIM> matSa{lltOfPa.matrixL()}; // sqrt(P_{a})

      matSa *= scalarMultiplier; // sqrt( (n + \kappa) * P_{a} )

      Matrix<STATE_DIM, SIGMA_DIM> sigmaXa;

      // X_0 = \bar{xa}
      util::copyToColumn<STATE_DIM, SIGMA_DIM>(0, sigmaXa, vecXa);

      for (int32_t i{0}; i < STATE_DIM; ++i) {
         const int32_t IDX_1{i + 1};
         const int32_t IDX_2{i + STATE_DIM + 1};

         util::copyToColumn<STATE_DIM, SIGMA_DIM>(IDX_1, sigmaXa, vecXa);
         util::copyToColumn<STATE_DIM, SIGMA_DIM>(IDX_2, sigmaXa, vecXa);

         const Vector<STATE_DIM> vecShiftTerm{util::getColumnAt<STATE_DIM, STATE_DIM>(i, matSa)};

         util::addColumnFrom<STATE_DIM, SIGMA_DIM>(IDX_1, sigmaXa, vecShiftTerm); // X_i^a     = \bar{xa} + sqrt( (n^a +
                                                                                  // \kappa) * P^{a} )
         util::subColumnFrom<STATE_DIM, SIGMA_DIM>(IDX_2, sigmaXa, vecShiftTerm); // X_{i+n}^a = \bar{xa} - sqrt( (n^a +
                                                                                  // \kappa) * P^{a} )
      }

      return sigmaXa;
   }

   /**
    * @brief Calculate the weighted mean and covariance given a set of sigma points.
    * @param[in] sigmaX Matrix of (probably posterior) sigma points where each column contains a single sigma point.
    * @param[out] vecX Output weighted mean of the sigma points.
    * @param[out] matPxx Output weighted covariance of the sigma points.
    */
   template <int32_t STATE_DIM, int32_t SIGMA_DIM>
   void calculateWeightedMeanAndCovariance(const Matrix<STATE_DIM, SIGMA_DIM> &sigmaX, Vector<STATE_DIM> &vecX,
                                           Matrix<STATE_DIM, STATE_DIM> &matPxx)
   {
      // 1. calculate mean of the sigma points
      vecX = m_weightM0 * util::getColumnAt<STATE_DIM, SIGMA_DIM>(0, sigmaX);
      for (int32_t i{1}; i < SIGMA_DIM; ++i) {
         vecX += m_weighti * util::getColumnAt<STATE_DIM, SIGMA_DIM>(i, sigmaX); // y += W[0, i] Y[:, i]
      }

      // 2. calculate covariance: P_{yy} = \sum_{i_0}^{2n} W[0, i] (Y[:, i] -
      // \bar{y}) (Y[:, i] - \bar{y})^T
      Vector<STATE_DIM> devXi{util::getColumnAt<STATE_DIM, SIGMA_DIM>(0, sigmaX) - vecX}; // Y[:, 0] - \bar{ y }

      matPxx = m_weightC0 * devXi * devXi.transpose(); // P_0 = W[0, 0] (Y[:, 0] - \bar{y}) (Y[:, 0] -
                                                       // \bar{y})^T

      for (int32_t i{1}; i < SIGMA_DIM; ++i) {
         devXi = util::getColumnAt<STATE_DIM, SIGMA_DIM>(i, sigmaX) - vecX; // Y[:, i] - \bar{y}

         const Matrix<STATE_DIM, STATE_DIM> Pi{m_weighti * devXi * devXi.transpose()}; // P_i = W[0, i] (Y[:, i] -
                                                                                       // \bar{y}) (Y[:, i] - \bar{y})^T

         matPxx += Pi; // y += W[0, i] (Y[:, i] - \bar{y}) (Y[:, i] - \bar{y})^T
      }
      // ensurePD(matPxx);         // Ensure the covariance matrix is positive definite
   }

   /**
    * @brief calculate the cross-correlation given two sets sigma points X and Y
    * and their means x and y
    * @param sigmaX first matrix of sigma points where each column contain
    * single sigma point
    * @param vecX mean of the first set of sigma points
    * @param sigmaY second matrix of sigma points where each column contain
    * single sigma point
    * @param vecY mean of the second set of sigma points
    * @return matPxy, the cross-correlation matrix
    */
   template <int32_t STATE_DIM, int32_t MEAS_DIM, int32_t SIGMA_DIM>
   Matrix<STATE_DIM, MEAS_DIM>
   calculateCrossCorrelation(const Matrix<STATE_DIM, SIGMA_DIM> &sigmaX, const Vector<STATE_DIM> &vecX,
                             const Matrix<MEAS_DIM, SIGMA_DIM> &sigmaY, const Vector<MEAS_DIM> &vecY)
   {
      Vector<STATE_DIM> devXi{util::getColumnAt<STATE_DIM, SIGMA_DIM>(0, sigmaX) - vecX}; // X[:, 0] - \bar{ x }
      Vector<MEAS_DIM> devYi{util::getColumnAt<MEAS_DIM, SIGMA_DIM>(0, sigmaY) - vecY};   // Y[:, 0] - \bar{ y }

      // P_0 = W[0, 0] (X[:, 0] - \bar{x}) (Y[:, 0] - \bar{y})^T
      Matrix<STATE_DIM, MEAS_DIM> matPxy{m_weightC0 * (devXi * devYi.transpose())};

      for (int32_t i{1}; i < SIGMA_DIM; ++i) {
         devXi = util::getColumnAt<STATE_DIM, SIGMA_DIM>(i, sigmaX) - vecX; // X[:, i] - \bar{x}
         devYi = util::getColumnAt<MEAS_DIM, SIGMA_DIM>(i, sigmaY) - vecY;  // Y[:, i] - \bar{y}

         matPxy += m_weighti * (devXi * devYi.transpose()); // y += W[0, i] (Y[:, i] -
                                                            // \bar{y}) (Y[:, i] - \bar{y})^T
      }

      return matPxy;
   }
};

/**
 * @brief Class for fitting tracks using the Unscented Kalman Filter (UKF) algorithm.
 *
 * UKF specialized for fitting tracks. This class is where all of the physics is
 */
class TrackFitterUKF : public TrackFitterUKFBase<6, 3, 1, 3> {
protected:
   static constexpr int32_t TF_DIM_X = 6;
   static constexpr int32_t TF_DIM_Z = 3;
   static constexpr int32_t TF_DIM_V = 1;
   static constexpr int32_t TF_DIM_N = 3;

   AtTools::AtPropagator fPropagator; ///< @brief Propagator for the track fitter
   std::unique_ptr<AtTools::AtStepper> fStepper{nullptr};
   AtTools::AtPropagator::StepState fMeanStep; /// Holds the step information for POCA propagation of mean state
   ROOT::Math::Plane3D fMeasurementPlane;      ///< Holds the measurement plane for the track fitter

   /// Smoothed state vector and covariance
   VectorEigenVecDimX m_vecXSmooth;
   /// Smoothed state covariance
   VectorEigenMatDimX m_matPSmooth;
   /// History of cross correlation between filtered state at k and predicted at k+1
   VectorEigenMatDimX m_matCPredHist;

public:
   bool fEnableEnStraggling{true};       ///< @brief Flag to enable/disable energy straggling
   double fMaxStragglingFactor{1. / 3.}; ///< @brief Maximum straggling factor for energy loss
                                         /**
                                          * Uncertainty in the position of the propagated track. Used to set a floor
                                          * in the model error covariance. Physically this represents the unmodeled
                                          * properties (error in RK4, etc.) Numerically is breaks the strong correlation that
                                          * exists between x/y/z in the propagator when the track is basically straight.
                                          */
   double fPosModelNoise{1e-4};

   /**
    * @brief Constructor for the TrackFitterUKF class.
    * @param propagator The propagator to be used for the track fitting, must be passed as an rvalue reference.
    *
    * Example usage:
    * ```
    * AtTools::AtPropagator propagator;
    * kf::TrackFitterUKF trackFitterUKF(std::move(propagator));
    * ```
    */
   TrackFitterUKF(AtTools::AtPropagator &&propagator, std::unique_ptr<AtTools::AtStepper> &&stepper)
      : TrackFitterUKFBase(), fPropagator(std::move(propagator)), fStepper(std::move(stepper))
   {
   }
   virtual void Reset() override;
   void SetInitialState(const ROOT::Math::XYZPoint &initialPosition, const ROOT::Math::XYZVector &initialMomentum,
                        const TMatrixD &initialCovariance);

   void SetMeasCov(const TMatrixD &measCov);

   TMatrixD GetStateCovariance() const;
   TMatrixD GetAugStateCovariance() const;
   std::array<double, DIM_A> GetAugStateVector() const;
   const VectorEigenVecDimX &GetSmoothedStates() const { return m_vecXSmooth; };
   const VectorEigenMatDimX &GetSmoothedCovariances() const { return m_matPSmooth; };

   void predictUKF(const ROOT::Math::XYZPoint &z);
   void correctUKF(const ROOT::Math::XYZPoint &z);
   void smoothUKF();

protected:
   std::array<float32_t, TF_DIM_V> calculateProcessNoiseMean() override;
   Matrix<TF_DIM_V, TF_DIM_V> calculateProcessNoiseCovariance() override;

   kf::Vector<TF_DIM_X>
   funcF(const kf::Vector<TF_DIM_X> &x, const kf::Vector<TF_DIM_V> &v, const kf::Vector<TF_DIM_Z> &z);
   kf::Vector<TF_DIM_Z> funcH(const kf::Vector<TF_DIM_X> &x);
};
} // namespace kf

#endif // TRACKFITTERUKF_H
