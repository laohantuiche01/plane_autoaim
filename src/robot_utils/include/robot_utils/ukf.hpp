#ifndef ROBOT_UTILS__UKF_HPP_
#define ROBOT_UTILS__UKF_HPP_

#include <Eigen/Dense>
#include <functional>
#include <vector>
#include <cmath>
#include <iostream>

namespace robot_utils {

/**
 * @brief Unscented Kalman Filter (with advanced Gating and Adaptive capabilities)
 * 
 * @tparam N Dimension of the state vector
 */
template <int N>
class UKF {
public:
    using VectorX = Eigen::Matrix<double, N, 1>;
    using MatrixX = Eigen::Matrix<double, N, N>;
    using SigmaPoints = Eigen::Matrix<double, N, 2 * N + 1>;
    using Weights = Eigen::Matrix<double, 2 * N + 1, 1>;

    // State difference normalization function type
    using StateNormalizeFunc = std::function<void(VectorX&)>;

    /**
     * @brief Construct a new UKF object
     * 
     * @param alpha Spread of the sigma points (typically 1e-3)
     * @param beta Prior knowledge of the distribution (2.0 for Gaussian)
     * @param kappa Secondary scaling parameter (usually 0)
     */
    UKF(double alpha = 0.001, double beta = 2.0, double kappa = 0.0) 
        : alpha_(alpha), beta_(beta), kappa_(kappa) {
        
        lambda_ = alpha_ * alpha_ * (N + kappa_) - N;
        
        weights_c_.resize(2 * N + 1);
        weights_m_.resize(2 * N + 1);
        
        weights_m_(0) = lambda_ / (N + lambda_);
        weights_c_(0) = weights_m_(0) + (1.0 - alpha_ * alpha_ + beta_);
        
        for (int i = 1; i < 2 * N + 1; ++i) {
            double w = 0.5 / (N + lambda_);
            weights_m_(i) = w;
            weights_c_(i) = w;
        }
    }

    /**
     * @brief Initialize the UKF state and covariance
     */
    void init(const VectorX& x0, const MatrixX& P0) {
        x_ = x0;
        P_ = P0;
    }

    /**
     * @brief Predict step with Fading Memory (衰减记忆自适应)
     * 
     * @param f State transition function: x_{k} = f(x_{k-1})
     * @param Q Process noise covariance
     * @param normalize_diff Optional function to normalize state differences
     * @param fading_factor Factor >= 1.0 (e.g. 1.01). Increases P to prevent filter saturation and keep it responsive to sudden maneuvers.
     */
    void predict(const std::function<VectorX(const VectorX&)>& f, 
                 const MatrixX& Q,
                 const StateNormalizeFunc& normalize_diff = nullptr,
                 double fading_factor = 1.0) {
        SigmaPoints sigmas = generateSigmaPoints(x_, P_);
        
        // Predict sigma points
        for (int i = 0; i < 2 * N + 1; ++i) {
            sigmas.col(i) = f(sigmas.col(i));
        }
        
        // Predict state mean
        x_.setZero();
        for (int i = 0; i < 2 * N + 1; ++i) {
            x_ += weights_m_(i) * sigmas.col(i);
        }
        
        // Predict state covariance
        P_.setZero();
        for (int i = 0; i < 2 * N + 1; ++i) {
            VectorX diff = sigmas.col(i) - x_;
            if (normalize_diff) {
                normalize_diff(diff);
            }
            P_ += weights_c_(i) * diff * diff.transpose();
        }
        
        // Apply fading memory
        if (fading_factor > 1.0) {
            P_ *= fading_factor; 
        }
        
        P_ += Q;
        
        // Save predicted sigma points for update step
        predicted_sigmas_ = sigmas;
    }

    /**
     * @brief Standard Update step with Outlier Rejection (卡方检验/门限过滤)
     * 
     * @tparam M Dimension of the measurement vector
     * @param z Actual measurement
     * @param h Measurement function: z_{k} = h(x_{k})
     * @param R Measurement noise covariance
     * @param normalize_meas_diff Optional function to normalize measurement innovation
     * @param normalize_state_diff Optional function to normalize state differences
     * @param mahalanobis_thresh Chi-square threshold (e.g., 9.48 for 4 DoF). If > 0 and exceeded, returns false (measurement rejected).
     * @return true if updated, false if rejected as outlier
     */
    template <int M>
    bool update(const Eigen::Matrix<double, M, 1>& z,
                const std::function<Eigen::Matrix<double, M, 1>(const VectorX&)>& h,
                const Eigen::Matrix<double, M, M>& R,
                const std::function<void(Eigen::Matrix<double, M, 1>&)>& normalize_meas_diff = nullptr,
                const StateNormalizeFunc& normalize_state_diff = nullptr,
                double mahalanobis_thresh = -1.0) {
        
        using VectorZ = Eigen::Matrix<double, M, 1>;
        using MatrixZ = Eigen::Matrix<double, M, M>;
        using MatrixXZ = Eigen::Matrix<double, N, M>;
        using SigmaPointsZ = Eigen::Matrix<double, M, 2 * N + 1>;
        
        SigmaPointsZ Z_sigmas;
        
        // Transform sigma points into measurement space
        for (int i = 0; i < 2 * N + 1; ++i) {
            Z_sigmas.col(i) = h(predicted_sigmas_.col(i));
        }
        
        // Mean predicted measurement
        VectorZ z_pred = VectorZ::Zero();
        for (int i = 0; i < 2 * N + 1; ++i) {
            z_pred += weights_m_(i) * Z_sigmas.col(i);
        }
        
        // Measurement covariance and Cross covariance
        MatrixZ S = MatrixZ::Zero();
        MatrixXZ Tc = MatrixXZ::Zero();
        
        for (int i = 0; i < 2 * N + 1; ++i) {
            VectorZ z_diff_sigma = Z_sigmas.col(i) - z_pred;
            if (normalize_meas_diff) normalize_meas_diff(z_diff_sigma);
            
            VectorX x_diff_sigma = predicted_sigmas_.col(i) - x_;
            if (normalize_state_diff) normalize_state_diff(x_diff_sigma);
            
            S += weights_c_(i) * z_diff_sigma * z_diff_sigma.transpose();
            Tc += weights_c_(i) * x_diff_sigma * z_diff_sigma.transpose();
        }
        S += R;
        
        // Actual residual
        VectorZ z_diff = z - z_pred;
        if (normalize_meas_diff) {
            normalize_meas_diff(z_diff);
        }

        // Outlier Rejection via Mahalanobis Distance
        if (mahalanobis_thresh > 0.0) {
            double mahalanobis_sq = z_diff.transpose() * S.inverse() * z_diff;
            if (mahalanobis_sq > mahalanobis_thresh) {
                return false; // Gated! Measurement rejected.
            }
        }
        
        // Kalman gain
        MatrixXZ K = Tc * S.inverse();
        
        // Update state mean and covariance
        x_ += K * z_diff;
        P_ -= K * S * K.transpose();
        
        return true;
    }

    /**
     * @brief Adaptive Update (Sage-Husa like) - dynamically adjusts the R matrix based on innovations.
     * Useful when sensor noise is dynamic (e.g., motion blur degrades vision).
     * 
     * @param R_adaptive Passed by reference, it will be updated in-place!
     * @param alpha Learning rate for R (e.g., 0.1 ~ 0.3)
     */
    template <int M>
    bool updateAdaptive(const Eigen::Matrix<double, M, 1>& z,
                        const std::function<Eigen::Matrix<double, M, 1>(const VectorX&)>& h,
                        Eigen::Matrix<double, M, M>& R_adaptive,
                        double alpha = 0.2,
                        const std::function<void(Eigen::Matrix<double, M, 1>&)>& normalize_meas_diff = nullptr,
                        const StateNormalizeFunc& normalize_state_diff = nullptr,
                        double mahalanobis_thresh = -1.0) {
        
        using VectorZ = Eigen::Matrix<double, M, 1>;
        using MatrixZ = Eigen::Matrix<double, M, M>;
        using MatrixXZ = Eigen::Matrix<double, N, M>;
        using SigmaPointsZ = Eigen::Matrix<double, M, 2 * N + 1>;
        
        SigmaPointsZ Z_sigmas;
        
        for (int i = 0; i < 2 * N + 1; ++i) {
            Z_sigmas.col(i) = h(predicted_sigmas_.col(i));
        }
        
        VectorZ z_pred = VectorZ::Zero();
        for (int i = 0; i < 2 * N + 1; ++i) {
            z_pred += weights_m_(i) * Z_sigmas.col(i);
        }
        
        MatrixZ S_no_R = MatrixZ::Zero();
        MatrixXZ Tc = MatrixXZ::Zero();
        
        for (int i = 0; i < 2 * N + 1; ++i) {
            VectorZ z_diff_sigma = Z_sigmas.col(i) - z_pred;
            if (normalize_meas_diff) normalize_meas_diff(z_diff_sigma);
            
            VectorX x_diff_sigma = predicted_sigmas_.col(i) - x_;
            if (normalize_state_diff) normalize_state_diff(x_diff_sigma);
            
            S_no_R += weights_c_(i) * z_diff_sigma * z_diff_sigma.transpose();
            Tc += weights_c_(i) * x_diff_sigma * z_diff_sigma.transpose();
        }
        
        MatrixZ S = S_no_R + R_adaptive;
        
        VectorZ z_diff = z - z_pred;
        if (normalize_meas_diff) normalize_meas_diff(z_diff);

        if (mahalanobis_thresh > 0.0) {
            double mahalanobis_sq = z_diff.transpose() * S.inverse() * z_diff;
            if (mahalanobis_sq > mahalanobis_thresh) {
                return false; 
            }
        }
        
        // --- Adaptive R Estimation ---
        // R_new = (1-alpha)*R_old + alpha*(z_diff * z_diff^T - S_no_R)
        MatrixZ R_est = z_diff * z_diff.transpose() - S_no_R;
        
        // Constrain diagonal elements to be strictly positive to ensure numeric stability
        for (int i = 0; i < M; ++i) {
            R_est(i, i) = std::max(R_est(i, i), 1e-6);
        }
        // Force symmetry
        R_est = 0.5 * (R_est + R_est.transpose()); 
        
        R_adaptive = (1.0 - alpha) * R_adaptive + alpha * R_est;
        // ------------------------------
        
        MatrixXZ K = Tc * S.inverse();
        x_ += K * z_diff;
        P_ -= K * S * K.transpose();
        
        return true;
    }

    const VectorX& getState() const { return x_; }
    void setState(const VectorX& x) { x_ = x; }

    const MatrixX& getCovariance() const { return P_; }
    void setCovariance(const MatrixX& P) { P_ = P; }

private:
    SigmaPoints generateSigmaPoints(const VectorX& x, const MatrixX& P) {
        SigmaPoints sigmas;
        sigmas.col(0) = x;
        
        MatrixX P_safe = P + MatrixX::Identity() * 1e-6;
        Eigen::LLT<MatrixX> llt(P_safe);
        MatrixX L = llt.matrixL();
        
        double c = std::sqrt(N + lambda_);
        for (int i = 0; i < N; ++i) {
            sigmas.col(i + 1)     = x + c * L.col(i);
            sigmas.col(i + 1 + N) = x - c * L.col(i);
        }
        
        return sigmas;
    }

    VectorX x_;
    MatrixX P_;
    
    SigmaPoints predicted_sigmas_;
    
    double alpha_;
    double beta_;
    double kappa_;
    double lambda_;
    
    Weights weights_m_;
    Weights weights_c_;
};

} // namespace robot_utils

#endif // ROBOT_UTILS__UKF_HPP_