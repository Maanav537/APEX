#include <apex/gnc/observer.hpp>

#include <apex/gnc/bearing_sensor.hpp>

namespace apex {

CatObserver::CatObserver(const CatObserverParams& params) : params_(params), rng_(params.seed) {}

Vec3 CatObserver::measure(const Vec3& r_true) {
    return sensor_reading(r_true, params_.range_sigma, params_.angle_sigma, rng_, unit_noise_);
}

void CatObserver::update(const Vec3& own_position, const Vec3& r_true, const Vec3& own_velocity) {
    own_position_ = own_position;
    reading_ = measure(r_true);
    const Vec3 z = own_position + reading_;

    const double d = reading_.norm();
    const double r_var =
        params_.range_sigma * params_.range_sigma +
        (d * params_.angle_sigma) * (d * params_.angle_sigma);

    if (!started_) {
        p_hat_ = z;
        v_hat_ = own_velocity;
        a_hat_ = Vec3::Zero();
        P_.setZero();
        P_(0, 0) = r_var;
        P_(1, 1) = 100.0 * 100.0;
        P_(2, 2) = 100.0 * 100.0;
        k0_ = 1.0;
        started_ = true;
        return;
    }

    const double dt = params_.dt;
    const double dt2 = 0.5 * dt * dt;

    Mat3 F = Mat3::Identity();
    F(0, 1) = dt;
    F(0, 2) = dt2;
    F(1, 2) = dt;

    const Vec3 g{dt2, dt, 1.0};
    const Mat3 Q =
        (params_.accel_process_sigma * params_.accel_process_sigma) * (g * g.transpose());

    const Vec3 p_pred = p_hat_ + v_hat_ * dt + a_hat_ * dt2;
    const Vec3 v_pred = v_hat_ + a_hat_ * dt;
    const Vec3 a_pred = a_hat_;
    const Mat3 P_pred = F * P_ * F.transpose() + Q;

    const double S = P_pred(0, 0) + r_var + 1e-12;
    const Vec3 K = P_pred.col(0) / S;
    k0_ = K(0);

    const Vec3 e = z - p_pred;
    innovation_ = e;
    p_hat_ = p_pred + K(0) * e;
    v_hat_ = v_pred + K(1) * e;
    a_hat_ = a_pred + K(2) * e;

    Mat3 I_KH = Mat3::Identity();
    I_KH(0, 0) -= K(0);
    I_KH(1, 0) -= K(1);
    I_KH(2, 0) -= K(2);
    P_ = I_KH * P_pred;
}

}  // namespace apex
