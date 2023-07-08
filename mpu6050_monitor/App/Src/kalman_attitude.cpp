#include "kalman_attitude.h"

kalman_attitude_t::kalman_attitude_t() noexcept
    : Q_angle(3.05e-07f), Q_bias(9.14e-07f), R_measure(9.14e-06f), angle(0.0f), bias(0.0f), rate(0.0f),
      P{0.0f, 0.0f, 0.0f, 0.0f}
{
}

double kalman_attitude_t::update(double new_angle, double new_rate, double dt) noexcept
{
    // Predict the state
    rate = new_rate - bias;
    angle += dt * rate;

    // Calculate the error covariance of the predicted state
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Calculate Kalman gain
    double S = P[0][0] + R_measure; // the covariance of innovation
    double K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate the state
    double innovation = new_angle - angle;
    angle += K[0] * innovation;
    bias += K[1] * innovation;

    // Calculate the error covariance
    double p00 = P[0][0];
    double p01 = P[0][1];

    P[0][0] -= K[0] * p00;
    P[0][1] -= K[0] * p01;
    P[1][0] -= K[1] * p00;
    P[1][1] -= K[1] * p01;

    return angle;
}

void kalman_attitude_t::set_angle(double new_angle) noexcept
{
    this->angle = new_angle;
}

double kalman_attitude_t::get_angle() const noexcept
{
    return this->angle;
}

double kalman_attitude_t::get_rate() const noexcept
{
    return this->rate;
}
