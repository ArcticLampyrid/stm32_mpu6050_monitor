#pragma once

/**
 * A class to provide a Kalman filter for the attitude in one asis.
 */
class kalman_attitude_t
{
  private:
    /**
     * The calculated angle.
     */
    double angle;
    /**
     * The bias of rate.
     */
    double bias;
    /**
     * Unbiased rate.
     */
    double rate;
    /**
     * Error covariance matrix.
     */
    double P[2][2];

  public:
    kalman_attitude_t() noexcept;

    /**
     * Update the attitude. This should be called at intervals.
     * @param new_angle Unit: rad
     * @param new_rate Unit: rad/s
     * @param dt Unit:s
     * @return the calculated angle in unit: rad
     */
    double update(double new_angle, double new_rate, double dt) noexcept;

    /**
     * Set angle. This should be called at the beginning
     * @param angle Unit: rad
     */
    void set_angle(double angle) noexcept;

    /**
     * Get the calculated angle.
     * @return Unit: rad
     */
    [[nodiscard]] double get_angle() const noexcept;

    /**
     * Get the unbiased rate.
     * @return  Unit: rad/s
     */
    [[nodiscard]] double get_rate() const noexcept;

    /**
     * Noise variance for the angle.
     */
    double Q_angle;
    /**
     * Noise variance for the bias of rate.
     */
    double Q_bias;
    /**
     * Noise variance of the measurement.
     */
    double R_measure;
};
