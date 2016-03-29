package segway.estimators;

import segway.StateVariablesEstimator;

/**
 * @author Max Morozov
 */
public class KalmanEstimator implements StateVariablesEstimator {
    static final float psi_ref = -9.5f;    /* equilibrium point angle. The robot does not have perfect symmetry and the equilibrium point angle is not zero. */

    /**
     * Kalman filter to evaluate angle and angular velocity
     */
    private final TiltFilter filter = new TiltFilter();

    private float angle = 0;
    private float nextAngle = 0;

    /**
     * Returns angular velocity
     *
     * @return angular velocity in degree/sec
     */
    @Override
    public float getAngularVelocity() {
        return filter.get_kalman_rate();
    }

    /**
     * Returns body angle
     *
     * @return body angle in degree
     */
    @Override
    public float getAngle() {
        return angle;
    }

    /**
     * Update the estimator's state
     *
     * @param rate     angular velocity in degree/sec
     * @param angle    body angle in degree
     * @param interval execution interval in seconds
     */
    @Override
    public void updateState(float rate, float angle, float interval) {
        this.angle = nextAngle;

        //Kalman filter update
        filter.state_update(rate, interval);
        filter.kalman_update(angle - psi_ref);

        nextAngle = filter.get_kalman_angle();
    }

    /**
     * Updates the initial gyroscope offset
     *
     * @param gyroOffset initial gyro offset in degrees
     */
    @Override
    public void init(float gyroOffset) {
        filter.set_gyro_bias(gyroOffset);
    }
}
