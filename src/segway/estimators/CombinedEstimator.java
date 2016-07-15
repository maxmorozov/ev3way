package segway.estimators;

import segway.StateVariablesEstimator;

/**
 * This estimator uses Kalman filter to calculate gyroscope offset and to smooth
 * angular velocity values.
 * It uses integration to calculate angle values.
 */
public class CombinedEstimator implements StateVariablesEstimator {
    static final float psi_ref = -9.5f;    /* equilibrium point angle. The robot does not have perfect symmetry and the equilibrium point angle is not zero. */

    private float angularVelocity = 0;
    private float angle = 0;
    private float nextAngle = 0;

    /**
     * Kalman filter to evaluate gyro bias
     */
    private final TiltFilter filter = new TiltFilter();

    /**
     * Returns angular velocity
     *
     * @return angular velocity in degree/sec
     */
    @Override
    public float getAngularVelocity() {
        return angularVelocity;
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
        //Kalman filter update
        filter.state_update(rate, interval);
        filter.kalman_update(angle - psi_ref);

        angularVelocity = filter.get_kalman_rate();

        this.angle = nextAngle;
        nextAngle += (interval * angularVelocity);
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
