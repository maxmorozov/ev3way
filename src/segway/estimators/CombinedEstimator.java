package segway.estimators;

import segway.StateVariablesEstimator;
import segway.utils.SharedState;

/**
 *
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
     * Last angle from the accelerometer sensor
     */
    private float lastMeasuredAngle = 0;

    private final SharedState sharedState;

    public CombinedEstimator(SharedState sharedState) {
        this.sharedState = sharedState;
    }

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
     * @param gyroValue angular velocity in degree/sec
     * @param interval  execution interval in seconds
     */
    @Override
    public void updateState(float gyroValue, float interval) {
        //Kalman filter update
        filter.state_update(gyroValue, interval);

        float angle = sharedState.getBodyAngle();
        if (lastMeasuredAngle != angle) {
            filter.kalman_update(angle - psi_ref);
            lastMeasuredAngle = angle;
        }

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
