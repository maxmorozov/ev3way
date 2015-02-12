package segway.estimators;

import segway.StateVariablesEstimator;
import segway.utils.SharedState;

/**
 * @author Max Morozov
 */
public class KalmanEstimator implements StateVariablesEstimator {
    static final float psi_ref = -9.5f;    /* equilibrium point angle. The robot does not have perfect symmetry and the equilibrium point angle is not zero. */

    /**
     * Kalman filter to evaluate angle and angular velocity
     */
    private final TiltFilter filter = new TiltFilter();

    /**
     * Last angle from the accelerometer sensor
     */
    private float lastMeasuredAngle = 0;

    private float gyroOffset = 0;
    private float angle = 0;
    private float nextAngle = 0;

    private final SharedState sharedState;

    public KalmanEstimator(SharedState sharedState) {
        this.sharedState = sharedState;
    }

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
     * @param gyroValue angular velocity in degree/sec
     * @param interval  execution interval in seconds
     */
    @Override
    public void updateState(float gyroValue, float interval) {
        angle = nextAngle;

        //Kalman filter update
        filter.state_update(gyroValue - gyroOffset, interval);

        float angle = sharedState.getBodyAngle();
        if (lastMeasuredAngle != angle) {
            filter.kalman_update(angle - psi_ref);
            lastMeasuredAngle = angle;
        }

        nextAngle = filter.get_kalman_angle();
    }

    /**
     * Updates the initial gyroscope offset
     *
     * @param gyroOffset initial gyro offset in degrees
     */
    @Override
    public void init(float gyroOffset) {
        this.gyroOffset = gyroOffset;
    }
}
