package segway.estimators;

import segway.StateVariablesEstimator;
import segway.Constants;

/**
 * This estimator uses low-pass filter to find gyroscope offset
 * Angle is calculated by integration of the angular velocity values
 *
 * @author Max Morozov
 */
public class LowPassFilterEstimator implements StateVariablesEstimator {
    private float gyroOffset = 0;
    private float angularVelocity = 0;
    private float angle = 0;
    private float nextAngle = 0;

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
        //Use low-pass filter to calculate gyroscope offset
        gyroOffset = gyroOffset * Constants.GYRO_COMPENSATION_FILTER + (1 - Constants.GYRO_COMPENSATION_FILTER) * rate;
        angularVelocity = rate - gyroOffset;

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
        this.gyroOffset = gyroOffset;
    }
}
