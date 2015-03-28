package segway.estimators;

import segway.StateVariablesEstimator;
import segway.Constants;

/**
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
     * @param gyroValue angular velocity in degree/sec
     * @param interval  execution interval in seconds
     */
    @Override
    public void updateState(float gyroValue, float interval) {
        //Use low-pass filter to find average angular velocity
        gyroOffset = gyroOffset * Constants.GYRO_COMPENSATION_FILTER + (1 - Constants.GYRO_COMPENSATION_FILTER) * gyroValue;
        angularVelocity = angularVelocity * Constants.GYRO_FILTER + (1 - Constants.GYRO_FILTER) * (gyroValue - gyroOffset);

        angle = nextAngle;
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
