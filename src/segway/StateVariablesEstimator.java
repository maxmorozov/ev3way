package segway;

/**
 * Interface to calculate angular velocity and body angle
 *
 * @author Max Morozov
 */
public interface StateVariablesEstimator {
    /**
     * Returns angular velocity
     *
     * @return angular velocity in degree/sec
     */
    float getAngularVelocity();

    /**
     * Returns body angle
     *
     * @return body angle in degree
     */
    float getAngle();

    /**
     * Update the estimator's state
     *
     * @param rate angular velocity in degree/sec
     * @param angle body angle in degree
     * @param interval execution interval in seconds
     */
    void updateState(float rate, float angle, float interval);

    /**
     * Updates the initial gyroscope offset
     *
     * @param gyroOffset initial gyro offset in degrees
     */
    void init(float gyroOffset);
}
