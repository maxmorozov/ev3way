package segway.estimators;

import segway.StateVariablesEstimator;
import segway.utils.EstimatorLog;
import segway.utils.SharedState;

/**
 * @author Max Morozov
 */
public class LoggingEstimator implements StateVariablesEstimator {
    private final StateVariablesEstimator lpfEstimator;
    private final StateVariablesEstimator kalmanEstimator;
    private final SharedState sharedState;


    public LoggingEstimator(StateVariablesEstimator lpfEstimator, StateVariablesEstimator kalmanEstimator, SharedState sharedState) {
        this.lpfEstimator = lpfEstimator;
        this.kalmanEstimator = kalmanEstimator;
        this.sharedState = sharedState;
    }


    /**
     * Returns angular velocity
     *
     * @return angular velocity in degree/sec
     */
    @Override
    public float getAngularVelocity() {
        return lpfEstimator.getAngularVelocity();
    }

    /**
     * Returns body angle
     *
     * @return body angle in degree
     */
    @Override
    public float getAngle() {
        return lpfEstimator.getAngle();
    }

    /**
     * Update the estimator's state
     *
     * @param gyroValue angular velocity in degree/sec
     * @param interval  execution interval in seconds
     */
    @Override
    public void updateState(float gyroValue, float interval) {
        lpfEstimator.updateState(gyroValue, interval);
        kalmanEstimator.updateState(gyroValue, interval);

        EstimatorLog.State state = new EstimatorLog.State();
        state.time = System.nanoTime();
        state.gyroValue = gyroValue;
        state.angleValue = sharedState.getBodyAngle();

        state.lpfGyro = lpfEstimator.getAngularVelocity();
        state.lpfAngle = lpfEstimator.getAngle();
        state.lpgGyroOffset = lpfEstimator.getGyroOffset();

        state.kalmanGyro = kalmanEstimator.getAngularVelocity();
        state.kalmanAngle = kalmanEstimator.getAngle();
        state.kalmanGyroOffset = kalmanEstimator.getGyroOffset();

        EstimatorLog.add(state);
    }

    /**
     * Updates the initial gyroscope offset
     *
     * @param gyroOffset initial gyro offset in degrees
     */
    @Override
    public void init(float gyroOffset) {
        lpfEstimator.init(gyroOffset);
        kalmanEstimator.init(gyroOffset);
    }

    @Override
    public float getGyroOffset() {
        return lpfEstimator.getGyroOffset();
    }
}
