package segway;

/**
 * Constants calculated from model by LQR optimization
 *
 * @author Max Morozov
 */
public class Constants {
    // Low Path Filter Coefficients
    public static final float BATTERY_FILTER = 0.8f;       // average battery value
    public static final float GYRO_CALIBRATION_FILTER = 0.8f;	    // calibrate gyro offset
    public static final float GYRO_COMPENSATION_FILTER = 0.999f;	// compensate gyro drift

    public static final int CONTROLLER_TIME = 8; //8 mc
    /**
     * Number of milliseconds for calibrating gyro
     */
    public static final int GYRO_CALIBRATION_PERIOD = 1000;

    /**
     * Number of offset samples to average when calculating gyro offset.
     */
    public static final int GYRO_OFFSET_SAMPLES = 100;

    /**
     * Time to balance the robot before starting the autonomous movement
     */
    public static final int INITIAL_BALANCING_PERIOD = 5000;

    /* minimum distance in cm for obstacle avoidance */
    public static final int MIN_DISTANCE = 80;

    /**
     * Converts radians to degrees
     */
    public static final float RAD_TO_DEGREE = (float)(180 / Math.PI);
}
