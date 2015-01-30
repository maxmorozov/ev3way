package navigation;

/**
 * @author Max Morozov
 */
public interface Navigator {
    /**
     * Returns the encoded control.
     * <p/>
     * Low byte - speed of the forward movement. -100(backward max.) to 100(forward max.)
     * <p/>
     * High byte - speed of the turning. -100(turn left max.) to 100(turn right max.)
     *
     * @return encoded control
     */
    short getControl();

    /**
     * Notifies the navigator that som obstacle is detected or not detected
     *
     * @param isObstacle true if an obstacle is detected, false otherwise
     */
    void obstacleDetected(boolean isObstacle);

    /**
     * Indicates that the robot can move without external control
     */
    void enableAutonomousDrive();
}
