package segway.navigation;

import segway.Navigator;

/**
 * Keeps the robot in the one state
 *
 * @author Max Morozov
 */
public class SteadyStateNavigator implements Navigator{
    /**
     * Returns the encoded control.
     * <p/>
     * Low byte - speed of the forward movement. -100(backward max.) to 100(forward max.)
     * <p/>
     * High byte - speed of the turning. -100(turn left max.) to 100(turn right max.)
     *
     * @return encoded control
     */
    @Override
    public short getControl() {
        return 0;
    }

    /**
     * Notifies the navigator that som obstacle is detected or not detected
     *
     * @param isObstacle true if an obstacle is detected, false otherwise
     */
    @Override
    public void obstacleDetected(boolean isObstacle) {
    }

    /**
     * Indicates that the robot can move without external control
     */
    @Override
    public void enableAutonomousDrive() {
    }
}
