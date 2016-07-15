package segway.navigation;

import segway.Navigator;

/**
 * Navigator that allows the robot move about the room with obstacle avoidance.
 *
 * @author Max Morozov
 */
public class AutonomousNavigator implements Navigator {
    private volatile boolean enableMovement = false;
    private volatile boolean isObstacle = false;

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
        if (enableMovement) {
            byte cmd_forward, cmd_turn;
            if (isObstacle) {
                cmd_forward = 0;
                cmd_turn = 100;
            } else {
                cmd_forward = 100;
                cmd_turn = 0;
            }
            return (short) ((cmd_forward & 0xFF) | ((cmd_turn & 0xFF) << 8));
        }

        return 0;
    }

    /**
     * Notifies the navigator that som obstacle is detected or not detected
     *
     * @param isObstacle true if an obstacle is detected, false otherwise
     */
    @Override
    public void obstacleDetected(boolean isObstacle) {
        this.isObstacle = isObstacle;
    }

    /**
     * Indicates that the robot can move without external control
     */
    @Override
    public void enableAutonomousDrive() {
        enableMovement = true;
    }
}
