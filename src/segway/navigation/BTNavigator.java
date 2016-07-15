package segway.navigation;

import lejos.hardware.Bluetooth;
import lejos.hardware.lcd.LCD;
import lejos.remote.nxt.NXTConnection;
import segway.Navigator;

/**
 * Navigator that uses remote controler via Bluetooth.
 * <p/>
 * It has not been tested on EV3 yet.
 *
 * @author Max Morozov
 */
public class BTNavigator implements Navigator {
    private static final int BT_RCV_BUF_SIZE = 32;

    private final byte[] buffer = new byte[BT_RCV_BUF_SIZE];
    private NXTConnection connection;

    private volatile boolean isObstacle = false;
    private volatile boolean isAutonomous = false;

    public BTNavigator() {
        LCD.clearDisplay();
        LCD.drawString("Connecting", 0, 0);
        connection = Bluetooth.getNXTCommConnector().waitForConnection(0, NXTConnection.PACKET);
        LCD.clearDisplay();
    }

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
        if (connection.read(buffer, BT_RCV_BUF_SIZE) >= 0) {
            /*
                * R/C command from NXT GamePad
                * buf[0]: -100(forward max.) to 100(backward max.)
                * buf[1]: -100(turn left max.) to 100(turn right max.)
                */
            byte cmd_forward = (byte) -normalize(buffer[0]); /* reverse the direction */
            byte cmd_turn = normalize(buffer[1]);
            if (isObstacle) {
                /* make NXJway move backward to avoid obstacle */
                cmd_forward = -100;
                cmd_turn = 0;
            }

            return (short) ((cmd_forward & 0xFF) | ((cmd_turn & 0xFF) << 8));
        }
        return 0;
    }

    private byte normalize(byte command) {
        if (command == -1)
            return 0;
        else
            return command;
    }

    /**
     * Notifies the navigator that som obstacle is detected or not detected
     *
     * @param isObstacle true if an obstacle is detected, false otherwise
     */
    @Override
    public void obstacleDetected(boolean isObstacle) {
        this.isObstacle = isObstacle && isAutonomous;
    }

    /**
     * Indicates that the robot can move without external control
     */
    @Override
    public void enableAutonomousDrive() {
        isAutonomous = true;
    }
}
