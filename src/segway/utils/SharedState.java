package segway.utils;

import lejos.hardware.Battery;
import segway.Constants;

/**
 * @author Max Morozov
 */
public class SharedState {
    private volatile float batteryVoltage;

    /**
     * Time when balancing was started
     */
    private long startTime = 0;

    /**
     * Indicates that the robot is balancing now
     */
    private boolean isBalancing = false;

    public SharedState() {
        batteryVoltage = Battery.getVoltageMilliVolt();
    }

    public float getBatteryVoltage() {
        return batteryVoltage;
    }

    /**
     * Updates the current battery voltage using Low-Pass filter to reduce noise
     */
    public void updateBatteryVoltage() {
        batteryVoltage = batteryVoltage * Constants.BATTERY_FILTER + (1 - Constants.BATTERY_FILTER) * Battery.getVoltageMilliVolt();
    }

    public long getStartTime() {
        if (isBalancing)
            return startTime;
        else
            return System.currentTimeMillis();
    }

    public boolean isBalancing() {
        return isBalancing;
    }

    public void setBalancing(boolean balancing) {
        startTime = System.currentTimeMillis();
        isBalancing = balancing;
    }
}
