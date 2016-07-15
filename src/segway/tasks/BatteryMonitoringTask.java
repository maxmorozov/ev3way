package segway.tasks;

import segway.Constants;
import segway.utils.SharedState;
import segway.Navigator;

/**
 * Task to monitor battery voltage
 *
 * @author Max Morozov
 */
public class BatteryMonitoringTask implements Runnable {
    private final SharedState state;
    private final Navigator navigator;

    public BatteryMonitoringTask(SharedState state, Navigator navigator) {
        this.state = state;
        this.navigator = navigator;
    }

    /**
     * Performs time checking and battery voltage averaging
     *
     * @see Thread#run()
     */
    @Override
    public void run() {
        state.updateBatteryVoltage();

        long workingTime = System.currentTimeMillis() - state.getStartTime();

        if (workingTime >= Constants.INITIAL_BALANCING_PERIOD) {
            navigator.enableAutonomousDrive();
        }
    }
}
