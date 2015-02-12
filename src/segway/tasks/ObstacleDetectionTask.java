package segway.tasks;

import segway.Constants;
import segway.utils.SharedState;
import lejos.robotics.SampleProvider;
import segway.Navigator;

/**
 * @author Max Morozov
 */
public class ObstacleDetectionTask implements Runnable {
    private final SampleProvider sensor;
    private final SharedState shared;
    private final Navigator navigator;

    private final float[] sample;

    public ObstacleDetectionTask(SampleProvider sensor, SharedState shared, Navigator navigator) {
        this.sensor = sensor;
        this.shared = shared;
        this.navigator = navigator;
        this.sample = new float[sensor.sampleSize()];
    }

    /**
     * When an object implementing interface <code>Runnable</code> is used
     * to create a thread, starting the thread causes the object's
     * <code>run</code> method to be called in that separately executing
     * thread.
     * <p/>
     * The general contract of the method <code>run</code> is that it may
     * take any action whatsoever.
     *
     * @see Thread#run()
     */
    @Override
    public void run() {
        if (shared.isBalancing()) {
            sensor.fetchSample(sample, 0);
            navigator.obstacleDetected(sample[0] < Constants.MIN_DISTANCE);
        }
    }
}
