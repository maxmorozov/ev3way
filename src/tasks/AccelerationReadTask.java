package tasks;

import impl.Constants;
import impl.SharedState;
import lejos.hardware.sensor.SensorMode;

/**
 * @author Max Morozov
 */
public class AccelerationReadTask implements Runnable {
    private final SharedState shared;

    private final SensorMode accelSensor;

    private final float[] accelValues;

    public AccelerationReadTask(SensorMode accelSensor, SharedState shared) {
        this.accelSensor = accelSensor;
        this.shared = shared;
        this.accelValues = new float[accelSensor.sampleSize()];
    }

    @Override
    public void run() {
        accelSensor.fetchSample(accelValues, 0);
        float angle = (float)Math.atan2(-accelValues[1], accelValues[0]) * Constants.RAD_TO_DEGREE;
//            float angle = -(float) Math.atan(accelValues[1] / accelValues[0]) * Constants.RAD_TO_DEGREE;
        shared.setBodyAngle(angle);
    }
}
