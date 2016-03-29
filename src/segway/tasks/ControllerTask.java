package segway.tasks;

import segway.StateVariablesEstimator;
import segway.controller.BalanceController;
import segway.Constants;
import segway.utils.SharedState;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.DCMotor;
import lejos.robotics.EncoderMotor;
import segway.Navigator;

/**
 * @author Max Morozov
 */
public class ControllerTask implements Runnable {
    private State currentState = State.Init;

    private final SharedState shared;
    private final Navigator navigator;
    private final BalanceController controller = new BalanceController();

    private final SensorMode imu;
    private final EncoderMotor leftMotor;
    private final EncoderMotor rightMotor;

    private final StateVariablesEstimator estimator;

    //Gyro offset
    private float gyroOffset = 0;
    private int avgCount = 0;
    private float gyroMax, gyroMin;
    private float gyroValue = 0;

    //Prepare to balance
    //1 sec = 250 periods
    private static final int BEEP_INTERVAL = 1000 / Constants.CONTROLLER_TIME;
    private int countDown = BEEP_INTERVAL * 6; //6 seconds

    static final float EXEC_PERIOD = Constants.CONTROLLER_TIME / 1000f;

    private final float[] sample;

    private static final boolean FILTER_GYRO = false;

    //private long calibrationEndTime;

    //the object to stop the program when the robot fell
    private final Runnable stopper;

    public ControllerTask(SensorMode imu, EncoderMotor leftMotor, EncoderMotor rightMotor, Navigator navigator, SharedState shared, StateVariablesEstimator estimator, Runnable stopper) {
        this.imu = imu;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.navigator = navigator;
        this.shared = shared;
        this.estimator = estimator;
        this.stopper = stopper;
        this.sample = new float[imu.sampleSize()];
    }

    /**
     * Performs balance and drive control.
     *
     * @see Thread#run()
     */
    @Override
    public void run() {
        switch (currentState) {
            case Init:
                // Ensure that the motor controller is active since this affects the gyro values.
                leftMotor.stop();
                rightMotor.stop();

                leftMotor.resetTachoCount();
                rightMotor.resetTachoCount();
                gyroMax = -1000;
                gyroMin = 1000;
                gyroOffset = 0;
                avgCount = 0;

                currentState = State.Calibrating;
                displayCalibrating();
                break;

            case Calibrating: {
                //calibrate using low pass filter
                //gyroOffset = gyroOffset * Constants.GYRO_CALIBRATION_FILTER + (1 - Constants.GYRO_CALIBRATION_FILTER) * gyro.readRate();

                //calibrate using average value of gyro
                float gyroValue = readRate();

                gyroOffset += gyroValue;
                ++avgCount;
                gyroMax = Math.max(gyroMax, gyroValue);
                gyroMin = Math.min(gyroMin, gyroValue);

                if (avgCount == Constants.GYRO_OFFSET_SAMPLES) {
                    if ((gyroMax - gyroMin) > 2) {
                        currentState = State.Init;
                    } else {
                        gyroOffset /= avgCount;
                        estimator.init(gyroOffset);

                        currentState = State.Prepare;
                        displayBalanceWarning();
                    }
                }
                break;
            }

            case Prepare:
                // Play warning beep sequence each second before balance starts
                if ((countDown % BEEP_INTERVAL) == 0) {
                    LCD.drawInt(countDown / BEEP_INTERVAL, 5, 4);
                    Sound.playTone(440, 100);
                }
                if (--countDown == 0) {
                    LCD.clear();
                    currentState = State.Control;
                    shared.setBalancing(true);
                }
                break;

            case Control: {
                short navigation = navigator.getControl();
                byte cmd_forward = (byte) (navigation & 0xff);
                byte cmd_turn = (byte) ((navigation >> 8) & 0xff);

                readState();

                short result = controller.control(
                        cmd_forward,
                        cmd_turn,
                        estimator.getAngularVelocity(),
                        estimator.getAngle(),
                        leftMotor.getTachoCount(),
                        rightMotor.getTachoCount(),
                        shared.getBatteryVoltage());

                //Check if the robot has fallen
                if (!controller.isOk()) {
                    Sound.beepSequenceUp();
                    LCD.drawString("Oops... I fell", 0, 4);

                    leftMotor.flt();
                    rightMotor.flt();

                    stopper.run();

                    result = 0;
                }
                byte left_motor_power = (byte) (result & 0xFF);
                byte right_motor_power = (byte) ((result >> 8) & 0xFF);

                controlMotor(leftMotor, left_motor_power);
                controlMotor(rightMotor, right_motor_power);
            }
            break;

            default:
                leftMotor.stop();
                rightMotor.stop();

        }
    }

    private float readRate() {
        imu.fetchSample(sample, 0);
        if (FILTER_GYRO)
            gyroValue = Constants.GYRO_FILTER * gyroValue + (1 - Constants.GYRO_FILTER) * sample[5];
        else
            gyroValue = sample[5];
        return gyroValue;
    }

    private void readState() {
        imu.fetchSample(sample, 0);
        if (FILTER_GYRO)
            gyroValue = Constants.GYRO_FILTER * gyroValue + (1 - Constants.GYRO_FILTER) * sample[5];
        else
            gyroValue = sample[5];

        float angle = (float) Math.atan2(-sample[1], sample[0]) * Constants.RAD_TO_DEGREE;
        estimator.updateState(gyroValue, angle, EXEC_PERIOD);
    }

    private static void displayCalibrating() {
        LCD.clear();
        LCD.drawString("leJOS EV3Way", 0, 1);

        LCD.drawString("Lay robot down", 0, 4);
        LCD.drawString("flat to get gyro", 0, 5);
        LCD.drawString("offset.", 0, 6);
    }

    /**
     * Warn user the NXJWay is about to start balancing.
     */
    private static void displayBalanceWarning() {
        LCD.clear();
        LCD.drawString("leJOS EV3Way", 0, 1);
        LCD.drawString("Balance in", 0, 3);
    }

    private static void controlMotor(DCMotor motor, byte power) {
        motor.setPower(Math.abs(power));
        if (power > 0)
            motor.forward();
        else
            motor.backward();
    }

    private enum State {
        Init,
        Calibrating,
        Prepare,
        Control
    }
}
