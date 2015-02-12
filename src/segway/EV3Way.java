package segway;

import segway.estimators.KalmanEstimator;
import segway.utils.SharedState;
import lejos.hardware.Button;
import lejos.hardware.Key;
import lejos.hardware.KeyListener;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.HiTechnicAccelerometer;
import lejos.utility.TextMenu;
import segway.estimators.LowPassFilterEstimator;
import segway.navigation.AutonomousNavigator;
import segway.navigation.BTNavigator;
import segway.navigation.SteadyStateNavigator;
import segway.sensors.EV3IRSensor;
import segway.sensors.HiTechnicGyro;
import segway.tasks.*;

public class EV3Way {
    private Task[] tasks = new Task[4];


    public static void main(String[] args) {
        //Provide a way to interrupt the program
        Button.ESCAPE.addKeyListener(new KeyListener() {
            @Override
            public void keyPressed(Key k) {
                System.exit(0);
            }

            @Override
            public void keyReleased(Key k) {

            }
        });

        final EV3Way app = new EV3Way();


        app.start();

        //app.monitor();
    }

    public EV3Way() {
//        MindsensorsAbsoluteIMU imu = new MindsensorsAbsoluteIMU(SensorPort.S2);
        HiTechnicGyro gyro = new HiTechnicGyro(SensorPort.S1);
        HiTechnicAccelerometer accelSensor = new HiTechnicAccelerometer(SensorPort.S4);
        EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S3);

        UnregulatedMotor rightMotor = new UnregulatedMotor(MotorPort.A);
        UnregulatedMotor leftMotor = new UnregulatedMotor(MotorPort.D);

        SharedState state = new SharedState();
        Navigator navigator = getNavigator();

        Runnable stopper = new Runnable() {
            @Override
            public void run() {
                stop();
            }
        };

        StateVariablesEstimator estimator = getEstimator(state);
        tasks[0] = new Task(Constants.CONTROLLER_TIME, new ControllerTask(gyro.getRateMode(), leftMotor, rightMotor, navigator, state, estimator, stopper), Thread.MAX_PRIORITY);
        tasks[1] = new Task(20, new ObstacleDetectionTask(irSensor.getDistanceMode(), state, navigator));
        tasks[2] = new Task(100, new BatteryMonitoringTask(state, navigator));
        tasks[3] = new Task(12, new AccelerationReadTask(accelSensor, state));
    }

    /**
     * Selects the appropriate navigator
     *
     * @return navigator instance bn
     */
    private Navigator getNavigator() {
        String[] navigatorItems = {"Steady", "Remote", "Auto", "Exit"};
        TextMenu main = new TextMenu(navigatorItems, 1, "Navigation");
        LCD.clear();
        int selection = main.select();
        LCD.clear();

        switch (selection) {
            case 0:
                return new SteadyStateNavigator();
            case 1:
                return new BTNavigator();
            case 2:
                return new AutonomousNavigator();
        }
        System.exit(0);
        return null;
    }

    /**
     * Selects the appropriate navigator
     *
     * @return navigator instance bn
     */
    private StateVariablesEstimator getEstimator(SharedState state) {
        String[] navigatorItems = {"Low Pass Filter", "Kalman Filter", "Exit"};
        TextMenu main = new TextMenu(navigatorItems, 1, "Estimator");
        LCD.clear();
        int selection = main.select();
        LCD.clear();

        switch (selection) {
            case 0:
                return new LowPassFilterEstimator();
            case 1:
                return new KalmanEstimator(state);
        }
        System.exit(0);
        return null;
    }

    public void start() {
        for (Task task : tasks) {
            task.start();
        }
    }

    public void stop() {
        for (Task task : tasks) {
            task.finish();
        }
    }
}
