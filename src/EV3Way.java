import impl.SharedState;
import lejos.hardware.Button;
import lejos.hardware.Key;
import lejos.hardware.KeyListener;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.HiTechnicAccelerometer;
import lejos.hardware.sensor.HiTechnicGyro;
import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.utility.TextMenu;
import navigation.Navigator;
import navigation.impl.AutonomousNavigator;
import navigation.impl.BTNavigator;
import navigation.impl.SteadyStateNavigator;
import tasks.AccelerationReadTask;
import tasks.BatteryMonitoringTask;
import tasks.ControllerTask;
import tasks.ObstacleDetectionTask;

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
    }

    public EV3Way() {
        HiTechnicGyro gyro = new HiTechnicGyro(SensorPort.S1);
        NXTUltrasonicSensor ultrasonicSensor = new NXTUltrasonicSensor(SensorPort.S2);
        HiTechnicAccelerometer accelSensor = new HiTechnicAccelerometer(SensorPort.S4);

        UnregulatedMotor leftMotor = new UnregulatedMotor(MotorPort.C);
        UnregulatedMotor rightMotor = new UnregulatedMotor(MotorPort.B);

        SharedState state = new SharedState();
        Navigator navigator = getNavigator();

        Runnable stopper = new Runnable() {
            @Override
            public void run() {
                stop();
            }
        };

        tasks[0] = new Task(4, new ControllerTask(gyro.getRateMode(), leftMotor, rightMotor, navigator, state, stopper), Thread.MAX_PRIORITY);
        tasks[1] = new Task(20, new ObstacleDetectionTask(ultrasonicSensor, state, navigator));
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

/*
    private void waitTasks() throws InterruptedException {
        for (Task task : tasks) {
            task.join();
        }
    }
*/
}
