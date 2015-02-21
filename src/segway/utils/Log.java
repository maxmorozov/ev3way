package segway.utils;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

/**
 * @author Max Morozov
 */
public class Log {
    private static final List<State> states = new ArrayList<>(3000);

    public static class State {
        public long time;
        public float gyro_value;
        public float accel_angle;
        public float motor_pos;

        public float psi;
        public float psidot;
        public float theta;
        public float thetadot;

        public float theta_ref;
        public float thetadot_ref;

        public float voltage;
        public float volume;
        public float err_theta;
    }

    public static void add(State state) {
        states.add(state);
    }

    public static int size() {
        return states.size();
    }

    public static void print() {
        try {
            FileOutputStream os = new FileOutputStream("ev3way.log");
            try {
                PrintStream ps = new PrintStream(os);
                for (State st: states) {
                    ps.println(String.format("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                            st.time, st.gyro_value, st.accel_angle, st.motor_pos,
                            st.psi, st.psidot, st.theta, st.thetadot, st.theta_ref,
                            st.thetadot_ref, st.voltage, st.volume, st.err_theta));
                }
                os.close();

            } catch (IOException e) {
                e.printStackTrace();
            } finally {
                try {
                    os.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

    }
}
