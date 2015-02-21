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
public class EstimatorLog {
    private static final List<State> states = new ArrayList<>(3000);

    public static class State {
        public long time;
        public float gyroValue;
        public float angleValue;

        public float lpfGyro;
        public float lpfAngle;
        public float lpgGyroOffset;

        public float kalmanGyro;
        public float kalmanAngle;
        public float kalmanGyroOffset;
    }


    public static void add(State state) {
        states.add(state);
    }

    public static int size() {
        return states.size();
    }

    public static void print() {
        try {
            FileOutputStream os = new FileOutputStream("ev3way_estimator.log");
            try {
                PrintStream ps = new PrintStream(os);
                for (State st: states) {
                    ps.println(String.format("%d,%f,%f,%f,%f,%f,%f,%f,%f",
                            st.time, st.gyroValue, st.angleValue,
                            st.lpfGyro, st.lpfAngle, st.lpgGyroOffset,
                            st.kalmanGyro, st.kalmanAngle, st.kalmanGyroOffset));
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
