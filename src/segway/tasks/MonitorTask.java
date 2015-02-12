package segway.tasks;

import segway.utils.DeviationCalc;

/**
 * @author Max Morozov
 */
public class MonitorTask implements Runnable {
    private final Runnable subtask;

    private volatile boolean reset = false;

    private DeviationCalc devCalc = new DeviationCalc();
    private DeviationCalc taskCalc = new DeviationCalc();

    long prior_time = System.nanoTime();

    public MonitorTask(Runnable subtask) {
        this.subtask = subtask;
        reset = true;
    }

    @Override
    public void run() {
        if (reset) {
            devCalc = new DeviationCalc();
            taskCalc = new DeviationCalc();
            prior_time = System.nanoTime();
            reset = false;
        } else {
            long time = System.nanoTime();
            devCalc.add((time-prior_time) / 1e6f);
            prior_time = time;
        }

        long t1 = System.nanoTime();
        subtask.run();
        long t2 = System.nanoTime();
        taskCalc.add((t2-t1)/1e6f);
    }

    public float getAveragePeriod() {
        return devCalc.getAverage();
    }

    public float getPeriodDeviation() {
        return devCalc.getDeviation();
    }

    public float getAverageTask() {
        return taskCalc.getAverage();
    }

    public float getTaskDeviation() {
        return taskCalc.getDeviation();
    }

    public void reset() {
        reset = true;
    }
}
