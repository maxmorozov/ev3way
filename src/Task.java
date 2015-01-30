/**
 * @author Max Morozov
 */
public class Task extends Thread {
    /**
     * Task activation period in ms
     */
    private final int period;
    /**
     * Task action that should be executed every {@link Task#period} milliseconds
     */
    private final Runnable taskAction;

    private volatile boolean stop = false;

    public Task(int period, Runnable taskAction) {
        this.period = period;
        this.taskAction = taskAction;
    }

    public Task(int period, Runnable taskAction, int priority) {
        this.period = period;
        this.taskAction = taskAction;
        setPriority(priority);
    }

    /**
     * Stops the task execution
     */
    public void finish() {
        stop = true;
    }

    @Override
    public void run() {
        //Using 'int' type for the end time increase the loop performance to 2 micro-seconds,
        //but limits the valid time range by INT_MAX.
        //The end time of type 'long' has limit UINT_MAX.
        long end = System.currentTimeMillis();
        while (!stop) {
            taskAction.run();

            end += period;
            try {
                Thread.sleep(end - System.currentTimeMillis());
            } catch (InterruptedException e) {
                break;
            }
        }
    }
}
