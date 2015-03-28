package segway.utils;

/**
 * https://en.wikipedia.org/wiki/Standard_deviation
 * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
 * @author Max Morozov
 */
public class DeviationCalc {
    private float average = 0;
    private float variance = 0;
    private int count = 0;

    public void add(float value) {
        ++count;
        float prior_average = average;
        average = average + (value - average) / count;
        variance = variance + (value - prior_average)*(value - average);
    }

    public float getAverage() {
        return average;
    }

    public float getDeviation() {
        return (float) Math.sqrt(variance / count);
    }
}
