package frclib;

import edu.wpi.first.wpilibj.MedianFilter;
import trclib.TrcFilter;

public class FrcMedianFilter extends TrcFilter
{
    private final MedianFilter filter;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FrcMedianFilter(String instanceName, int numSamples)
    {
        super(instanceName);
        filter = new MedianFilter(numSamples);
    }

    @Override
    public double filterData(double data)
    {
        return filter.calculate(data);
    }

    @Override
    public void reset()
    {
        filter.reset();
    }
}
