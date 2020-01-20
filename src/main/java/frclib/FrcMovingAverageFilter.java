package frclib;

import edu.wpi.first.wpilibj.LinearFilter;
import trclib.TrcFilter;

public class FrcMovingAverageFilter extends TrcFilter
{
    private final LinearFilter filter;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    protected FrcMovingAverageFilter(String instanceName, int numPoints)
    {
        super(instanceName);
        filter = LinearFilter.movingAverage(numPoints);
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
