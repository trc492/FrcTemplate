package frclib;

import edu.wpi.first.wpilibj.LinearFilter;
import trclib.TrcFilter;

public class FrcHighPassFilter extends TrcFilter
{
    private final LinearFilter filter;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    protected FrcHighPassFilter(String instanceName, double timeConstant, double dt)
    {
        super(instanceName);
        filter = LinearFilter.highPass(timeConstant, dt);
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
