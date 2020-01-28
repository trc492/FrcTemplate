package trclib;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

//import trclib.TrcUtil;

public class TrcUtilTest 
{
    
    @Test
    public void inRangeTest() 
    {
        assertEquals(false, TrcUtil.inRange(3, 1, 2));
        assertEquals(true, TrcUtil.inRange(5, 1, 1e10));
        assertEquals(false, TrcUtil.inRange(25, 60, 1));
        assertEquals(true, TrcUtil.inRange(-1, -5, 20));    
    }

    @Test
    public void roundTest()
    {
        assertEquals(1, TrcUtil.round(1.1));
        assertEquals(0, TrcUtil.round(0.0));
        assertEquals(365, TrcUtil.round(365.49));
        assertEquals(50, TrcUtil.round(49.999));
    }

    @Test
    public void maxMagnitudeTest()
    {
        assertEquals(100, TrcUtil.maxMagnitude(new double[] {0.0, 3.5, 100.0}), 1e-9);
        assertEquals(1, TrcUtil.maxMagnitude(new double[] {0.0, 1.0, 1.0}), 1e-9);
        assertEquals(1, TrcUtil.maxMagnitude(new double[] {1.0}), 1e-9);
        assertEquals(3, TrcUtil.maxMagnitude(new double[] {-2.0, -1.0, -3.0}), 1e-9);
    }

    @Test
    public void averageTest()
    {
        assertEquals(0, TrcUtil.average(), 1e-9);
        assertEquals(0, TrcUtil.average(new double[] {0.0}), 1e-9);
        assertEquals(4, TrcUtil.average(new double[] {3.0, 5.0}), 1e-9);
        assertEquals(-2, TrcUtil.average(new double[] {-1.0, -3.0}), 1e-9);
        assertEquals(3, TrcUtil.average(new double[] {3.0, 3.0, 3.0, 3.0, 3.0}), 1e-9);
    }
}