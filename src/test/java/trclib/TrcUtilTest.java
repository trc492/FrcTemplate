package trclib;

import org.junit.Test;

import static org.junit.Assert.*;

public class TrcUtilTest
{
    @Test
    public void inRangeTest()
    {
        assertFalse(TrcUtil.inRange(3, 1, 2));
        assertTrue(TrcUtil.inRange(5, 1, 1e10));
        assertFalse(TrcUtil.inRange(25, 60, 1));
        assertTrue(TrcUtil.inRange(-1, -5, 20));
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
        assertEquals(100, TrcUtil.maxMagnitude(0.0, 3.5, 100.0), 1e-9);
        assertEquals(1, TrcUtil.maxMagnitude(0.0, 1.0, 1.0), 1e-9);
        assertEquals(1, TrcUtil.maxMagnitude(1.0), 1e-9);
        assertEquals(3, TrcUtil.maxMagnitude(-2.0, -1.0, -3.0), 1e-9);
    }

    @Test
    public void averageTest()
    {
        assertEquals(0, TrcUtil.average(), 1e-9);
        assertEquals(0, TrcUtil.average(0.0), 1e-9);
        assertEquals(4, TrcUtil.average(3.0, 5.0), 1e-9);
        assertEquals(-2, TrcUtil.average(-1.0, -3.0), 1e-9);
        assertEquals(3, TrcUtil.average(3.0, 3.0, 3.0, 3.0, 3.0), 1e-9);
    }
}