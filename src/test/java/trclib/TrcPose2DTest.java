package trclib;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

import trclib.TrcPose2D;

public class TrcPose2DTest
{
    private TrcPose2D test;

    @Before
    public void setup()
    {
        test = new TrcPose2D(0, 0, 0);
    }

    @Test
    public void testRelativeTo()
    {
        test = test.relativeTo(new TrcPose2D(0, -2, 0));
        assertEquals(0, test.x, 0);
        assertEquals(2, test.y, 0);
        assertEquals(0, test.angle, 0);
        test = test.relativeTo(new TrcPose2D(0, 0, 180));
        assertEquals(0, test.x, 0.1);
        assertEquals(-2, test.y, 0.1);
        assertEquals(-180, test.angle, 0);
        test = test.relativeTo(new TrcPose2D(123123123, 0, -90));
        assertEquals(-2, test.x, 0.1);
        assertEquals(123123123, test.y, 0.1);
        assertEquals(-90, test.angle, 0);
        test = test.relativeTo(new TrcPose2D(0, 123123123));
        assertEquals(-2, test.x, 0.1);
        assertEquals(0, test.y, 0.1);
        assertEquals(-90, test.angle, 0);
        test = test.relativeTo(new TrcPose2D(0, 0, -90), true);
        assertEquals(0, test.x, 0.1);
        assertEquals(2, test.y, 0.1);
        assertEquals(0, test.angle, 0);
        test = test.relativeTo(new TrcPose2D(0, 0, -90), false);
        assertEquals(2, test.x, 0.1);
        assertEquals(0, test.y, 0.1);
        assertEquals(0, test.angle, 0);
    }
}