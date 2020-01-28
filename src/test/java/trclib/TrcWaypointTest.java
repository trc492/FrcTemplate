package trclib;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class TrcWaypointTest
{
    @Test
    public void distanceToTest()
    {
        TrcWaypoint p1 = new TrcWaypoint(1, 0, 0, 1, 1, 1, 1, 1);
        TrcWaypoint p2 = new TrcWaypoint(2, 3, 4, 2, 2, 2, 2, 2);
        assertEquals(5.0, p1.distanceTo(p2), 1e-6);
        TrcWaypoint p3 = new TrcWaypoint(3, 5, 12, 3, 3, 3, 3, 3);
        assertEquals(13.0, p1.distanceTo(p3), 1e-6);
        TrcWaypoint p4 = new TrcWaypoint(4, 8, 15, 4, 4, 4, 4, 4);
        assertEquals(17.0, p1.distanceTo(p4), 1e-6);
        TrcWaypoint p5 = new TrcWaypoint(5, 7, 24, 5, 5, 5, 5, 5);
        assertEquals(25.0, p1.distanceTo(p5), 1e-6);
        TrcWaypoint p6 = new TrcWaypoint(6, 20, 21, 6, 6, 6, 6, 6);
        assertEquals(29.0, p1.distanceTo(p6), 1e-6);
        TrcWaypoint p7 = new TrcWaypoint(7, 12, 35, 7, 7, 7, 7, 7);
        assertEquals(37.0, p1.distanceTo(p7), 1e-6);
    }
}
