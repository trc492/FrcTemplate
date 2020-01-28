package trclib;

import static org.junit.Assert.assertEquals;

//import org.junit.Before;
import org.junit.Test;

public class TrcPathTest
{
    @Test
    public void getSizeTest()
    {
        TrcWaypoint waypoint1 = new TrcWaypoint(1, 1, 1, 1, 1, 1, 1, 1);
        TrcWaypoint waypoint2 = new TrcWaypoint(2, 2, 2, 2, 2, 2, 2, 2);
        TrcPath path = new TrcPath(waypoint1, waypoint2);
        assertEquals(2, path.getSize());
    }

    @Test
    public void toRadiansTest()
    {
        TrcWaypoint waypointDeg1 = new TrcWaypoint(1, 1, 1, 1, 1, 1, 1, 90);
        TrcWaypoint waypointDeg2 = new TrcWaypoint(2, 2, 2, 2, 2, 2, 2, 180);
        TrcPath pathDeg = new TrcPath(waypointDeg1, waypointDeg2);
        TrcWaypoint waypointRad1 = new TrcWaypoint(1, 1, 1, 1, 1, 1, 1, Math.PI / 2);
        TrcWaypoint waypointRad2 = new TrcWaypoint(2, 2, 2, 2, 2, 2, 2, Math.PI);
        TrcPath pathRad = new TrcPath(false, waypointRad1, waypointRad2);
        assertEquals(pathRad.getWaypoint(0).heading, pathDeg.toRadians().getWaypoint(0).heading, 1e-6);
        assertEquals(pathRad.getWaypoint(1).heading, pathDeg.toRadians().getWaypoint(1).heading, 1e-6);
    }

    @Test
    public void toDegreesTest()
    {
        TrcWaypoint waypointRad1 = new TrcWaypoint(1, 1, 1, 1, 1, 1, 1, Math.PI / 2);
        TrcWaypoint waypointRad2 = new TrcWaypoint(2, 2, 2, 2, 2, 2, 2, Math.PI);
        TrcPath pathRad = new TrcPath(false, waypointRad1, waypointRad2);
        TrcWaypoint waypointDeg1 = new TrcWaypoint(1, 1, 1, 1, 1, 1, 1, 90);
        TrcWaypoint waypointDeg2 = new TrcWaypoint(2, 2, 2, 2, 2, 2, 2, 180);
        TrcPath pathDeg = new TrcPath(waypointDeg1, waypointDeg2);
        assertEquals(pathDeg.getWaypoint(0).heading, pathRad.toDegrees().getWaypoint(0).heading, 1e-6);
        assertEquals(pathDeg.getWaypoint(1).heading, pathRad.toDegrees().getWaypoint(1).heading, 1e-6);
    }
}