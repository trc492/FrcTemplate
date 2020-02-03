package trclib;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.Arrays;

public class TrcPathTest
{
    @Test
    public void trapezoidVelocityTest()
    {
        TrcPose2D[] poses = new TrcPose2D[] { new TrcPose2D(), new TrcPose2D(0, 0.5), new TrcPose2D(0, 5),
            new TrcPose2D(-5, 10) };
        TrcPath path = new TrcPath(
            Arrays.stream(poses).map(pose -> new TrcWaypoint(pose, null)).toArray(TrcWaypoint[]::new));
        testTrapezoid(path, 10, 50);
        testTrapezoid(path, 50, 10);
    }

    private void testTrapezoid(TrcPath path, double maxVel, double maxAccel)
    {
        TrcPath trapezoid = path.trapezoidVelocity(maxVel, maxAccel);
        // up to two points inserted
        assertTrue(TrcUtil.inRange(trapezoid.getSize(), path.getSize(), path.getSize() + 2, true));
        for (int i = 0; i < trapezoid.getSize(); i++)
        {
            TrcWaypoint from = trapezoid.getWaypoint(0);
            TrcWaypoint to = trapezoid.getWaypoint(1);
            assertEquals(from.distanceTo(to), TrcUtil.average(from.velocity, to.velocity) * from.timeStep, 1e-3);
            assertEquals(from.acceleration, Math.abs(to.velocity - from.velocity) / from.timeStep, 1e-3);
        }
        // average velocity must be less than the maximum velocity
        assertTrue(trapezoid.getArcLength() / trapezoid.getPathDuration() < maxVel);
        // The first and last points must have zero velocity
        assertEquals(0, trapezoid.getWaypoint(0).velocity, 1e-6);
        assertEquals(0, trapezoid.getWaypoint(trapezoid.getSize() - 1).velocity, 1e-6);
    }

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