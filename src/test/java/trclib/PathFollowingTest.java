package trclib;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import trclib.simulator.SimulatedHolonomicDrivebase;
import trclib.simulator.Simulator;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

public class PathFollowingTest
{
    private SimulatedHolonomicDrivebase driveBase;
    private Simulator s;
    private TrcPidDrive pidDrive;
    private TrcHolonomicPurePursuitDrive purePursuit;
    private TrcPidController xPid, yPid, turnPid;
    private double tolerance;
    private int turnTolerance;

    @Before
    public void setup()
    {
        driveBase = new SimulatedHolonomicDrivebase(5, 10, 6, 10);
        driveBase.setOdometryScales(1, 1, 1.0 / 0.5);
        driveBase.setOdometryEnabled(true);
        s = new Simulator(20, 20, 40, 1, driveBase);

        xPid = new TrcPidController("xPid", new TrcPidController.PidCoefficients(1, 0, 0.05), 0.03,
            driveBase::getXPosition);
        yPid = new TrcPidController("yPid", new TrcPidController.PidCoefficients(1, 0, 0.05), 0.03,
            driveBase::getYPosition);
        turnPid = new TrcPidController("turnPid", new TrcPidController.PidCoefficients(0.01, 0, 0.01), 3,
            driveBase::getHeading);
        pidDrive = new TrcPidDrive("pidDrive", driveBase, xPid, yPid, turnPid);

        tolerance = 0.05;
        turnTolerance = 3;
        purePursuit = new TrcHolonomicPurePursuitDrive("", driveBase, 0.3, tolerance, turnTolerance,
            new TrcPidController.PidCoefficients(1), new TrcPidController.PidCoefficients(0.01, 0, 0.01),
            new TrcPidController.PidCoefficients(0.02, 0, 0, 1 / 5.0));
        purePursuit.setMoveOutputLimit(0.6);
    }

    @After
    public void tearDown()
    {
        s.stop();
    }

    private void assertTimeout(TrcEvent event, long millis)
    {
        AtomicBoolean done = new AtomicBoolean(false);
        Thread testThread = Thread.currentThread();
        Thread t = new Thread(() -> {
            if (s.blockForEvent(event))
            {
                done.set(true);
                testThread.interrupt();
            }
        });
        t.start();
        try
        {
            Thread.sleep(millis);
            t.interrupt();
            if (!done.get())
            {
                fail("Timed out!");
            }
        }
        catch (InterruptedException ignored)
        {
        }
    }

    private void assertPosition(TrcPose2D curr, TrcPose2D position)
    {
        assertEquals(position.x, curr.x, tolerance);
        assertEquals(position.y, curr.y, tolerance);
        assertEquals(position.angle, curr.angle, turnTolerance);
    }

    @Test
    public void pidDriveTest()
    {
        TrcPose2D[][] poses = new TrcPose2D[][] { { new TrcPose2D(0, 0), null }, { new TrcPose2D(3, 3), null },
            { new TrcPose2D(0, 3), null }, { new TrcPose2D(0, 0), null } };
        TrcPath path = new TrcPath(
            Arrays.stream(poses).map(p -> new TrcWaypoint(p[0], p[1])).toArray(TrcWaypoint[]::new));
        s.addPath(path);
        s.start();

        xPid.setNoOscillation(true);
        yPid.setNoOscillation(true);

        TrcEvent event = new TrcEvent("");
        pidDrive.setRelativeTarget(3, 3, 0, event);
        assertTimeout(event, 4000);
        pidDrive.setRelativeTarget(-3, 0, 0, event);
        assertTimeout(event, 4000);
        xPid.setNoOscillation(false);
        yPid.setNoOscillation(false);
        pidDrive.setRelativeTarget(0, -3, 0, event);
        System.out.println(pidDrive.getAbsoluteTargetPose());
        assertTimeout(event, 4000);
        assertPosition(new TrcPose2D(), new TrcPose2D());
    }

    @Test
    public void purePursuitTest()
    {
        TrcPose2D[][] poses = new TrcPose2D[][] { { new TrcPose2D(0, 0), null },
            { new TrcPose2D(2, 2, 90), new TrcPose2D(3, 3) }, { new TrcPose2D(2, 5, 90), null } };
        TrcPath path = new TrcPath(
            Arrays.stream(poses).map(p -> new TrcWaypoint(p[0], p[1])).toArray(TrcWaypoint[]::new));
        s.addPath(path);
        s.start();

        TrcEvent event = new TrcEvent("");
        TrcPose2D ref = driveBase.getFieldPosition();
        purePursuit.start(path, event, 0);

        assertTimeout(event, 5000);

        TrcPose2D end = poses[poses.length - 1][0];
        assertPosition(driveBase.getFieldPosition().relativeTo(ref), end);
    }
}
