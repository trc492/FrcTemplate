package team492;

import org.junit.Before;
import org.junit.Test;
import trclib.PathFollowingTestBase;
import trclib.TrcEvent;
import trclib.TrcHolonomicPurePursuitDrive;
import trclib.TrcPath;
import trclib.TrcPidController;
import trclib.TrcPose2D;
import trclib.TrcWaypoint;

import java.util.Arrays;

import static org.junit.Assert.*;

public class CmdShooterAutoTest extends PathFollowingTestBase
{
    private CmdShooterAuto auto;
    private TrcHolonomicPurePursuitDrive purePursuit;

    @Before
    public void setup()
    {
        super.setup();
        auto = new CmdShooterAuto(null);

        purePursuit = new TrcHolonomicPurePursuitDrive("", driveBase, 12, tolerance, turnTolerance,
            new TrcPidController.PidCoefficients(1, 0, 1), new TrcPidController.PidCoefficients(0.01, 0, 0.01),
            new TrcPidController.PidCoefficients(0.001, 0, 0, 1 / 100.0));
        purePursuit.setMoveOutputLimit(0.6);
    }

    @Test
    public void wallStartTest()
    {
        auto.start(0, FrcAuto.StartPosition.RIGHT_WALL, CmdShooterAuto.AfterAction.INTAKE_AND_SHOOT);
        testAuto(new TrcPose2D(FrcAuto.StartPosition.RIGHT_WALL.getXPos(), -RobotInfo.ROBOT_LENGTH / 2));
    }

    @Test
    public void feederStartTest()
    {
        auto.start(0, FrcAuto.StartPosition.LEFT_BUMPER_FEEDER, CmdShooterAuto.AfterAction.INTAKE_AND_SHOOT);
        testAuto(new TrcPose2D(FrcAuto.StartPosition.LEFT_BUMPER_FEEDER.getXPos(), 0));
    }

    @Test
    public void goalCenteredStartTest()
    {
        auto.start(0, FrcAuto.StartPosition.IN_VISION, CmdShooterAuto.AfterAction.INTAKE_AND_SHOOT);
        testAuto(new TrcPose2D(FrcAuto.StartPosition.IN_VISION.getXPos(), 0), false, true);
    }

    private void testAuto(TrcPose2D start)
    {
        testAuto(start, true, true);
    }

    private void testAuto(TrcPose2D start, boolean shootPath, boolean pickupPath)
    {
        driveBase.setFieldPosition(inSimulatorReferenceFrame(start));
        boolean started = false;
        if (shootPath)
        {
            TrcPath path = auto.createToShootPath(start);
            TrcPath displayPath = new TrcPath(Arrays.stream(path.getAllWaypoints()).map(w -> {
                TrcWaypoint wp = new TrcWaypoint(w);
                TrcPose2D origin = new TrcPose2D().relativeTo(start);
                TrcPose2D pose = new TrcPose2D(w.x, w.y, w.heading).relativeTo(origin);
                pose = inSimulatorReferenceFrame(pose);
                wp.x = pose.x;
                wp.y = pose.y;
                return wp;
            }).toArray(TrcWaypoint[]::new));
            s.addPath(displayPath);
            started = true;
            s.start();

            TrcEvent event = new TrcEvent("");
            purePursuit.start(path, event, 0);

            assertTimeout(event, 10000);
        }

        if (pickupPath)
        {
            TrcPose2D newStart = driveBase.getFieldPosition().relativeTo(new TrcPose2D(-RobotInfo.FIELD_WIDTH / 2,
                -RobotInfo.INITIATION_LINE_TO_ALLIANCE_WALL + RobotInfo.FIELD_LENGTH / 2));
            TrcPath path = auto.createPickupPath(newStart);
            TrcPath displayPath = new TrcPath(Arrays.stream(path.getAllWaypoints()).map(w -> {
                TrcWaypoint wp = new TrcWaypoint(w);
                TrcPose2D origin = new TrcPose2D().relativeTo(newStart);
                TrcPose2D pose = new TrcPose2D(w.x, w.y, w.heading).relativeTo(origin);
                pose = inSimulatorReferenceFrame(pose);
                wp.x = pose.x;
                wp.y = pose.y;
                return wp;
            }).toArray(TrcWaypoint[]::new));
            s.addPath(displayPath);
            if (!started)
                s.start();

            TrcEvent event = new TrcEvent("");
            purePursuit.start(path, event, 0);

            assertTimeout(event, 10000);
        }

    }

    private TrcPose2D inSimulatorReferenceFrame(TrcPose2D pose2D)
    {
        return pose2D.relativeTo(new TrcPose2D(RobotInfo.FIELD_WIDTH / 2,
            RobotInfo.INITIATION_LINE_TO_ALLIANCE_WALL - RobotInfo.FIELD_LENGTH / 2));
    }
}