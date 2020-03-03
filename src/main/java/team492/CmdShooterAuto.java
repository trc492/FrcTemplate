package team492;

import frclib.FrcRemoteVisionProcessor;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcPath;
import trclib.TrcPose2D;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;
import trclib.TrcUtil;
import trclib.TrcWaypoint;

import java.util.Arrays;

public class CmdShooterAuto implements TrcRobot.RobotCommand
{
    private static final String instanceName = "CmdShooterAuto";
    private static final double relocalizationTimeout = 0.5;

    private enum State
    {
        DELAY, LOCALIZE, MOVE_TO_SHOOT, SHOOT, MOVE_OFF_LINE, PICKUP, MOVE_TO_SHOOT_2, SHOOT_2, WAIT, DONE
    }

    public enum AfterAction
    {
        NOTHING, INTAKE, INTAKE_AND_SHOOT
    }

    private Robot robot;
    private TrcStateMachine<State> sm;
    private TrcEvent event;
    private TrcTimer timer;
    private double delay;
    private FrcAuto.StartPosition startPosition;
    private AfterAction afterAction;
    private TrcDbgTrace dbgTrace;

    public CmdShooterAuto(Robot robot)
    {
        this.robot = robot;
        dbgTrace = TrcDbgTrace.getGlobalTracer();
        sm = new TrcStateMachine<>(instanceName + ".sm");
        event = new TrcEvent(instanceName + ".event");
        timer = new TrcTimer(instanceName + ".timer");

        if (robot != null)
        {
            HalDashboard.putNumber("RobotVel", 0);
        }
    }

    public void start(double delay, FrcAuto.StartPosition startPosition, AfterAction afterAction)
    {
        this.delay = delay;
        this.startPosition = startPosition;
        this.afterAction = afterAction;
        sm.start(State.DELAY);
        if (robot != null)
        {
            robot.intake.setSpacingDistance(6);
        }
        dbgTrace.traceInfo(instanceName + ".start", "Starting with options: delay=%.3f,startPosition=%s,afterAction=%s",
            delay, startPosition, afterAction);
    }

    private TrcPath createPath(TrcPose2D... poses)
    {
        return createPath(RobotInfo.ROBOT_MAX_REQ_SPEED, poses);
    }

    private TrcPath createPath(double maxVel, TrcPose2D... poses)
    {
        dbgTrace.traceInfo(instanceName + ".createPath", "Absolute path:");
        for (TrcPose2D pose : poses)
        {
            dbgTrace.traceInfo(instanceName + ".createPath", "\t%s", pose);
        }
        TrcPath path = new TrcPath(Arrays.stream(poses).map(p -> new TrcWaypoint(p.relativeTo(poses[0], false), null))
            .toArray(TrcWaypoint[]::new));
        TrcPath ret = path.trapezoidVelocity(maxVel, RobotInfo.ROBOT_MAX_ACCEL);
        dbgTrace.traceInfo(instanceName + ".createPath", "Relative path:");
        for (TrcWaypoint waypoint : ret.getAllWaypoints())
        {
            dbgTrace.traceInfo(instanceName + ".createPath", "\t%s", waypoint.toString());
        }
        return ret;
    }

    public TrcPath createToShootPath(TrcPose2D start)
    {
        dbgTrace.traceInfo(instanceName + ".createToShootPath", "[%.3f] Creating to shoot path with start=%s",
            TrcUtil.getModeElapsedTime(), start);
        if (startPosition != FrcAuto.StartPosition.RIGHT_WALL)
        {
            double targetY = -RobotInfo.ROBOT_LENGTH * 1.5;
            TrcPose2D target = new TrcPose2D(RobotInfo.TARGET_X_POS, targetY);
            TrcPose2D middle = new TrcPose2D(start.x, target.y);
            return startPosition == FrcAuto.StartPosition.IN_VISION ?
                createPath(120, start, target) :
                createPath(120, start, middle, target);
        }
        else
        {
            TrcPose2D target = new TrcPose2D(RobotInfo.TRENCH_RUN_X_POS - 24, -50, -15);
            TrcPose2D middle = new TrcPose2D(start.x - 15, start.y);
            return createPath(start, middle, target);
        }
    }

    public TrcPath createPickupPath(TrcPose2D start)
    {
        dbgTrace.traceInfo(instanceName + ".createToPickupPath", "[%.3f] Creating to pickup path with start=%s",
            TrcUtil.getModeElapsedTime(), start);
        TrcPose2D target = new TrcPose2D(RobotInfo.TRENCH_RUN_X_POS, -158.63 - 12);
        if (startPosition == FrcAuto.StartPosition.RIGHT_WALL)
        {
            TrcPose2D middle = new TrcPose2D(RobotInfo.TRENCH_RUN_X_POS, -100, 0);
            return createPath(20, start, middle, target);
        }
        else if (startPosition == FrcAuto.StartPosition.IN_VISION)
        {
            double x = TrcUtil.average(RobotInfo.TRENCH_RUN_X_POS, RobotInfo.TARGET_X_POS);
            TrcPose2D middle1 = new TrcPose2D(x, -50);
            TrcPose2D middle2 = new TrcPose2D(RobotInfo.TRENCH_RUN_X_POS, -50);
            TrcPath path = createPath(15, start, middle1, middle2, target);
            TrcWaypoint last = path.getWaypoint(path.getSize() - 1);
            Arrays.stream(path.getAllWaypoints()).filter(wp -> Math.abs(wp.x - last.x)<12).forEach(wp -> wp.velocity = 40);
            path.inferTimeSteps();
            dbgTrace.traceInfo(instanceName + ".createPickupPath", "With edited velocities:");
            for (TrcWaypoint waypoint : path.getAllWaypoints())
            {
                dbgTrace.traceInfo(instanceName + ".createPickupPath", "\t%s", waypoint.toString());
            }
            if (robot != null)
            {
                robot.purePursuit.setMoveOutputLimit(0.4);
            }
            return path;
        }
        else
        {
            TrcPose2D middle = new TrcPose2D(target.x, start.y);
            return createPath(20, start, middle, target);
        }
    }

    public TrcPath createToShoot2Path(TrcPose2D start)
    {
        dbgTrace.traceInfo(instanceName + ".createToShoot2Path", "[%.3f] Creating to shoot 2 path with start=%s",
            TrcUtil.getModeElapsedTime(), start);
        TrcPose2D target = new TrcPose2D(RobotInfo.TARGET_X_POS + 39.16 / 2, -60);
        return createPath(140, start, target);
    }

    private void localizeAtStart()
    {
        double x = startPosition != FrcAuto.StartPosition.CUSTOM ?
            startPosition.getXPos() :
            HalDashboard.getNumber(FrcAuto.CUSTOM_XPOS_KEY, 0);
        double y = -RobotInfo.ROBOT_LENGTH / 2;
        double angle = 0;
        //        double y = RobotInfo.INITIATION_LINE_TO_ALLIANCE_WALL - robot.alignment.getShortestDistanceToWall()
        //            - RobotInfo.ROBOT_LENGTH / 2;
        //        double angle = -robot.alignment.getAngleToWall();
        if (startPosition == FrcAuto.StartPosition.IN_VISION)
        {
            // if no vision, assume perfectly centered to goal
            FrcRemoteVisionProcessor.RelativePose pose = robot.vision.getLastPose();
            if (pose != null)
            {
                double theta = robot.vision.vision.getHeading();
                double yDist = RobotInfo.INITIATION_LINE_TO_ALLIANCE_WALL + 4;
                double xDist = Math.tan(Math.toRadians(theta)) * yDist;
                x = RobotInfo.TARGET_X_POS - xDist;
                dbgTrace.traceInfo(instanceName + ".localizeAtStart", "Vison xDist=%.2f", xDist);
                //                x = RobotInfo.TARGET_X_POS;
            }
        }
        else if (startPosition == FrcAuto.StartPosition.RIGHT_WALL)
        {
            x = startPosition.getXPos();
            y = -RobotInfo.ROBOT_LENGTH / 2;
            angle = 0;
        }
        TrcPose2D pose = new TrcPose2D(x, y, angle);
        dbgTrace
            .traceInfo(instanceName + ".localizeAtStart", "[%.3f] Localizing to pose: %s", TrcUtil.getModeElapsedTime(),
                pose.toString());
        robot.driveBase.setFieldPosition(pose);
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        if (!sm.isEnabled())
            return true;
        HalDashboard
            .putNumber("RobotVel", TrcUtil.magnitude(robot.driveBase.getXVelocity(), robot.driveBase.getYVelocity()));
        State state = sm.checkReadyAndGetState();
        if (state != null)
        {
            dbgTrace.traceInfo(instanceName + ".cmdPeriodic", "[%.3f] CurrState=%s", TrcUtil.getModeElapsedTime(),
                state.name());
            TrcPath path;
            switch (state)
            {
                case DELAY:
                    robot.intake.extendIntake(); // lower it to release tension in gravity comp
                    if (delay > 0)
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.LOCALIZE);
                    }
                    else
                    {
                        sm.setState(State.LOCALIZE);
                    }
                    break;

                case LOCALIZE:
                    localizeAtStart();
                    sm.setState(startPosition != FrcAuto.StartPosition.IN_VISION ? State.MOVE_TO_SHOOT : State.SHOOT);
                    break;

                case MOVE_TO_SHOOT:
                    path = createToShootPath(robot.driveBase.getFieldPosition());
                    robot.shooter.setFlywheelVelocity(RobotInfo.FLYWHEEL_HIGH_SPEED);
                    robot.shooter.setPitch(0);
                    robot.purePursuit.setMoveOutputLimit(startPosition == FrcAuto.StartPosition.RIGHT_WALL ? 0.2 : 0.6);
                    robot.purePursuit.setFollowingDistance(12);
                    robot.purePursuit.start(path, event, 4);
                    sm.waitForSingleEvent(event, State.SHOOT);
                    break;

                case SHOOT:
                    robot.autoShooter.shoot(instanceName, 3, 6, TaskAutoShooter.Mode.BOTH, event, true);
                    State nextState;
                    if (afterAction != AfterAction.NOTHING)
                    {
                        nextState = State.PICKUP;
                    }
                    else if (startPosition == FrcAuto.StartPosition.IN_VISION)
                    {
                        nextState = State.MOVE_OFF_LINE;
                    }
                    else
                    {
                        nextState = State.DONE;
                    }
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case MOVE_OFF_LINE:
                    robot.pidDrive.setAbsoluteTargetPose(robot.driveBase.getFieldPosition());
                    robot.pidDrive.setAbsoluteYTarget(-RobotInfo.ROBOT_LENGTH, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case PICKUP:
                    robot.shooter.stowShooter();
                    robot.purePursuit.setMoveOutputLimit(0.22);
                    path = createPickupPath(robot.driveBase.getFieldPosition());
                    robot.intake.intakeMultiple(true, 0.8, 0.8);
                    robot.purePursuit.start(path, event, 10);
                    sm.waitForSingleEvent(event,
                        afterAction == AfterAction.INTAKE ? State.DONE : State.MOVE_TO_SHOOT_2);
                    break;

                case MOVE_TO_SHOOT_2:
                    robot.intake.stopIntake(false);
                    //                    robot.shooter.setPitch(RobotInfo.FLYWHEEL_HIGH_ANGLE);
                    //                    robot.shooter.setFlywheelVelocity(RobotInfo.FLYWHEEL_HIGH_SPEED);
                    path = createToShoot2Path(robot.driveBase.getFieldPosition());
                    robot.purePursuit.setMoveOutputLimit(0.8);
                    robot.purePursuit.start(path, event, 4);
                    sm.waitForSingleEvent(event, State.SHOOT_2);
                    break;

                case SHOOT_2:
                    robot.autoShooter.shoot(instanceName, 3, 0, TaskAutoShooter.Mode.BOTH, event, true);
                    sm.waitForSingleEvent(event, State.WAIT);
                    break;

                case WAIT:
                    timer.set(0.2, event); // wait for last ball
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                    robot.globalTracer.traceInfo(instanceName + ".cmdPeriodic", "[%.3f] Finished auto! Pose=%s",
                        TrcUtil.getModeElapsedTime(), robot.driveBase.getFieldPosition());
                    cancel();
                    return true;
            }
        }
        return false;
    }

    private boolean relocalizeWithVision()
    {
        FrcRemoteVisionProcessor.RelativePose relPose = robot.vision.getLastPose();
        if (relPose == null)
            return false;
        TrcPose2D p = new TrcPose2D(relPose.x, relPose.y);
        double heading = robot.driveBase.getHeading();
        TrcPose2D inFieldFrame = p.relativeTo(new TrcPose2D(0, 0, -heading));
        TrcPose2D pose = new TrcPose2D(RobotInfo.TARGET_X_POS - inFieldFrame.x,
            RobotInfo.INITIATION_LINE_TO_ALLIANCE_WALL - inFieldFrame.y, heading);
        dbgTrace.traceInfo(instanceName + ".relocalizeWithVision", "CameraRelPose=%s,NewPose=%s", p.toString(),
            pose.toString());
        robot.driveBase.setFieldPosition(pose);
        return true;
    }

    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }

    @Override
    public void cancel()
    {
        sm.stop();
        robot.intake.stopIntake();
        robot.shooter.stopFlywheel();
        robot.shooter.stowShooter();
        robot.intake.setSpacingDistance(4);
    }
}
