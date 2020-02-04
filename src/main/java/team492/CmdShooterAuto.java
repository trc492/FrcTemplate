package team492;

import frclib.FrcRemoteVisionProcessor;
import trclib.TrcEvent;
import trclib.TrcPath;
import trclib.TrcPose2D;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;
import trclib.TrcUtil;

public class CmdShooterAuto implements TrcRobot.RobotCommand
{
    private static final String instanceName = "CmdShooterAuto";
    private static final double relocalizationTimeout = 0.5;

    private enum State
    {
        DELAY, MOVE_TO_SHOOT, SHOOT, RELOCALIZE, PICKUP, MOVE_TO_SHOOT_2, SHOOT_2, DONE
    }

    public enum AfterAction
    {
        NOTHING, INTAKE, INTAKE_AND_SHOOT;
    }

    private Robot robot;
    private TrcStateMachine<State> sm;
    private Double relocalizationTimedOutTime = null;
    private TrcEvent event;
    private TrcTimer timer;
    private double delay;
    private AfterAction afterAction;

    public CmdShooterAuto(Robot robot)
    {
        this.robot = robot;
        sm = new TrcStateMachine<>(instanceName + ".sm");
        event = new TrcEvent(instanceName + ".event");
        timer = new TrcTimer(instanceName + ".timer");
    }

    public void start(double delay, AfterAction afterAction)
    {
        this.delay = delay;
        this.afterAction = afterAction;
        sm.start(State.DELAY);
    }

    private TrcPath createToShootPath()
    {
        double targetY = -RobotInfo.ROBOT_LENGTH - 10;
        TrcPose2D target = new TrcPose2D(RobotInfo.TARGET_X_POS, targetY, 0);
        return null;
    }

    private TrcPath createPickupPath()
    {
        throw new IllegalStateException("Not implemented yet!"); // TODO: implement
    }

    private TrcPath createToShoot2Path()
    {
        throw new IllegalStateException("Not implemented yet!"); // TODO: implement
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        if (!sm.isEnabled())
            return true;
        State state = sm.checkReadyAndGetState();
        if (state != null)
        {
            TrcPath path;
            switch (state)
            {
                case DELAY:
                    robot.shooter.setFlywheelVelocity(RobotInfo.FLYWHEEL_HIGH_SPEED);
                    robot.shooter.setPitch(RobotInfo.FLYWHEEL_HIGH_ANGLE);
                    if (delay > 0)
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.MOVE_TO_SHOOT);
                    }
                    else
                    {
                        sm.setState(State.MOVE_TO_SHOOT);
                    }
                    break;

                case MOVE_TO_SHOOT:
                    if (robot.vision.getLastPose() != null)
                    {
                        sm.setState(State.SHOOT);
                    }
                    else
                    {
                        path = createToShootPath();
                        robot.purePursuit.start(path, event, 4);
                        sm.waitForSingleEvent(event, State.SHOOT);
                    }
                    break;

                case SHOOT:
                    robot.autoShooter.shoot(instanceName, robot.getNumBalls(), 2, TaskAutoShooter.Mode.BOTH, event);
                    relocalizationTimedOutTime = null;
                    sm.waitForSingleEvent(event, State.RELOCALIZE);
                    break;

                case RELOCALIZE:
                    if (relocalizationTimedOutTime == null)
                    {
                        relocalizationTimedOutTime = TrcUtil.getCurrentTime() + relocalizationTimeout;
                    }
                    if (relocalizeWithVision() || TrcUtil.getCurrentTime() >= relocalizationTimedOutTime)
                    {
                        sm.setState(afterAction == AfterAction.NOTHING ? State.DONE : State.PICKUP);
                    }
                    break;

                case PICKUP:
                    robot.shooter.setPitch(0);
                    path = createPickupPath();
                    robot.intake.intakeMultiple();
                    robot.purePursuit.start(path, event, 5);
                    sm.waitForSingleEvent(event,
                        afterAction == AfterAction.INTAKE ? State.DONE : State.MOVE_TO_SHOOT_2);
                    break;

                case MOVE_TO_SHOOT_2:
                    robot.intake.stopIntake();
                    robot.shooter.setPitch(RobotInfo.FLYWHEEL_HIGH_ANGLE);
                    robot.shooter.setFlywheelVelocity(RobotInfo.FLYWHEEL_HIGH_SPEED);
                    path = createToShoot2Path();
                    robot.purePursuit.start(path, event, 4);
                    sm.waitForSingleEvent(event, State.SHOOT);
                    break;

                case SHOOT_2:
                    robot.autoShooter.shoot(instanceName, robot.getNumBalls(), 2, TaskAutoShooter.Mode.BOTH, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
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
        robot.shooter.stopFlywheel();
        robot.shooter.setPitch(0);
    }
}
