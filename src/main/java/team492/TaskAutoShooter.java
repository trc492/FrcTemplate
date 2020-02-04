package team492;

import frclib.FrcRemoteVisionProcessor;
import org.apache.commons.math3.linear.RealVector;
import trclib.TrcEvent;
import trclib.TrcOwnershipManager;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

public class TaskAutoShooter
{
    private static final double VEL_TOLERANCE = 15; // in/sec
    private static final double PITCH_TOLERANCE = 5; // deg
    private static final double HEADING_TOLERANCE = 5; // deg

    public enum Mode
    {
        ALIGN_ONLY, SHOOT_ONLY, BOTH
    }

    private final String instanceName = "AutoShooter";

    private Robot robot;
    private TrcTaskMgr.TaskObject taskObject;
    private TrcEvent event, onFinishedEvent;
    private double timedOutTime;
    private RealVector traj;
    private int ballsToShoot;
    private TrcPidController headingPid;
    private Mode mode;

    public TaskAutoShooter(Robot robot)
    {
        this.robot = robot;
        taskObject = TrcTaskMgr.getInstance().createTask(instanceName + ".shooterTask", this::shooterTask);

        event = new TrcEvent(instanceName + ".event");
        TrcPidController.PidCoefficients headingPidCoeff = new TrcPidController.PidCoefficients(RobotInfo.GYRO_TURN_KP,
            RobotInfo.GYRO_TURN_KI, RobotInfo.GYRO_TURN_KD);
        headingPid = new TrcPidController(instanceName + "HeadingController", headingPidCoeff, HEADING_TOLERANCE,
            this::getAngleFromWall);
        headingPid.setAbsoluteSetPoint(true);
        headingPid.setTarget(0.0);
    }

    private double getAngleFromWall()
    {
        FrcRemoteVisionProcessor.RelativePose pose = robot.preferences.useVision ? robot.vision.getLastPose() : null;
        return pose != null ? -pose.theta : 0.0; // make negative so control effort is positive
    }

    private void stop()
    {
        if (onFinishedEvent != null)
        {
            onFinishedEvent.set(true);
            onFinishedEvent = null;
        }
        taskObject.unregisterTask();
        robot.shooter.stopFlywheel();
        headingPid.reset();
        if (shouldAlign())
        {
            robot.driveBase.stop(instanceName);
            robot.driveBase.releaseExclusiveAccess(instanceName);
        }
        if (shouldShoot())
        {
            robot.conveyor.stop(instanceName);
            robot.conveyor.releaseExclusiveAccess(instanceName);
        }
    }

    private void shooterTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        if (TrcUtil.getCurrentTime() >= timedOutTime || (shouldShoot() && ballsToShoot <= 0))
        {
            stop();
            return;
        }
        FrcRemoteVisionProcessor.RelativePose pose = robot.vision.getLastPose();
        if (pose != null)
        {
            RealVector traj = TrajectoryCalculator.calculateWithArmWithDrag(
                TrcUtil.createVector(pose.r, RobotInfo.HIGH_TARGET_HEIGHT - RobotInfo.PIVOT_HEIGHT));
            if (traj != null)
            {
                this.traj = traj;
                robot.shooter.setFlywheelVelocity(traj.getEntry(0));
                robot.shooter.setPitch(traj.getEntry(1));
            }
        }
        if (shouldAlign())
        {
            robot.driveBase.holonomicDrive(instanceName, robot.getXInput(), robot.getYInput(), headingPid.getOutput(),
                robot.getFieldOriented() ? robot.driveBase.getHeading() : 0.0);
        }
        if (traj != null)
        {
            if (event.isSignaled())
            {
                ballsToShoot--;
                event.clear();
            }
            if (readyToShoot(traj))
            {
                robot.ledIndicator.setShooterReady(true);
                if (shouldShoot())
                {
                    robot.conveyor.shoot(instanceName, event);
                }
            }
            else
            {
                robot.ledIndicator.setShooterReady(false);
            }
        }
    }

    private boolean readyToShoot(RealVector traj)
    {
        double velTarget = traj.getEntry(0);
        boolean velReady = Math.abs(robot.shooter.getFlywheelVelocity() - velTarget) <= VEL_TOLERANCE;
        double pitchTarget = traj.getEntry(1);
        boolean pitchReady = Math.abs(robot.shooter.getPitch() - pitchTarget) <= PITCH_TOLERANCE;
        FrcRemoteVisionProcessor.RelativePose pose = robot.vision.getLastPose();
        boolean headingReady = pose != null && Math.abs(robot.vision.getLastPose().theta) <= HEADING_TOLERANCE;
        return velReady && pitchReady && headingReady;
    }

    public boolean isActive()
    {
        return taskObject.isRegistered();
    }

    public void cancel()
    {
        if (isActive())
        {
            stop();
        }
    }

    private boolean shouldAlign()
    {
        return mode == Mode.ALIGN_ONLY || mode == Mode.BOTH;
    }

    private boolean shouldShoot()
    {
        return mode == Mode.SHOOT_ONLY || mode == Mode.BOTH;
    }

    public void trackTarget()
    {
        shoot(instanceName, 0, 0, Mode.ALIGN_ONLY, null);
    }

    public void shoot()
    {
        shoot(robot.getNumBalls(), robot.getNumBalls()); // 1 sec per ball
    }

    public void shoot(int numBalls, double timeout)
    {
        shoot(instanceName, numBalls, timeout, Mode.BOTH, null);
    }

    public void shoot(String instanceName, int numBalls, double timeout, Mode mode, TrcEvent onFinishedEvent)
    {
        if (onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }
        if (isActive())
        {
            return; // don't interrupt a current operation
        }
        if (instanceName == null)
        {
            instanceName = this.instanceName;
        }
        this.mode = mode;
        if (shouldAlign() && !robot.driveBase.acquireExclusiveAccess(instanceName))
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.cancel();
            }
            robot.globalTracer
                .traceErr("TaskAutoShooter.shoot", "Unable to acquire exclusive access of drivebase! CurrOwner=%s",
                    TrcOwnershipManager.getInstance().getOwner(robot.driveBase));
            return;
        }
        if (shouldShoot() && !robot.conveyor.acquireExclusiveAccess(instanceName))
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.cancel();
            }
            robot.globalTracer
                .traceErr("TaskAutoShooter.shoot", "Unable to acquire exclusive access of conveyor! CurrOwner=%s",
                    TrcOwnershipManager.getInstance().getOwner(robot.conveyor));
            return;
        }
        timedOutTime = timeout == 0.0 ? Double.POSITIVE_INFINITY : TrcUtil.getCurrentTime() + timeout;
        ballsToShoot = numBalls;
        this.onFinishedEvent = onFinishedEvent;
        if (onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }
        this.event.clear();
        headingPid.setTarget(0.0);
        taskObject.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
    }
}
