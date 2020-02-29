package team492;

import frclib.FrcRemoteVisionProcessor;
import hallib.HalDashboard;
import org.apache.commons.math3.linear.RealVector;
import trclib.TrcEvent;
import trclib.TrcOwnershipManager;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

public class TaskAutoShooter
{
    private static final double VEL_TOLERANCE = 7; // in/sec
    private static final double HEADING_TOLERANCE = 1.2; // deg

    private static final double VEL_FUDGE_FACTOR = 1.2;
    private static final double ANGLE_FUDGE_FACTOR = 1.0;
    public static final double TARGET_OFFSET = -2.7;
    private boolean lockInPlace;

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
    private boolean releaseConveyor, releaseDriveBase;
    private String owner;
    private double nextBallShootTime = 0;
    private boolean isAligned = false;

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
        headingPid.setTarget(TARGET_OFFSET);

        HalDashboard.putNumber("HeadingErr", 0);
    }

    private double getAngleFromWall()
    {
        return robot.preferences.useVision ? -robot.vision.vision.getHeading() : 0;
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
            robot.driveBase.stop(owner);
            if (releaseDriveBase)
            {
                robot.driveBase.releaseExclusiveAccess(owner);
                robot.setAntiDefenseEnabled(owner, false);
            }
        }
        if (shouldShoot())
        {
            robot.conveyor.stop(owner);
            if (releaseConveyor)
            {
                robot.conveyor.releaseExclusiveAccess(owner);
            }
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
        if (pose != null && !(isAligned && lockInPlace))
        {
            // TODO: Remove the offset of the goal
            RealVector traj = TrajectoryCalculator.calculateWithArmWithDrag(TrcUtil
                .createVector((robot.vision.getTargetDepth() + RobotInfo.CAMERA_Y_OFFSET_TO_PIVOT) * 0.84,
                    RobotInfo.HIGH_TARGET_HEIGHT - RobotInfo.PIVOT_HEIGHT + 6));
            if (traj != null)
            {
                // TODO: Change to linear fudge factor?
                this.traj = traj;
                double velTarget = traj.getEntry(0) + 55;
                double angleTarget = traj.getEntry(1) * ANGLE_FUDGE_FACTOR;
                traj.setEntry(0, velTarget);
                traj.setEntry(1, angleTarget);
                robot.shooter.setFlywheelVelocity(velTarget);
                robot.shooter.setPitch(angleTarget);
            }
        }
        if (shouldAlign())
        {
            if (isAligned && lockInPlace)
            {
                robot.driveBase.stop(owner);
                robot.setAntiDefenseEnabled(owner, true);
            }
            else
            {
                double xPower = robot.getXInput();
                double yPower = robot.getYInput();
                double rotPower = headingPid.getOutput();
                HalDashboard.putNumber("HeadingErr", headingPid.getError());
                robot.driveBase.holonomicDrive(owner, xPower, yPower, rotPower, robot.getDriveGyroAngle());
                isAligned = headingPid.isOnTarget();
                robot.globalTracer
                    .traceInfo(instanceName + ".shooterTask", "Shooting alignment active - x=%.2f,y=%.2f,rot=%.2f",
                        xPower, yPower, rotPower);
            }
        }
        if (traj != null)
        {
            boolean ready = readyToShoot();
            robot.globalTracer.traceInfo(instanceName + ".shooterTask",
                "[%.3f] Shooting autoshoot active - event=%b,balls=%d,ready=%b", TrcUtil.getModeElapsedTime(),
                event.isSignaled(), ballsToShoot, ready);
            if (event.isSignaled())
            {
                ballsToShoot--;
                if (ballsToShoot == 0)
                {
                    stop();
                    return;
                }
                nextBallShootTime = TrcUtil.getCurrentTime() + 0.2;
                event.clear();
            }
            if (ready)
            {
                if (shouldShoot() && !robot.conveyor.isShooting())
                {
                    robot.conveyor.shoot(owner, event);
                }
            }
        }
    }

    public double getTargetPitch()
    {
        return traj == null ? 0 : traj.getEntry(1);
    }

    public double getTargetVel()
    {
        return traj == null ? 0 : traj.getEntry(0);
    }

    private boolean readyToShoot()
    {
        boolean velReady = Math.abs(flywheelError()) <= VEL_TOLERANCE;
        boolean pitchReady = robot.shooter.pitchOnTarget();
        FrcRemoteVisionProcessor.RelativePose pose = robot.vision.getLastPose();
        boolean headingReady = isAligned;//pose != null && Math.abs(robot.vision.getLastPose().theta) <= HEADING_TOLERANCE;
        double currTime = TrcUtil.getCurrentTime();
        boolean timeReady = currTime >= nextBallShootTime;
        robot.globalTracer.traceInfo(instanceName + ".readyToShoot",
            "[%.3f] Shooter readiness: vel=%b,pitch=%b,heading=%b,time=%b - headingErr=%.1f",
            TrcUtil.getModeElapsedTime(), velReady, pitchReady, headingReady, timeReady, headingPid.getError());
        return velReady && pitchReady && headingReady && timeReady;
    }

    private double flywheelError()
    {
        return robot.shooter.flywheel.motor.getClosedLoopError() * RobotInfo.FLYWHEEL_INCHES_PER_TICK / 0.1;
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
        shoot(instanceName, 0, 0, Mode.ALIGN_ONLY, null, false);
    }

    public void shoot()
    {
        shoot(robot.getNumBalls(), robot.getNumBalls()); // 1 sec per ball
    }

    public void shoot(int numBalls, double timeout)
    {
        shoot(instanceName, numBalls, timeout, Mode.BOTH, null, true);
    }

    public void shoot(String owner, int numBalls, double timeout, Mode mode, TrcEvent onFinishedEvent,
        boolean lockInPlace)
    {
        if (onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }
        if (isActive())
        {
            return; // don't interrupt a current operation
        }
        if (owner == null)
        {
            owner = this.instanceName;
        }
        this.mode = mode;
        this.lockInPlace = lockInPlace;
        isAligned = false;
        releaseDriveBase = !TrcOwnershipManager.getInstance().hasOwnership(owner, robot.driveBase);
        releaseConveyor = !TrcOwnershipManager.getInstance().hasOwnership(owner, robot.conveyor);
        if (shouldAlign() && !robot.driveBase.acquireExclusiveAccess(owner))
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.cancel();
            }
            robot.globalTracer
                .traceErr(owner + ".shoot", "Unable to acquire exclusive access of drivebase! CurrOwner=%s",
                    TrcOwnershipManager.getInstance().getOwner(robot.driveBase));
            return;
        }
        if (shouldShoot() && !robot.conveyor.acquireExclusiveAccess(owner))
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.cancel();
            }
            robot.globalTracer
                .traceErr(owner + ".shoot", "Unable to acquire exclusive access of conveyor! CurrOwner=%s",
                    TrcOwnershipManager.getInstance().getOwner(robot.conveyor));
            return;
        }
        this.owner = owner;
        timedOutTime = timeout == 0.0 ? Double.POSITIVE_INFINITY : TrcUtil.getCurrentTime() + timeout;
        ballsToShoot = numBalls;
        this.onFinishedEvent = onFinishedEvent;
        if (onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }
        this.event.clear();
        headingPid.setTarget(TARGET_OFFSET);
        taskObject.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
    }
}
