package team492;

import frclib.FrcRemoteVisionProcessor;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

public class TaskAutoAlign
{
    private static final double X_TOLERANCE = 5.0;
    private static final double Y_TOLERANCE = 3.0;
    private static final double ANGLE_TOLERANCE = 4.0;

    private static final double SHOOTER_ANGLE = 77; // approximate values to init
    private static final double SHOOTER_SPEED = 232; // approximate values to init

    private static final String instanceName = "AutoAlign";
    private Robot robot;
    private TrcTaskMgr.TaskObject taskObject;
    private TrcEvent event, onFinishedEvent;
    private TrcPidController xPid, yPid, rotPid;
    private double timedOutTime;

    public TaskAutoAlign(Robot robot)
    {
        this.robot = robot;
        taskObject = TrcTaskMgr.getInstance().createTask(instanceName + ".task", this::alignTask);

        event = new TrcEvent(instanceName + ".event");

        xPid = new TrcPidController(instanceName + ".xPid", robot.encoderXPidCtrl.getPidCoefficients(), 4.0,
            this::getXDistanceFromTarget);
        xPid.setAbsoluteSetPoint(true);

        yPid = new TrcPidController(instanceName + ".yPid", robot.encoderYPidCtrl.getPidCoefficients(), 4,
            this::getYDistanceFromWall);
        yPid.setAbsoluteSetPoint(true);

        rotPid = new TrcPidController(instanceName + ".rotPid", robot.gyroTurnPidCtrl.getPidCoefficients(), 5.0,
            this::getAngleFromWall);
        rotPid.setAbsoluteSetPoint(true);

        xPid.setTarget(0.0);
        yPid.setTarget(0.0);
        rotPid.setTarget(0.0);
    }

    private double getOutputScale(double taperLow, double taperHigh)
    {
        double x = Math.abs(getXDistanceFromTarget());
        if (x > taperHigh)
        {
            return 0;
        }
        else if (x > taperLow)
        {
            return 1 - TrcUtil.scaleRange(x, taperLow, taperHigh, 0, 1);
        }
        return 1;
    }

    private double getXDistanceFromTarget()
    {
        FrcRemoteVisionProcessor.RelativePose pose = robot.vision.getLastPose();
        if (pose == null)
            return 0.0;
        double theta = pose.theta + getAngleFromWall();
        return -Math.sin(Math.toRadians(theta)) * pose.r;
    }

    private double getYDistanceFromWall()
    {
        return -robot.alignment.getShortestDistanceToWall();
    }

    private double getAngleFromWall()
    {
        return -robot.alignment.getAngleToWall();
    }

    public boolean isActive()
    {
        return taskObject.isRegistered();
    }

    public void start(TrcEvent onFinishedEvent, double timeout)
    {
        if (onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }

        if (isActive())
        {
            return;
        }

        if (!robot.driveBase.acquireExclusiveAccess(instanceName))
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.cancel();
            }
            return;
        }

        this.onFinishedEvent = onFinishedEvent;
        timedOutTime = timeout == 0.0 ? Double.POSITIVE_INFINITY : TrcUtil.getCurrentTime() + timeout;
        event.clear();
        taskObject.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);

        // set initial values so the shooter takes less time
        robot.shooter.setFlywheelVelocity(SHOOTER_SPEED);
        robot.shooter.setPitch(SHOOTER_ANGLE);
    }

    private void stop()
    {
        if (onFinishedEvent != null)
        {
            onFinishedEvent.set(true);
            onFinishedEvent = null;
        }
        taskObject.unregisterTask();
        robot.autoShooter.cancel();
        robot.driveBase.stop();
        robot.driveBase.releaseExclusiveAccess(instanceName);
    }

    private void alignTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        if (event.isSignaled() || TrcUtil.getCurrentTime() >= timedOutTime)
        {
            stop();
            return;
        }

        double xOut = xPid.getOutput();
        double yOut = yPid.getOutput() * getOutputScale(0, 25);
        double rotOut = rotPid.getOutput() * getOutputScale(0, 20);
        robot.driveBase
            .holonomicDrive(instanceName, xOut, yOut, rotOut, getAngleFromWall());
        if (!robot.autoShooter.isActive() && Math.abs(xPid.getError()) <= X_TOLERANCE
            && Math.abs(yPid.getError()) <= Y_TOLERANCE && Math.abs(rotPid.getError()) <= ANGLE_TOLERANCE)
        {
            robot.autoShooter.shoot(5, 5.0, false, event);
        }
    }
}
