package team492;

import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;
import trclib.TrcWarpSpace;

public class TaskSnapToAngle
{
    private static final double HEADING_TOLERANCE = 2;
    private static double[] ANGLES = new double[] { 0, 90, 180, 270, 360 };

    private static final String instanceName = "TaskSnapToAngle";

    private Robot robot;
    private TrcPidController headingPid;
    private TrcTaskMgr.TaskObject taskObj;
    private TrcWarpSpace warpSpace;

    public TaskSnapToAngle(Robot robot)
    {
        this.robot = robot;
        TrcPidController.PidCoefficients headingPidCoeff = new TrcPidController.PidCoefficients(RobotInfo.GYRO_TURN_KP,
            RobotInfo.GYRO_TURN_KI, RobotInfo.GYRO_TURN_KD);
        headingPid = new TrcPidController(instanceName + ".HeadingController", headingPidCoeff, HEADING_TOLERANCE,
            robot.driveBase::getHeading);
        headingPid.setAbsoluteSetPoint(true);
        warpSpace = new TrcWarpSpace("SnapToAngle.warpSpace", 0, 360);

        taskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".TaskObj", this::controlTask);
    }

    private void controlTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        if (robot.getRotInput() != 0)
        {
            cancel();
            return;
        }
        robot.driveBase.holonomicDrive(instanceName, robot.getXInput(), robot.getYInput(), headingPid.getOutput(),
            robot.getFieldOriented() ? robot.driveBase.getHeading() : 0.0);
    }

    public void cancel()
    {
        robot.driveBase.stop();
        robot.driveBase.releaseExclusiveAccess(instanceName);
        headingPid.reset();
        taskObj.unregisterTask();
    }

    public void snapToAngle(double targetHeading)
    {
        if (!robot.driveBase.acquireExclusiveAccess(instanceName))
        {
            robot.globalTracer
                .traceErr(instanceName + ".snapToAngle", "Cannot acquire exclusive access of drivebase! Angle=%.1f",
                    targetHeading);
            return;
        }
        targetHeading = warpSpace.getOptimizedTarget(targetHeading, robot.driveBase.getHeading());
        robot.globalTracer.traceInfo(instanceName + ".snapToAngle", "Snapping to angle=%.1f, currAngle=%.1f", targetHeading, robot.driveBase.getHeading());
        headingPid.setTarget(targetHeading);
        taskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
    }

    public void snapToNearestAngle()
    {
        double heading = TrcUtil.modulo(robot.driveBase.getHeading(), 360.0);
        double targetHeading = ANGLES[0];
        for (double angle : ANGLES)
        {
            if (Math.abs(heading - angle) < Math.abs(heading - targetHeading))
            {
                targetHeading = angle;
            }
        }
        snapToAngle(targetHeading);
    }
}
