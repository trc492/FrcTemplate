package team492;

import com.playingwithfusion.TimeOfFlight;
import frclib.FrcCANTimeOfFlight;
import frclib.FrcLaserShark;
import frclib.FrcMedianFilter;
import trclib.TrcDistanceSensor;
import trclib.TrcFilter;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

public class WallAlignment
{
    private static boolean USE_LASER_SHARK = false;

    private TrcDistanceSensor leftLidar, rightLidar;
    private TrcFilter leftFilter, rightFilter;
    private TrcTaskMgr.TaskObject lidarTaskObj;

    public WallAlignment()
    {
        if (USE_LASER_SHARK)
        {
            leftLidar = new FrcLaserShark("LeftLidar", RobotInfo.DIO_LEFT_LIDAR);
            rightLidar = new FrcLaserShark("RightLidar", RobotInfo.DIO_RIGHT_LIDAR);
        }
        else
        {
            FrcCANTimeOfFlight leftCan = new FrcCANTimeOfFlight("LeftLidar", RobotInfo.CANID_LEFT_LIDAR);
            leftCan.identifySensor();
            this.leftLidar = leftCan;
            System.out.println("LeftCAN Firmware: " + leftCan.getFirmwareVersion());
            rightLidar = new FrcCANTimeOfFlight("RightLidar", RobotInfo.CANID_RIGHT_LIDAR);
        }

        leftFilter = new FrcMedianFilter("LeftLidarFilter", 5);
        rightFilter = new FrcMedianFilter("RightLidarFilter", 5);

        lidarTaskObj = TrcTaskMgr.getInstance().createTask("LidarTask", this::lidarTask);
    }

    private void lidarTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        // Don't cache the data, just keep reading to keep the filters up to date
        leftFilter.filterData(leftLidar.getDistanceInches());
        rightFilter.filterData(rightLidar.getDistanceInches());
    }

    public void enableRanging()
    {
        lidarTaskObj.registerTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        leftFilter.reset();
        rightFilter.reset();
    }

    public void disableRanging()
    {
        lidarTaskObj.unregisterTask();
        leftFilter.reset();
        rightFilter.reset();
    }

    public double getLeftDistance()
    {
        double distance = leftLidar.getDistanceInches();
        if (lidarTaskObj.isRegistered())
        {
            distance = leftFilter.filterData(distance);
        }
        return distance + RobotInfo.LIDAR_SENSOR_Y_OFFSET;
    }

    public double getRightDistance()
    {
        double distance = rightLidar.getDistanceInches();
        if (lidarTaskObj.isRegistered())
        {
            distance = rightFilter.filterData(distance);
        }
        return distance + RobotInfo.LIDAR_SENSOR_Y_OFFSET;
    }

    public double getShortestDistanceToWall()
    {
        return Math.cos(Math.toRadians(getAngleToWall())) * getForwardDistanceToWall();
    }

    public double getForwardDistanceToWall()
    {
        return TrcUtil.average(getLeftDistance(), getRightDistance());
    }

    public double getAngleToWall()
    {
        double l = getLeftDistance();
        double r = getRightDistance();
        double theta = Math.atan2(l - r, RobotInfo.LIDAR_INTER_SENSOR_DIST);
        return Math.toDegrees(theta);
    }
}
