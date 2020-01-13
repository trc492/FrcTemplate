package team492;

import frclib.FrcCANTimeOfFlight;
import frclib.FrcLaserShark;
import trclib.TrcDistanceSensor;
import trclib.TrcUtil;

public class WallAlignment
{
    private static boolean USE_LASER_SHARK = false;

    private TrcDistanceSensor leftLidar, rightLidar;

    public WallAlignment()
    {
        if (USE_LASER_SHARK)
        {
            leftLidar = new FrcLaserShark("LeftLidar", RobotInfo.LEFT_LIDAR);
            rightLidar = new FrcLaserShark("RightLidar", RobotInfo.RIGHT_LIDAR);
        }
        else
        {
            leftLidar = new FrcCANTimeOfFlight("LeftLidar", 11);
            rightLidar = new FrcCANTimeOfFlight("RightLidar", 12);
        }
    }

    public double getLeftDistance()
    {
        return leftLidar.getDistanceInches() + RobotInfo.LIDAR_SENSOR_Y_OFFSET;
    }

    public double getRightDistance()
    {
        return rightLidar.getDistanceInches() + RobotInfo.LIDAR_SENSOR_Y_OFFSET;
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
