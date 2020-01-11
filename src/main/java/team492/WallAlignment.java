package team492;

import frclib.FrcLaserShark;
import trclib.TrcUtil;

public class WallAlignment
{
    private FrcLaserShark leftLidar, rightLidar;

    public WallAlignment()
    {
        leftLidar = new FrcLaserShark("LeftLidar", RobotInfo.LEFT_LIDAR);
        rightLidar = new FrcLaserShark("RightLidar", RobotInfo.RIGHT_LIDAR);
    }

    public double getShortestDistanceToWall()
    {
        return Math.cos(Math.toRadians(getAngleToWall())) * getForwardDistanceToWall();
    }

    public double getForwardDistanceToWall()
    {
        return TrcUtil.average(leftLidar.getDistanceInches(), rightLidar.getDistanceInches());
    }

    public double getAngleToWall()
    {
        double l = leftLidar.getDistanceInches();
        double r = rightLidar.getDistanceInches();
        double theta = Math.atan2(l - r, RobotInfo.LIDAR_INTER_SENSOR_DIST);
        return 90 - Math.toDegrees(theta);
    }
}
