/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.subsystems;

import frclib.drivebase.FrcDifferentialDrive;
import frclib.drivebase.FrcMecanumDrive;
import frclib.drivebase.FrcRobotDrive;
import frclib.drivebase.FrcSwerveDrive;
import teamcode.RobotParams;

/**
 * This class creates the appropriate Robot Drive Base according to the specified robot type.
 */
public class RobotBase
{
    private final FrcRobotDrive.RobotInfo robotInfo;
    private final FrcRobotDrive robotDrive;

    /**
     * Constructor: Create an instance of the object.
     */
    public RobotBase()
    {
        switch (RobotParams.Preferences.robotType)
        {
            case VisionOnly:
                robotInfo = new RobotParams.VisionOnlyParams();
                robotDrive = null;
                break;

            case DifferentialRobot:
                robotInfo = new RobotParams.DifferentialParams();
                robotDrive = RobotParams.Preferences.useDriveBase?
                    new FrcDifferentialDrive(robotInfo, RobotParams.Preferences.useExternalOdometry): null;
                break;

            case MecanumRobot:
                robotInfo = new RobotParams.MecanumParams();
                robotDrive = RobotParams.Preferences.useDriveBase?
                    new FrcMecanumDrive(robotInfo, RobotParams.Preferences.useExternalOdometry): null;
                break;

            case SwerveRobot:
                robotInfo = new RobotParams.SwerveParams();
                robotDrive = RobotParams.Preferences.useDriveBase?
                    new FrcSwerveDrive(
                        (RobotParams.SwerveParams) robotInfo, RobotParams.Preferences.useExternalOdometry): null;
                break;

            default:
                robotInfo = null;
                robotDrive = null;
                break;
        }
    }   //RobotBase

    /**
     * This method returns the created RobotInfo object.
     *
     * @return created robot info.
     */
    public FrcRobotDrive.RobotInfo getRobotInfo()
    {
        return robotInfo;
    }   //getRobotInfo

    /**
     * This method returns the created RobotBase object.
     *
     * @return created robot drive.
     */
    public FrcRobotDrive getRobotDrive()
    {
        return robotDrive;
    }   //getRobotDrive

}   //class RobotDrive