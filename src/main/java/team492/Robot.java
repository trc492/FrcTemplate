/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
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

package team492;

import java.util.Locale;

import edu.wpi.first.wpilibj.DriverStation;
import frclib.FrcRobotBase;
import hallib.HalDashboard;
import trclib.TrcMecanumDriveBase;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobotBattery;
import trclib.TrcRobot.RunMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends FrcRobotBase
{
    //
    // Robot preferences.
    //

    //
    // Global constants.
    //
    public static final String programName = "FrcTemplate";

    //
    // Global objects.
    //
    public final DriverStation ds = DriverStation.getInstance();
    public final HalDashboard dashboard = HalDashboard.getInstance();
    public TrcPidController.PidCoefficients tunePidCoeff;

    //
    // Inputs.
    //

    //
    // Sensors.
    //
    public TrcRobotBattery battery;

    //
    // DriveBase subsystem.
    //
    public TrcMecanumDriveBase driveBase;
    public TrcPidDrive pidDrive;

    //
    // Vision subsystem.
    //

    //
    // Miscellaneous subsystem.
    //

    //
    // FMS Match info.
    //

    private FrcAuto autoMode;

    /**
     * Constructor.
     */
    public Robot()
    {
        super(programName);
    }   //Robot

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit()
    {
        //
        // Create and initialize global objects.
        //

        //
        // Create and initialize inputs.
        //

        //
        // Create and initialize sensors.
        //

        //
        // Create and initialize DriveBase subsystem.
        //

        //
        // Create PID controllers for DriveBase PID drive.
        //

        //
        // Create and initialize Vision subsystem.
        //

        //
        // Create and initialize other subsystems.
        //

        //
        // AutoAssist commands.
        //

        //
        // Create Robot Modes.
        //
        autoMode = new FrcAuto(this);
        setupRobotModes(new FrcTeleOp(this), autoMode, new FrcTest(this), new FrcDisabled(this));

    }   //robotInit

    public void robotStartMode(RunMode runMode, RunMode prevMode)
    {
        //
        // Start subsystems.
        //

        //
        // Read FMS Match info.
        //

        //
        // Read Tune PID Coefficients if in TEST_MODE.
        //

        //
        // Start trace logging.
        //

    }   //robotStartMode

    public void robotStopMode(RunMode runMode, RunMode nextMode)
    {
        //
        // Stop subsystems.
        //

        //
        // Stop trace logging.
        //

    }   //robotStopMode

    public void traceStateInfo(double elapsedTime, String stateName, double xTarget, double yTarget, double turnTarget)
    {
        final String funcName = "traceStateInfo";
        StringBuilder msg = new StringBuilder();

        // msg.append(String
        //     .format("[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f", elapsedTime, stateName,
        //         driveBase.getXPosition(), xTarget, driveBase.getYPosition(), yTarget, driveBase.getHeading(),
        //         turnTarget));

        if (driveBase != null)
        {
            msg.append(String.format(Locale.US, "tag=\">>>>>\" state=\"%s\"", stateName));
            msg.append(
                String.format(Locale.US, " xPos=\"%6.2f\" xTarget=\"%6.2f\"", driveBase.getXPosition(), xTarget));
            msg.append(
                String.format(Locale.US, " yPos=\"%6.2f\" yTarget=\"%6.2f\"", driveBase.getYPosition(), yTarget));
            msg.append(String
                .format(Locale.US, " heading=\"%6.2f\" headingTarget=\"%6.2f\"", driveBase.getHeading(), turnTarget));
        }

        if (battery != null)
        {
            msg.append(String.format(",voltage=\"%5.2fV(%5.2fV)\"", battery.getVoltage(), battery.getLowestVoltage()));
        }

        globalTracer.logEvent(funcName, "StateInfo", "%s", msg);
    }   //traceStateInfo

}   //class Robot
